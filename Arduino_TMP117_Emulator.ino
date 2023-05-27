// TMP117 Emulator - For basic testing and implementation of TMP117 sensor
//
// by RTEK1000 - May/26/2023
//
// Ref. Arduino I2C slave: https://forum.arduino.cc/t/i2c-slave-transmitter-receiver-not-working/890321/6
//
// Ref. TMP117: https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library

#include <Wire.h>
#include <EEPROM.h>

#define sensors 4

#define debug_cfg 0 // set 1 to enable

// Use analog input (10 bits) to simulate temperature value
// Temperature should vary +/- 4 degrees around 20 degrees
// ((adc - 512) * TMP117_RESOLUTION) + 20.0; (This formula can be changed)
#define use_ADC_for_temp 1    // set 1 to enable

#if use_ADC_for_temp == 1
#define in1 A0 // real Arduino pin (A4/A5 are used for I2C)
#define in2 A1
#define in3 A2
#define in4 A3
#endif

#define use_LED_for_alarm 1    // set 1 to enable

#if use_LED_for_alarm == 1
#define LED 13 // real Arduino pin (A4/A5 are used for I2C)
#endif

// (Automatically populate with random data, if everything is 0xFF)
#define use_real_EEPROM 1    // set 1 to enable

// EEPROM has reduced number of writes, use with care
#define fill_real_EEPROM 0   // set 1 to enable - random

#define EEPROM_base_addr 0   // Arduino board real EEPROM // 48 bytes

// base values:
float analogTempC[sensors] = {25.7578125, 23.8203125, 28.34375, 27.0859375};

#define TMP117_RESOLUTION 0.0078125f

// EEPROM initial values:
uint8_t EEPROM1_init[sensors][2] = {{0x25, 0x81}, {0x03, 0x34}, {0x93, 0x82}, {0x78, 0x12}};
uint8_t EEPROM2_init[sensors][2] = {{0x24, 0x82}, {0x02, 0x35}, {0x92, 0x83}, {0x77, 0x13}};
uint8_t EEPROM3_init[sensors][2] = {{0x23, 0x83}, {0x01, 0x36}, {0x91, 0x84}, {0x76, 0x14}};

// Automatic change of temperature value (true/enabled)
const bool auto_change_temperature[sensors] = {true, false, false, false};

// Automatic change of temperature value (true/enabled) [random]
const bool rand_auto_change_temperature[sensors] = {false, true, false, false};

// Allows to bypass conversion mode for fixed time (just for testing)
const bool fixed_CONV_Data_Ready_speed[sensors] = {false, false, false, false};

// CONV[2:0] = 100 (1-Hz [1000ms] conversion cycle)
const uint16_t fixed_TMP117_CONV_Data_Ready_speed[sensors] = {1000, 1000, 1000, 1000};

// CONV[2:0] = 100 (1-Hz [1000ms] conversion cycle)
uint16_t TMP117_CONV_Data_Ready_speed[sensors] = {15, 15, 15, 15};

uint16_t prev_TMP117_CONV_Data_Ready_speed[sensors] = {0};

// AVG[1:0] = 00 (Datasheet page 28)
const int16_t AVG_00_table[8] = {15, 125, 250, 500, 1000, 4000, 8000, 16000};

volatile uint8_t new_TMP117_CONF_buff[sensors][2] = {{0x02, 0x20}, {0x02, 0x20}, {0x02, 0x20}, {0x02, 0x20}};

//const uint16_t TMP117_CONF_write_mask = 0x0FFE;
const uint8_t TMP117_CONF_write_mask_MSB = 0x0F;
const uint8_t TMP117_CONF_write_mask_LSB = 0xFE;

bool TMP117_EEPROM_UL_enabled[sensors] = {0, 0, 0, 0};

// Address of the registers. Datasheet page 25
enum TMP117_Register_Addr {
  TMP117_TEMP_RESULT = 0X00,
  TMP117_CONFIGURATION = 0x01,
  TMP117_T_HIGH_LIMIT = 0X02,
  TMP117_T_LOW_LIMIT = 0X03,
  TMP117_EEPROM_UL = 0X04,
  TMP117_EEPROM1 = 0X05,
  TMP117_EEPROM2 = 0X06,
  TMP117_TEMP_OFFSET = 0X07,
  TMP117_EEPROM3 = 0X08,
  TMP117_DEVICE_ID = 0X0F
};

enum TMP117_Configuration_bits {
  TMP117_CONF_Not_Used     = 0,  // bit 0 (Not used)
  TMP117_CONF_Soft_Reset   = 1,  // When set to 1 it triggers software reset with a duration of 2 ms
  TMP117_CONF_DR_Alert     = 2,  // ALERT pin select bit
  TMP117_CONF_POL          = 3,  // ALERT pin polarity bit
  TMP117_CONF_T_nA         = 4,  // Therm/alert mode select
  TMP117_CONF_AVG0         = 5,  // Conversion averaging modes
  TMP117_CONF_AVG1         = 6,  // Conversion averaging modes
  TMP117_CONF_CONV0        = 7,  // Conversion cycle bit
  TMP117_CONF_CONV1        = 8,  // Conversion cycle bit
  TMP117_CONF_CONV2        = 9,  // Conversion cycle bit
  TMP117_CONF_MOD0         = 10, // Set conversion mode
  TMP117_CONF_MOD1         = 11, // Set conversion mode
  TMP117_CONF_EEPROM_Busy  = 12, // EEPROM busy flag
  TMP117_CONF_Data_Ready   = 13, // Data ready flag
  TMP117_CONF_LOW_Alert    = 14, // Low Alert flag
  TMP117_CONF_HIGH_ALERT   = 15  // High Alert flag
};

typedef struct
{
  uint8_t MSB;
  uint8_t LSB;
}  Data16_type;

typedef struct
{
  Data16_type TEMP_RESULT = {uint8_t(0x80), uint8_t(0x00)};   // 8000h
  Data16_type CONFIGURATION = {uint8_t(0x02), uint8_t(0x20)}; // 0220h
  Data16_type T_HIGH_LIMIT = {uint8_t(0x60), uint8_t(0x00)};  // 6000h
  Data16_type T_LOW_LIMIT = {uint8_t(0x80), uint8_t(0x00)};   // 8000h
  Data16_type EEPROM_UL = {uint8_t(0x00), uint8_t(0x00)};     // 0000h
  Data16_type EEPROM1;                                        // xxxxh
  Data16_type EEPROM2;                                        // xxxxh
  Data16_type TEMP_OFFSET = {uint8_t(0x00), uint8_t(0x00)};   // 0000h
  Data16_type EEPROM3;                                        // xxxxh
  Data16_type DEVICE_ID = {uint8_t(0x01), uint8_t(0x17)};     // 0117h
}  RegisterBank1_type;

volatile RegisterBank1_type TMP117_RegBank[sensors];

volatile Data16_type new_EEPROM1[sensors];
volatile Data16_type new_EEPROM2[sensors];
volatile Data16_type new_EEPROM3[sensors];

volatile uint8_t get_TMP117_Register[sensors] = {0xFF, 0xFF, 0xFF, 0xFF};

volatile bool TMP117_CONF_Data_Ready_clear[sensors] = {false, false, false, false};

volatile bool TMP117_Alert_clear[sensors] = {false, false, false, false};

volatile uint8_t CURR_DEV = 0;

volatile int8_t rec_buff_index = -1;
volatile uint8_t rec_buff[10] = {0};

volatile int8_t rec_data_buff_index = -1;
volatile uint8_t rec_data_buff[10] = {0};

unsigned char c;

unsigned long millis0[sensors] = {0};

void print_interr_buff(bool debug);

void setup() {
#if use_ADC_for_temp == 1
  pinMode(in1, INPUT);
  pinMode(in2, INPUT);
  pinMode(in3, INPUT);
  pinMode(in4, INPUT);
#endif

#if use_LED_for_alarm == 1
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);
#endif

  Serial.begin(115200);

  Serial.println("\r\n\r\nTMP117 Emulator start");

  Wire.begin(0x48);
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // I2C Slave Promiscuous Mode (ATmega328 and others from this MCU family, maybe Mega2560 board)
  // TWAMR: TWI (Slave) Address Mask Register
  // https://stackoverflow.com/questions/34691478/arduino-as-slave-with-multiple-i2c-addresses
  TWAMR = (1 | 2) << 1; // 0x48, 0x49, 0x4A, 0x4B

  data_EEPROM_init();
}

void loop(void) {
  print_interr_buff(uint8_t(debug_cfg)); // 1: debug

  for (uint8_t sen = 0; sen < sensors; sen++) {
    temperature_update(sen);

    alarm_update(sen);

    EEPROM_update(sen);

    config_update(sen);

    // Conversion time update
    conv_time_update(sen);

    // Conv Ready update
    if ((millis() - millis0[sen]) > TMP117_CONV_Data_Ready_speed[sen]) {
      millis0[sen] = millis();

      setCONFbit(sen, TMP117_CONF_Data_Ready); // TMP117_CONF[sen] |= 1 << TMP117_CONF_Data_Ready;
    }
  }
}

void data_EEPROM_init(void) {
  if ((uint8_t(use_real_EEPROM) == 1) && (uint8_t(fill_real_EEPROM) == 1)) {
    for (uint8_t device = 0; device < sensors; device++) {
      fillEEPROM(device);
    }
  }

  if (uint8_t(use_real_EEPROM) == 1) {
    uint8_t tester = 0;
    for (uint8_t i = EEPROM_base_addr; i < (48 + EEPROM_base_addr); i++) {
      if (EEPROM.read(i) == 0xFF) {
        tester++;
      }
    }

    // All byte are 0xFF, fill random data
    if (tester == 48) {
      for (uint8_t device = 0; device < sensors; device++) {
        fillEEPROM(device);
      }
    }
  }

  for (uint8_t device = 0; device < sensors; device++) {
    if (uint8_t(use_real_EEPROM) == 1) {
      initFromEEPROM(device);
    } else {
      TMP117_RegBank[device].EEPROM1.MSB = EEPROM1_init[device][0];
      TMP117_RegBank[device].EEPROM1.LSB = EEPROM1_init[device][1];

      TMP117_RegBank[device].EEPROM2.MSB = EEPROM2_init[device][0];
      TMP117_RegBank[device].EEPROM2.LSB = EEPROM2_init[device][1];

      TMP117_RegBank[device].EEPROM3.MSB = EEPROM3_init[device][0];
      TMP117_RegBank[device].EEPROM3.LSB = EEPROM3_init[device][1];
    }
  }

  for (uint8_t i = 0; i < sensors; i++) {
    EEPROM_update(i);
  }
}

void print_interr_buff(uint8_t debug) {
  if (rec_buff_index >= 0) {
    if (debug == 1) {
      Serial.print(rec_buff_index, DEC);
      Serial.print(": TWDR 0x");
      Serial.println(rec_buff[rec_buff_index], HEX);
    }

    rec_buff_index--;
  }

  if (rec_data_buff_index >= 0) {
    if (debug == 1) {
      Serial.print(rec_data_buff_index, DEC);
      Serial.print(": Data 0x");
      Serial.println(rec_data_buff[rec_data_buff_index], HEX);
    }

    rec_data_buff_index--;
  }
}

void setCONFbit(uint8_t sen, uint16_t bit_pos) {
  if (bit_pos > 7) {
    TMP117_RegBank[sen].CONFIGURATION.MSB |= (1 << (bit_pos - 8));
  } else {
    TMP117_RegBank[sen].CONFIGURATION.LSB |= (1 << bit_pos);
  }
}

void resetCONFbit(uint8_t sen, uint16_t bit_pos) {
  if (bit_pos > 7) {
    TMP117_RegBank[sen].CONFIGURATION.MSB &= ~(1 << (bit_pos - 8));
  } else {
    TMP117_RegBank[sen].CONFIGURATION.LSB &= ~(1 << bit_pos);
  }
}

void temperature_update(uint8_t sen) {
  uint16_t adc[sensors] = {0, 0, 0, 0};

#if use_ADC_for_temp == 1
  adc[0] = analogRead(in1);
  adc[1] = analogRead(in2);
  adc[2] = analogRead(in3);
  adc[3] = analogRead(in4);
#endif

  if (TMP117_CONF_Data_Ready_clear[sen] == true) {
    TMP117_CONF_Data_Ready_clear[sen] = false;

    resetCONFbit(sen, TMP117_CONF_Data_Ready); //TMP117_CONF[sen] &= ~(uint16_t)(1 << TMP117_CONF_Data_Ready);

    if (uint8_t(use_ADC_for_temp) == 1) {
      analogTempC[sen] = ((adc[sen] - 512) * TMP117_RESOLUTION) + 20.0;
    } else if (auto_change_temperature[sen] == true) {
      if (analogTempC[sen] < 25.0) {
        analogTempC[sen] += TMP117_RESOLUTION;
      } else {
        analogTempC[sen] = 20.0;
      }
    } else if (rand_auto_change_temperature[sen] == true) {
      analogTempC[sen] = random(-1000, 1000) * TMP117_RESOLUTION + 25.0;
    }
  }

  uint16_t digitalTempC = int16_t(float(analogTempC[sen] / TMP117_RESOLUTION));

  TMP117_RegBank[sen].TEMP_RESULT.MSB = (digitalTempC >> 8) & 0xFF;
  TMP117_RegBank[sen].TEMP_RESULT.LSB = digitalTempC & 0xFF;
}

void alarm_update(uint8_t sen) {
  uint16_t t_H_alarm = 0;
  uint16_t t_L_alarm = 0;
  bool t_na_mod = 0;
  bool pin_pol = 0;
  bool dr_alert = 0;
  uint16_t temp_res = 0;
  bool H_alert = 0;
  bool L_alert = 0;
  bool dat_ready = 0;

  t_H_alarm = uint16_t(TMP117_RegBank[sen].T_HIGH_LIMIT.MSB << 8);
  t_H_alarm |= TMP117_RegBank[sen].T_HIGH_LIMIT.LSB;

  t_L_alarm = uint16_t(TMP117_RegBank[sen].T_LOW_LIMIT.MSB << 8);
  t_L_alarm |= TMP117_RegBank[sen].T_LOW_LIMIT.LSB;

  t_na_mod = (TMP117_RegBank[sen].CONFIGURATION.LSB >> 4) & 1;

  pin_pol = (TMP117_RegBank[sen].CONFIGURATION.LSB >> 3) & 1;

  dr_alert = (TMP117_RegBank[sen].CONFIGURATION.LSB >> 2) & 1;

  dat_ready = (TMP117_RegBank[sen].CONFIGURATION.MSB >> 5) & 1;

  H_alert = (TMP117_RegBank[sen].CONFIGURATION.MSB >> 7) & 1;

  L_alert = (TMP117_RegBank[sen].CONFIGURATION.MSB >> 6) & 1;

  temp_res = uint16_t(TMP117_RegBank[sen].TEMP_RESULT.MSB << 8);
  temp_res |= TMP117_RegBank[sen].TEMP_RESULT.LSB;

  // alert mode
  if (t_na_mod == 0) {
    if (TMP117_Alert_clear[sen] == true) {
      TMP117_Alert_clear[sen] = false;

      H_alert = 0;
      L_alert = 0;
    }

    if (dat_ready == 1) {
      if (temp_res > t_H_alarm) {
        H_alert = 1;
      }

      if (temp_res < t_L_alarm) {
        L_alert = 1;
      }
    }

#if use_LED_for_alarm == 1
    if ((H_alert == 1) || (L_alert == 1)) {
      digitalWrite(LED, pin_pol);
    }
#endif
  } else { // if (t_na_mod == 1) // Therm Mode
    if (dat_ready == 1) {
      if (temp_res > t_H_alarm) {
        H_alert = 1;
        L_alert = 0;
      }

      if (temp_res < t_L_alarm) {
        H_alert = 0;
        L_alert = 0;
      }
    }
  }

  TMP117_RegBank[sen].CONFIGURATION.MSB &= ~(H_alert << 7);
  TMP117_RegBank[sen].CONFIGURATION.MSB |= H_alert << 7;

  TMP117_RegBank[sen].CONFIGURATION.MSB &= ~(L_alert << 6);
  TMP117_RegBank[sen].CONFIGURATION.MSB |= L_alert << 6;
}

void saveToEEPROM(uint8_t sen) {
  uint16_t addr = (sen * 6) + EEPROM_base_addr; // 0-3 // 48 bytes

  EEPROM.update(addr, new_EEPROM1[sen].MSB);
  EEPROM.update(addr + 1, new_EEPROM1[sen].LSB);
  EEPROM.update(addr + 2, new_EEPROM2[sen].MSB);
  EEPROM.update(addr + 3, new_EEPROM2[sen].LSB);
  EEPROM.update(addr + 4, new_EEPROM3[sen].MSB);
  EEPROM.update(addr + 5, new_EEPROM3[sen].LSB);
}

void initFromEEPROM(uint8_t sen) {
  uint16_t addr = (sen * 6) + EEPROM_base_addr; // 0-3 // 48 bytes

  TMP117_RegBank[sen].EEPROM1.MSB = EEPROM.read(addr);
  TMP117_RegBank[sen].EEPROM1.LSB = EEPROM.read(addr + 1);
  TMP117_RegBank[sen].EEPROM2.MSB = EEPROM.read(addr + 2);
  TMP117_RegBank[sen].EEPROM2.LSB = EEPROM.read(addr + 3);
  TMP117_RegBank[sen].EEPROM3.MSB = EEPROM.read(addr + 4);
  TMP117_RegBank[sen].EEPROM3.LSB = EEPROM.read(addr + 5);
}

void fillEEPROM(uint8_t sen) {
  uint16_t addr = (sen * 6) + EEPROM_base_addr; // 0-3 // 48 bytes

  EEPROM.update(addr, random(0, 255));
  EEPROM.update(addr + 1, random(0, 255));
  EEPROM.update(addr + 2, random(0, 255));
  EEPROM.update(addr + 3, random(0, 255));
  EEPROM.update(addr + 4, random(0, 255));
  EEPROM.update(addr + 5, random(0, 255));
}

void EEPROM_update(uint8_t sen) {
  if (((TMP117_RegBank[sen].EEPROM_UL.MSB >> 7) & 1) == 1) {
    TMP117_RegBank[sen].EEPROM1.MSB = new_EEPROM1[sen].MSB;
    TMP117_RegBank[sen].EEPROM1.LSB = new_EEPROM1[sen].LSB;

    TMP117_RegBank[sen].EEPROM2.MSB = new_EEPROM2[sen].MSB;
    TMP117_RegBank[sen].EEPROM2.LSB = new_EEPROM2[sen].LSB;

    TMP117_RegBank[sen].EEPROM3.MSB = new_EEPROM3[sen].MSB;
    TMP117_RegBank[sen].EEPROM3.LSB = new_EEPROM3[sen].LSB;

    if (uint8_t(use_real_EEPROM) == 1) {
      saveToEEPROM(sen);
    }

    if (TMP117_EEPROM_UL_enabled[sen] == false) {
      TMP117_EEPROM_UL_enabled[sen] = true;

      Serial.print("Sensor ");
      Serial.print(sen, DEC);
      Serial.println(" EEPROM Unlock");
    }
  } else { // restore values
    new_EEPROM1[sen].MSB = TMP117_RegBank[sen].EEPROM1.MSB; // TMP117_EEPROM1_MSB[sen];
    new_EEPROM1[sen].LSB = TMP117_RegBank[sen].EEPROM1.LSB; // TMP117_EEPROM1_LSB[sen];

    new_EEPROM2[sen].MSB = TMP117_RegBank[sen].EEPROM2.MSB; // TMP117_EEPROM2_MSB[sen];
    new_EEPROM2[sen].LSB = TMP117_RegBank[sen].EEPROM2.LSB; // TMP117_EEPROM2_LSB[sen];

    new_EEPROM3[sen].MSB = TMP117_RegBank[sen].EEPROM3.MSB; // TMP117_EEPROM3_MSB[sen];
    new_EEPROM3[sen].LSB = TMP117_RegBank[sen].EEPROM3.LSB; // TMP117_EEPROM3_LSB[sen];

    if (TMP117_EEPROM_UL_enabled[sen] == true) {
      TMP117_EEPROM_UL_enabled[sen] = false;

      Serial.print("Sensor ");
      Serial.print(sen, DEC);
      Serial.println(" EEPROM Lock");
    }
  }
}

void config_update(uint8_t sen) {
  uint8_t TMP117_CONF_tmp0_MSB = 0;
  uint8_t TMP117_CONF_tmp0_LSB = 0;

  uint8_t TMP117_CONF_tmp1_MSB = 0;
  uint8_t TMP117_CONF_tmp1_LSB = 0;

  TMP117_CONF_tmp0_MSB = TMP117_RegBank[sen].CONFIGURATION.MSB & TMP117_CONF_write_mask_MSB;
  TMP117_CONF_tmp0_LSB = TMP117_RegBank[sen].CONFIGURATION.LSB & TMP117_CONF_write_mask_LSB;

  TMP117_CONF_tmp1_MSB = new_TMP117_CONF_buff[sen][1] & TMP117_CONF_write_mask_MSB;
  TMP117_CONF_tmp1_LSB = new_TMP117_CONF_buff[sen][0] & TMP117_CONF_write_mask_LSB;

  if ((TMP117_CONF_tmp0_MSB != TMP117_CONF_tmp1_MSB) ||
      (TMP117_CONF_tmp0_LSB != TMP117_CONF_tmp1_LSB)) {

    TMP117_RegBank[sen].CONFIGURATION.MSB &= ~(TMP117_CONF_write_mask_MSB);
    TMP117_RegBank[sen].CONFIGURATION.MSB |= TMP117_CONF_tmp1_MSB;

    TMP117_RegBank[sen].CONFIGURATION.LSB &= ~(TMP117_CONF_write_mask_LSB);
    TMP117_RegBank[sen].CONFIGURATION.LSB |= TMP117_CONF_tmp1_LSB;

    Serial.print("Sensor ");
    Serial.print(sen, DEC);
    Serial.print(" new config: ");
    Serial.print(TMP117_RegBank[sen].CONFIGURATION.MSB, BIN);
    Serial.print("b (MSB) ");
    Serial.print(TMP117_RegBank[sen].CONFIGURATION.LSB, BIN);
    Serial.println("b (LSB) ");

    new_TMP117_CONF_buff[sen][1] = TMP117_RegBank[sen].CONFIGURATION.MSB;
    new_TMP117_CONF_buff[sen][0] = TMP117_RegBank[sen].CONFIGURATION.LSB;
  }

  TMP117_RegBank[sen].CONFIGURATION.MSB = TMP117_RegBank[sen].CONFIGURATION.MSB;
  TMP117_RegBank[sen].CONFIGURATION.LSB = TMP117_RegBank[sen].CONFIGURATION.LSB;
}

void conv_time_update(uint8_t sen) {
  uint16_t TMP117_CONF_tmp;
  uint8_t TMP117_CONF_AVG;
  uint8_t TMP117_CONF_CONV;

  TMP117_CONF_tmp = TMP117_RegBank[sen].CONFIGURATION.MSB << 8;
  TMP117_CONF_tmp |= TMP117_RegBank[sen].CONFIGURATION.LSB;

  TMP117_CONF_AVG = (TMP117_CONF_tmp >> TMP117_CONF_AVG0) & 0x03;   // 2 bits

  TMP117_CONF_CONV = (TMP117_CONF_tmp >> TMP117_CONF_CONV0) & 0x07; // 3 bits

  // Conv time update
  if (fixed_CONV_Data_Ready_speed[sen] == true) {
    TMP117_CONV_Data_Ready_speed[sen] = fixed_TMP117_CONV_Data_Ready_speed[sen];
  } else {
    uint16_t calc_time = 0;

    calc_time = AVG_00_table[TMP117_CONF_CONV];

    TMP117_CONV_Data_Ready_speed[sen] = calc_time;
  }

  // Print new conv time
  if (prev_TMP117_CONV_Data_Ready_speed[sen] != TMP117_CONV_Data_Ready_speed[sen]) {
    prev_TMP117_CONV_Data_Ready_speed[sen] = TMP117_CONV_Data_Ready_speed[sen];

    Serial.print("Sensor ");
    Serial.print(sen, DEC);
    Serial.print(": Conv time = ");
    Serial.print(TMP117_CONV_Data_Ready_speed[sen], DEC);
    Serial.print("ms | AVG = ");
    Serial.print(TMP117_CONF_AVG, BIN);
    Serial.print("b | CONV = ");
    Serial.print(TMP117_CONF_CONV, BIN);
    Serial.println("b");
  }
}

void requestEvent(void) {
  unsigned char bufferSend[2] = {0, 0};

  switch (get_TMP117_Register[CURR_DEV]) {
    case TMP117_TEMP_RESULT:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].TEMP_RESULT.MSB;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].TEMP_RESULT.LSB;
      break;
    case TMP117_CONFIGURATION:
      TMP117_CONF_Data_Ready_clear[CURR_DEV] = true;
      TMP117_Alert_clear[CURR_DEV] = true;

      bufferSend[0] = TMP117_RegBank[CURR_DEV].CONFIGURATION.MSB;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].CONFIGURATION.LSB;
      break;
    case TMP117_T_HIGH_LIMIT:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].T_HIGH_LIMIT.MSB;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].T_HIGH_LIMIT.LSB;
      break;
    case TMP117_T_LOW_LIMIT:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].T_LOW_LIMIT.MSB;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].T_LOW_LIMIT.LSB;
      break;
    case TMP117_EEPROM_UL:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].EEPROM_UL.MSB;
      bufferSend[1] = 0;
      break;
    case TMP117_EEPROM1:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].EEPROM1.MSB;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].EEPROM1.LSB;
      break;
    case TMP117_EEPROM2:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].EEPROM2.MSB;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].EEPROM2.LSB;
      break;
    case TMP117_TEMP_OFFSET:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].TEMP_OFFSET.MSB;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].TEMP_OFFSET.LSB;
      break;
    case TMP117_EEPROM3:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].EEPROM3.MSB;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].EEPROM3.LSB;
      break;
    case TMP117_DEVICE_ID:
      bufferSend[0] = TMP117_RegBank[CURR_DEV].DEVICE_ID.MSB; // 0x01;
      bufferSend[1] = TMP117_RegBank[CURR_DEV].DEVICE_ID.LSB; // 0x17;
      break;
    default:
      // statements
      break;
  }

  Wire.write(bufferSend, 2);
}

void receiveEvent(int howMany) {
  unsigned char bufferSend[2];
  unsigned char TWDR_buff = TWDR;

  rec_buff_index++;

  rec_buff[rec_buff_index] = TWDR_buff;

  if (howMany < 10) {
    for (int i = 0; i < howMany; i++) { // loop through all but the last
      c = Wire.read(); // receive byte as a character

      rec_data_buff_index++;
      rec_data_buff[rec_data_buff_index] = c;
    }
  }

  if (howMany == 1) {
    get_TMP117_Register[CURR_DEV] = c;
  } else if (howMany == 3) {
    switch (rec_data_buff[0]) {
      case TMP117_CONFIGURATION:
        new_TMP117_CONF_buff[CURR_DEV][1] = rec_data_buff[1];
        new_TMP117_CONF_buff[CURR_DEV][0] = rec_data_buff[2];
        break;
      case TMP117_T_HIGH_LIMIT:
        TMP117_RegBank[CURR_DEV].T_HIGH_LIMIT.MSB = rec_data_buff[1];
        TMP117_RegBank[CURR_DEV].T_HIGH_LIMIT.LSB = rec_data_buff[2];
        break;
      case TMP117_T_LOW_LIMIT:
        TMP117_RegBank[CURR_DEV].T_LOW_LIMIT.MSB = rec_data_buff[1];
        TMP117_RegBank[CURR_DEV].T_LOW_LIMIT.LSB = rec_data_buff[2];
        break;
      case TMP117_EEPROM_UL:
        TMP117_RegBank[CURR_DEV].EEPROM_UL.MSB = rec_data_buff[1]; // LSB not used
        break;
      case TMP117_EEPROM1:
        new_EEPROM1[CURR_DEV].MSB = rec_data_buff[1];
        new_EEPROM1[CURR_DEV].LSB = rec_data_buff[2];
        break;
      case TMP117_EEPROM2:
        new_EEPROM2[CURR_DEV].MSB = rec_data_buff[1];
        new_EEPROM2[CURR_DEV].LSB = rec_data_buff[2];
        break;
      case TMP117_TEMP_OFFSET:
        TMP117_RegBank[CURR_DEV].TEMP_OFFSET.MSB = rec_data_buff[1];
        TMP117_RegBank[CURR_DEV].TEMP_OFFSET.LSB = rec_data_buff[2];
        break;
      case TMP117_EEPROM3:
        new_EEPROM3[CURR_DEV].MSB = rec_data_buff[1];
        new_EEPROM3[CURR_DEV].LSB = rec_data_buff[2];
        break;
      default:
        // statements
        break;
    }
  } else if (howMany == 0) {
    switch (TWDR_buff) {
      case 0x20:
        CURR_DEV = 0; // 0x48
        break;
      case 0x24:
        CURR_DEV = 1; // 0x49
        break;
      case 0x28:
        CURR_DEV = 2; // 0x4A
        break;
      case 0x2C:
        CURR_DEV = 3; // 0x4B
        break;
      default:
        // statements
        break;
    }
  }

  while (Wire.available()) {
    (void)Wire.read();
  }
}
