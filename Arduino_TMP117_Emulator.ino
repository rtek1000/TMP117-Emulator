// TMP117 Emulator - For basic testing and implementation of TMP117 sensor
//
// by RTEK1000 - May/26/2023
//
// Ref. Arduino I2C slave: https://forum.arduino.cc/t/i2c-slave-transmitter-receiver-not-working/890321/6
//
// Ref. TMP117: https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library

#include <Wire.h>

#define sensors 4

#define debug_cfg 0 // set 1 to enable

// base values:
float analogTempC[sensors] = {25.7578125, 23.8203125, 28.34375, 27.0859375};

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

#define TMP117_RESOLUTION 0.0078125f

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

volatile uint8_t CURR_DEV = 0;

volatile int8_t rec_buff_index = -1;
volatile uint8_t rec_buff[10] = {0};

volatile int8_t rec_data_buff_index = -1;
volatile uint8_t rec_data_buff[10] = {0};

unsigned char c;

unsigned long millis0[sensors] = {0};

void print_interr_buff(bool debug);

void setup() {
  Serial.begin(115200);

  Serial.println("\r\n\r\nTMP117 Emulator start");

  Wire.begin(0x48);
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // I2C Slave Promiscuous Mode (ATmega328 and others from this MCU family)
  // TWAMR: TWI (Slave) Address Mask Register
  // https://stackoverflow.com/questions/34691478/arduino-as-slave-with-multiple-i2c-addresses
  TWAMR = (1 | 2) << 1; // 0x48, 0x49, 0x4A, 0x4B 

  for (uint8_t device = 0; device < sensors; device++) {
    TMP117_RegBank[device].EEPROM1.MSB = EEPROM1_init[device][0];
    TMP117_RegBank[device].EEPROM1.LSB = EEPROM1_init[device][1];

    TMP117_RegBank[device].EEPROM2.MSB = EEPROM2_init[device][0];
    TMP117_RegBank[device].EEPROM2.LSB = EEPROM2_init[device][1];

    TMP117_RegBank[device].EEPROM3.MSB = EEPROM3_init[device][0];
    TMP117_RegBank[device].EEPROM3.LSB = EEPROM3_init[device][1];
  }

  for (uint8_t i = 0; i < sensors; i++) {
    EEPROM_update(i);
  }
}

void loop(void) {
  print_interr_buff(uint8_t(debug_cfg)); // 1: debug

  for (uint8_t dev = 0; dev < sensors; dev++) {
    temperature_update(dev);

    EEPROM_update(dev);

    config_update(dev);

    // Conversion time update
    conv_time_update(dev);

    // Conv Ready update
    if ((millis() - millis0[dev]) > TMP117_CONV_Data_Ready_speed[dev]) {
      millis0[dev] = millis();

      setCONFbit(dev, TMP117_CONF_Data_Ready); // TMP117_CONF[dev] |= 1 << TMP117_CONF_Data_Ready;
    }
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

void setCONFbit(uint8_t dev, uint16_t bit_pos) {
  if (bit_pos > 7) {
    TMP117_RegBank[dev].CONFIGURATION.MSB |= (1 << (bit_pos - 8));
  } else {
    TMP117_RegBank[dev].CONFIGURATION.LSB |= (1 << bit_pos);
  }
}

void resetCONFbit(uint8_t dev, uint16_t bit_pos) {
  if (bit_pos > 7) {
    TMP117_RegBank[dev].CONFIGURATION.MSB &= ~(1 << (bit_pos - 8));
  } else {
    TMP117_RegBank[dev].CONFIGURATION.LSB &= ~(1 << bit_pos);
  }
}

void temperature_update(uint8_t dev) {
  if (TMP117_CONF_Data_Ready_clear[dev] == true) {
    TMP117_CONF_Data_Ready_clear[dev] = false;

    resetCONFbit(dev, TMP117_CONF_Data_Ready); //TMP117_CONF[dev] &= ~(uint16_t)(1 << TMP117_CONF_Data_Ready);

    if (auto_change_temperature[dev] == true) {
      if (analogTempC[dev] < 25.0) {
        analogTempC[dev] += TMP117_RESOLUTION;
      } else {
        analogTempC[dev] = 20.0;
      }
    } else if (rand_auto_change_temperature[dev] == true) {
      analogTempC[dev] = random(-1000, 1000) * TMP117_RESOLUTION + 25.0;
    }
  }

  uint16_t digitalTempC = int16_t(float(analogTempC[dev] / TMP117_RESOLUTION));

  TMP117_RegBank[dev].TEMP_RESULT.MSB = (digitalTempC >> 8) & 0xFF;
  TMP117_RegBank[dev].TEMP_RESULT.LSB = digitalTempC & 0xFF;
}

void EEPROM_update(uint8_t dev) {
  if (((TMP117_RegBank[dev].EEPROM_UL.MSB >> 7) & 1) == 1) {
    TMP117_RegBank[dev].EEPROM1.MSB = new_EEPROM1[dev].MSB;
    TMP117_RegBank[dev].EEPROM1.LSB = new_EEPROM1[dev].LSB;

    TMP117_RegBank[dev].EEPROM2.MSB = new_EEPROM2[dev].MSB;
    TMP117_RegBank[dev].EEPROM2.LSB = new_EEPROM2[dev].LSB;

    TMP117_RegBank[dev].EEPROM3.MSB = new_EEPROM3[dev].MSB;
    TMP117_RegBank[dev].EEPROM3.LSB = new_EEPROM3[dev].LSB;

    if (TMP117_EEPROM_UL_enabled[dev] == false) {
      TMP117_EEPROM_UL_enabled[dev] = true;

      Serial.print("Sensor ");
      Serial.print(dev, DEC);
      Serial.println(" EEPROM Unlock");
    }
  } else { // restore values
    new_EEPROM1[dev].MSB = TMP117_RegBank[dev].EEPROM1.MSB; // TMP117_EEPROM1_MSB[dev];
    new_EEPROM1[dev].LSB = TMP117_RegBank[dev].EEPROM1.LSB; // TMP117_EEPROM1_LSB[dev];

    new_EEPROM2[dev].MSB = TMP117_RegBank[dev].EEPROM2.MSB; // TMP117_EEPROM2_MSB[dev];
    new_EEPROM2[dev].LSB = TMP117_RegBank[dev].EEPROM2.LSB; // TMP117_EEPROM2_LSB[dev];

    new_EEPROM3[dev].MSB = TMP117_RegBank[dev].EEPROM3.MSB; // TMP117_EEPROM3_MSB[dev];
    new_EEPROM3[dev].LSB = TMP117_RegBank[dev].EEPROM3.LSB; // TMP117_EEPROM3_LSB[dev];

    if (TMP117_EEPROM_UL_enabled[dev] == true) {
      TMP117_EEPROM_UL_enabled[dev] = false;

      Serial.print("Sensor ");
      Serial.print(dev, DEC);
      Serial.println(" EEPROM Lock");
    }
  }
}

void config_update(uint8_t dev) {
  uint8_t TMP117_CONF_tmp0_MSB = 0;
  uint8_t TMP117_CONF_tmp0_LSB = 0;

  uint8_t TMP117_CONF_tmp1_MSB = 0;
  uint8_t TMP117_CONF_tmp1_LSB = 0;

  TMP117_CONF_tmp0_MSB = TMP117_RegBank[dev].CONFIGURATION.MSB & TMP117_CONF_write_mask_MSB;
  TMP117_CONF_tmp0_LSB = TMP117_RegBank[dev].CONFIGURATION.LSB & TMP117_CONF_write_mask_LSB;

  TMP117_CONF_tmp1_MSB = new_TMP117_CONF_buff[dev][1] & TMP117_CONF_write_mask_MSB;
  TMP117_CONF_tmp1_LSB = new_TMP117_CONF_buff[dev][0] & TMP117_CONF_write_mask_LSB;

  if ((TMP117_CONF_tmp0_MSB != TMP117_CONF_tmp1_MSB) ||
      (TMP117_CONF_tmp0_LSB != TMP117_CONF_tmp1_LSB)) {

    TMP117_RegBank[dev].CONFIGURATION.MSB &= ~(TMP117_CONF_write_mask_MSB);
    TMP117_RegBank[dev].CONFIGURATION.MSB |= TMP117_CONF_tmp1_MSB;

    TMP117_RegBank[dev].CONFIGURATION.LSB &= ~(TMP117_CONF_write_mask_LSB);
    TMP117_RegBank[dev].CONFIGURATION.LSB |= TMP117_CONF_tmp1_LSB;

    Serial.print("Sensor ");
    Serial.print(dev, DEC);
    Serial.print(" new config: ");
    Serial.print(TMP117_RegBank[dev].CONFIGURATION.MSB, BIN);
    Serial.print("b (MSB) ");
    Serial.print(TMP117_RegBank[dev].CONFIGURATION.LSB, BIN);
    Serial.println("b (LSB) ");

    new_TMP117_CONF_buff[dev][1] = TMP117_RegBank[dev].CONFIGURATION.MSB;
    new_TMP117_CONF_buff[dev][0] = TMP117_RegBank[dev].CONFIGURATION.LSB;
  }

  TMP117_RegBank[dev].CONFIGURATION.MSB = TMP117_RegBank[dev].CONFIGURATION.MSB;
  TMP117_RegBank[dev].CONFIGURATION.LSB = TMP117_RegBank[dev].CONFIGURATION.LSB;
}

void conv_time_update(uint8_t dev) {
  uint16_t TMP117_CONF_tmp;
  uint8_t TMP117_CONF_AVG;
  uint8_t TMP117_CONF_CONV;

  TMP117_CONF_tmp = TMP117_RegBank[dev].CONFIGURATION.MSB << 8;
  TMP117_CONF_tmp |= TMP117_RegBank[dev].CONFIGURATION.LSB;

  TMP117_CONF_AVG = (TMP117_CONF_tmp >> TMP117_CONF_AVG0) & 0x03;   // 2 bits

  TMP117_CONF_CONV = (TMP117_CONF_tmp >> TMP117_CONF_CONV0) & 0x07; // 3 bits

  // Conv time update
  if (fixed_CONV_Data_Ready_speed[dev] == true) {
    TMP117_CONV_Data_Ready_speed[dev] = fixed_TMP117_CONV_Data_Ready_speed[dev];
  } else {
    uint16_t calc_time = 0;

    calc_time = AVG_00_table[TMP117_CONF_CONV];

    TMP117_CONV_Data_Ready_speed[dev] = calc_time;
  }

  // Print new conv time
  if (prev_TMP117_CONV_Data_Ready_speed[dev] != TMP117_CONV_Data_Ready_speed[dev]) {
    prev_TMP117_CONV_Data_Ready_speed[dev] = TMP117_CONV_Data_Ready_speed[dev];

    Serial.print("Sensor ");
    Serial.print(dev, DEC);
    Serial.print(": Conv time = ");
    Serial.print(TMP117_CONV_Data_Ready_speed[dev], DEC);
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
