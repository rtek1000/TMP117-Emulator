/******************************************************************************
  Example7_AdvancedI2CFunctions.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  July 22 2019
  ~
   * Modified by RTEK1000 May/26/2023

  This sketch allows the user to change the address of the device and to
  change the Wire port for I2C Communications. The address can be physically
  changed with an external jumper on the back of the sensor.

  Resources:
  Wire.h (included with Arduino IDE)
  SparkFunTMP117.h (included in the src folder) http://librarymanager/All#SparkFun_TMP117

  Development environment specifics:
  Arduino 1.8.9+
  Hardware Version 1.0.0

  This code is beerware; if you see me (or any other SparkFun employee) at
  the local, and you've found our code helpful, please buy us a round!

  Distributed as-is; no warranty is given.
******************************************************************************/

/*
  NOTE: For the most accurate readings:
  - Avoid heavy bypass traffic on the I2C bus
  - Use the highest available communication speeds
  - Use the minimal supply voltage acceptable for the system
  - Place device horizontally and out of any airflow when storing
  For more information on reaching the most accurate readings from the sensor,
  reference the "Precise Temperature Measurements with TMP116" datasheet that is
  linked on Page 35 of the TMP117's datasheet
*/

#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMP117.h" // Used to send and recieve specific information from our sensor

TMP117 sensor; // Initalize sensor

/* The default address of the device is 0x48 (GND)
  Sensor address can be changed with an external jumper to:
  VCC = 0x49
  SDA = 0x4A
  SCL = 0x4B
  For a table of the addresses, reference page 19, Table 2 on the TMP117 Datasheet


  This sketch only works if your platform doesn't have multiple I2C ports. This will not
  work on an Arduino Uno platform.

  To test the functionality of changing the addresses, change "Wire1" to "Wire" below
*/

uint8_t avg_val = 0b01;  // max: 11b
uint8_t conv_val = 0b100; // max: 111b

const uint8_t avg_mask = 0b11;  // max: 11b
const uint8_t conv_mask = 0b111; // max: 111b

uint8_t current_addr = 0;

void setup()
{
  Wire.begin(); // Compilation will fail here if your platform doesn't have multiple I2C ports
  Serial.begin(115200);    // Start serial communication at 115200 baud
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)

  Serial.println("\r\n\r\nTMP117 Example 7: Advanced I2C Functions");
  if (sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("Begin");
  }
  else
  {
    Serial.println("Device failed to setup - Freezing code.");
    while (1); // Runs forever
  }
  Serial.println("Device Address Options: ");

  Wire.beginTransmission(0x48); // test device
  if (Wire.endTransmission() == 0) {
    Serial.println("1: 0x48 (GND/Default) - [Found]");
  } else {
    Serial.println("1: 0x48 (GND/Default) - [Not found]");
  }

  Wire.beginTransmission(0x49); // test device
  if (Wire.endTransmission() == 0) {
    Serial.println("2: 0x49 (V+) - [Found]");
  } else {
    Serial.println("2: 0x49 (V+) - [Not found]");
  }

  Wire.beginTransmission(0x4A); // test device
  if (Wire.endTransmission() == 0) {
    Serial.println("3: 0x4A (SDA) - [Found]");
  } else {
    Serial.println("3: 0x4A (SDA) - [Not found]");
  }

  Wire.beginTransmission(0x4B); // test device
  if (Wire.endTransmission() == 0) {
    Serial.println("4: 0x4B (SCL) - [Found]");
  } else {
    Serial.println("4: 0x4B (SCL) - [Not found]");
  }

  Serial.println("S: Set conversion time");
  Serial.println(); // White space for easier readings
  Serial.print("Current Device Address: 0x");
  Serial.println(sensor.getAddress(), HEX); // Prints the current address of the device in Hex
  current_addr = sensor.getAddress();
}

void loop()
{
  Serial.println("Enter new address (number 1-4, L, T): ");
  while (Serial.available() == 0); // Waits for the user input
  uint8_t serialCommand = Serial.read(); // Reads the input from the serial port
  Serial.print("Number received: ");
  if ((serialCommand >= '0') && (serialCommand <= '9')) {
    Serial.println(serialCommand - 48);
  } else {
    Serial.println(char(serialCommand));
  }
  if (serialCommand == 49) // 0x48 (GND/Default) // ASCII 1: 49
  {
    current_addr = 0x48;
    sensor_handle(current_addr);
  }
  else if (serialCommand == 50) // 0x49 (V+) // ASCII 2: 50
  {
    current_addr = 0x49;
    sensor_handle(current_addr);
  }
  else if (serialCommand == 51) // 0x4A (SDA) // ASCII 3: 51
  {
    current_addr = 0x4A;
    sensor_handle(current_addr);
  }
  else if (serialCommand == 52) // 0x4B (SCL) // ASCII 4: 52
  {
    current_addr = 0x4B;
    sensor_handle(current_addr);
  }
  else if ((serialCommand == 'L') || (serialCommand == 'l'))
  {
    setEEPROM_UL(current_addr);
  }
  else if ((serialCommand == 'T') || (serialCommand == 't'))
  {
    sensor_handle2(current_addr);
  }
  else // Runs when a number not (1-4 or S) is entered
  {
    Serial.println("Please enter a number 1-4, L, T");
    Serial.println();
  }
}

void printSensorInfo(void) {
  Serial.print("Sensor Address: 0x");
  Serial.println(sensor.getAddress(), HEX);
  Serial.println();
}

void sensor_handle(uint8_t sensor_addr) {
  sensor.begin(sensor_addr, Wire);
  printSensorInfo();
  getTemp();
  getUID();
}

void sensor_handle2(uint8_t sensor_addr) {
  sensor.begin(sensor_addr, Wire);
  printSensorInfo();

  uint16_t tmp_conf = 0;

  Serial.print("Get CONV configuration: 0b");

  Wire.beginTransmission(sensor.getAddress()); // select device
  if (Wire.endTransmission() == 0) {
    tmp_conf = sensor.readRegister(TMP117_CONFIGURATION);
  }

  //tmp_conf = 0;

  conv_val = (tmp_conf >> 7) & 0b111;

  Serial.println(conv_val, BIN);

  avg_val = 0; // to get 15ms value

  if (conv_val < 0b111) {
    conv_val++;
  } else {
    conv_val = 0;
  }

  tmp_conf &= ~(uint16_t(avg_mask) << 5);
  tmp_conf &= ~(uint16_t(conv_mask) << 7);

  tmp_conf |= uint16_t(avg_val) << 5;
  tmp_conf |= uint16_t(conv_val) << 7;

  Serial.print("Set CONV configuration: 0b");
  Serial.println(conv_val, BIN);

  Serial.print("Set AVG configuration: 0b");
  Serial.println(avg_val, BIN);

  // set conv time:
  Wire.beginTransmission(sensor.getAddress()); // select device
  if (Wire.endTransmission() == 0) {
    sensor.writeRegister(TMP117_CONFIGURATION, tmp_conf);
  }
}

void getUID(void) {
  // get serial UID:
  uint16_t UID[3];

  Wire.beginTransmission(sensor.getAddress()); // select device
  if (Wire.endTransmission() == 0) {
    UID[0] = sensor.readRegister(TMP117_EEPROM1);
    UID[1] = sensor.readRegister(TMP117_EEPROM2);
    UID[2] = sensor.readRegister(TMP117_EEPROM3);
  }

  char c_text[15];

  sprintf(c_text, "%02X%02X%02X%02X%02X%02X",
          (UID[0] >> 8) & 0xFF, UID[0] & 0xFF,
          (UID[1] >> 8) & 0xFF, UID[1] & 0xFF,
          (UID[2] >> 8) & 0xFF, UID[2] & 0xFF);

  Serial.print("UID: ");
  Serial.println(c_text);
}

void setEEPROM_UL(uint8_t sensor_addr) {
  uint16_t EEPROM_UL = 0;

  sensor.begin(sensor_addr, Wire);
  printSensorInfo();

  Wire.beginTransmission(sensor.getAddress()); // select device
  if (Wire.endTransmission() == 0) {
    EEPROM_UL = sensor.readRegister(TMP117_EEPROM_UL);
  }

  delay(10);

  if (((EEPROM_UL >> 15) & 1) == 0) {
    Serial.println("Get EEPROM_UL lock");
  } else {
    Serial.println("Get EEPROM_UL unlock");
  }

  Serial.println("Set EEPROM_UL");

  if (((EEPROM_UL >> 14) & 1) == 0) {
    Wire.beginTransmission(sensor.getAddress()); // select device
    if (Wire.endTransmission() == 0) {
      if (((EEPROM_UL >> 15) & 1) == 0) { // lock
        sensor.writeRegister(TMP117_EEPROM_UL, 0x8000); // unlock
      } else {
        sensor.writeRegister(TMP117_EEPROM_UL, 0x0000); // lock
      }
    }

    delay(10);

    Wire.beginTransmission(sensor.getAddress()); // select device
    if (Wire.endTransmission() == 0) {
      EEPROM_UL = sensor.readRegister(TMP117_EEPROM_UL);

      if (((EEPROM_UL >> 15) & 1) == 0) {
        Serial.println("Get EEPROM_UL lock");
      } else {
        Serial.println("Get EEPROM_UL unlock");
      }
    }
  } else {
    Serial.println("EEPROM busy");
  }
}

void getTemp(void) {
  // Data Ready is a flag for the conversion modes - in continous conversion the dataReady flag should always be high
  if (sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    float tempC = sensor.readTempC();
    float tempF = sensor.readTempF();
    // Print temperature in °C and °F
    Serial.print("Temperature in Celsius: ");
    Serial.println(tempC);
    Serial.print("Temperature in Fahrenheit: ");
    Serial.println(tempF);
  }
}
