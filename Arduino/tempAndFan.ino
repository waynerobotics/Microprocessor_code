#include <SPI.h>
 
// Define pins for bit-banging I2C for Sensor 1
#define BB_SDA_1 11  // D11 on Arduino Nano ESP32
#define BB_SCL_1 12  // D12 on Arduino Nano ESP32
 
// Define pins for bit-banging I2C for Sensor 2
#define BB_SDA_2 9   // D9 on Arduino Nano ESP32
#define BB_SCL_2 10  // D10 on Arduino Nano ESP32
 
#define SEALEVELPRESSURE_HPA (1013.25)
 
// BME280 I2C address (typically 0x76 or 0x77)
// Using the same address for both sensors is okay since they're on separate bit-banged buses
#define BME280_ADDRESS 0x77
 
// BME280 registers
#define BME280_REG_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_PRESS_LSB 0xF8
#define BME280_REG_PRESS_XLSB 0xF9
#define BME280_REG_TEMP_MSB 0xFA
#define BME280_REG_TEMP_LSB 0xFB
#define BME280_REG_TEMP_XLSB 0xFC
#define BME280_REG_HUM_MSB 0xFD
#define BME280_REG_HUM_LSB 0xFE
 
// Calibration data for Sensor 1
uint16_t dig_T1_1;
int16_t dig_T2_1, dig_T3_1;
uint16_t dig_P1_1;
int16_t dig_P2_1, dig_P3_1, dig_P4_1, dig_P5_1, dig_P6_1, dig_P7_1, dig_P8_1, dig_P9_1;
uint8_t dig_H1_1, dig_H3_1;
int16_t dig_H2_1, dig_H4_1, dig_H5_1;
int8_t dig_H6_1;
 
// Calibration data for Sensor 2
uint16_t dig_T1_2;
int16_t dig_T2_2, dig_T3_2;
uint16_t dig_P1_2;
int16_t dig_P2_2, dig_P3_2, dig_P4_2, dig_P5_2, dig_P6_2, dig_P7_2, dig_P8_2, dig_P9_2;
uint8_t dig_H1_2, dig_H3_2;
int16_t dig_H2_2, dig_H4_2, dig_H5_2;
int8_t dig_H6_2;
 
// For temperature calculation
int32_t t_fine_1;
int32_t t_fine_2;
 
unsigned long delayTime;
 
void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for serial to be ready
  Serial.println(F("Dual BME280 Bit-Banging Test"));
 
  // Initialize Sensor 1
  initSensor(1);
 
  // Initialize Sensor 2
  initSensor(2);
 
  Serial.println("-- Dual BME280 Temperature Sensors --");
  delayTime = 1000;
  Serial.println();
}
 
void loop() {
  printTemperatures();
  delay(delayTime);
}
 
void printTemperatures() {
  float temp1 = readTemperature(1);
  float temp2 = readTemperature(2);
 
  Serial.print("Sensor 1 Temperature = ");
  Serial.print(temp1);
  Serial.println(" °C");
 
  Serial.print("Sensor 2 Temperature = ");
  Serial.print(temp2);
  Serial.println(" °C");
 
  Serial.println();
}
 
// Initialize a specific sensor
void initSensor(uint8_t sensorNum) {
  uint8_t BB_SDA = (sensorNum == 1) ? BB_SDA_1 : BB_SDA_2;
  uint8_t BB_SCL = (sensorNum == 1) ? BB_SCL_1 : BB_SCL_2;
 
  Serial.print(F("Initializing BME280 Sensor "));
  Serial.println(sensorNum);
 
  // Initialize bit-banging I2C pins with external pull-ups
  pinMode(BB_SDA, OUTPUT);
  pinMode(BB_SCL, OUTPUT);
  digitalWrite(BB_SDA, HIGH);
  digitalWrite(BB_SCL, HIGH);
 
  delay(100);  // Give some time for the I2C lines to stabilize
 
  // Reset the I2C bus
  i2c_reset(sensorNum);
 
  // Try to wake up the sensor with a soft reset
  i2c_write_register(sensorNum, BME280_ADDRESS, BME280_REG_RESET, 0xB6);
  delay(10);  // Wait for reset to complete
 
  // Try both possible I2C addresses
  uint8_t chipId = 0;
  uint8_t addr = BME280_ADDRESS;
 
  // Try first address
  chipId = i2c_read_register(sensorNum, addr, BME280_REG_ID);
  Serial.print("Trying address 0x");
  Serial.print(addr, HEX);
  Serial.print(", Chip ID: 0x");
  Serial.println(chipId, HEX);
 
  // If first address fails, try the other one
  if (chipId != 0x60 && chipId != 0x61) {
    addr = (addr == 0x76) ? 0x77 : 0x76;  // Toggle between 0x76 and 0x77
    chipId = i2c_read_register(sensorNum, addr, BME280_REG_ID);
    Serial.print("Trying alternate address 0x");
    Serial.print(addr, HEX);
    Serial.print(", Chip ID: 0x");
    Serial.println(chipId, HEX);
  }
 
  // Verify we have a valid ID
  if (chipId != 0x60 && chipId != 0x61) {
    Serial.print("Could not find a valid BME280 sensor at position ");
    Serial.println(sensorNum);
    Serial.println("Please check your wiring. Make sure pull-up resistors are present!");
   
    // Debug I2C scan
    Serial.println("\nScanning all I2C addresses for sensor...");
    for (uint8_t address = 1; address < 127; address++) {
      i2c_start(sensorNum);
      bool ack = i2c_write_byte(sensorNum, address << 1);
      i2c_stop(sensorNum);
     
      if (ack) {
        Serial.print("Device found at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
      }
    }
   
    // Continue anyway - maybe the other sensor is working
    Serial.println("Continuing with setup...");
  } else {
    // Read calibration data for the detected sensor
    readCalibrationData(sensorNum);
   
    // Configure the sensor
    // Set humidity oversampling
    i2c_write_register(sensorNum, BME280_ADDRESS, BME280_REG_CTRL_HUM, 0x01);  // x1 oversampling
   
    // Set temperature and pressure oversampling and mode
    // 0b101 = x16 pressure oversampling
    // 0b101 = x16 temperature oversampling
    // 0b11 = normal mode
    i2c_write_register(sensorNum, BME280_ADDRESS, BME280_REG_CTRL_MEAS, 0b10110111);
   
    // Set config
    // 0b001 = 0.5ms standby time
    // 0b100 = IIR filter x16
    // 0b0 = SPI disabled
    i2c_write_register(sensorNum, BME280_ADDRESS, BME280_REG_CONFIG, 0b00100100);
   
    Serial.print("Sensor ");
    Serial.print(sensorNum);
    Serial.println(" initialized successfully");
  }
}
 
// Bit-banging I2C functions with sensor number parameter
void i2c_start(uint8_t sensorNum) {
  uint8_t BB_SDA = (sensorNum == 1) ? BB_SDA_1 : BB_SDA_2;
  uint8_t BB_SCL = (sensorNum == 1) ? BB_SCL_1 : BB_SCL_2;
 
  // Ensure correct initial state
  digitalWrite(BB_SDA, HIGH);
  digitalWrite(BB_SCL, HIGH);
  delayMicroseconds(10);
 
  // Start condition: SDA goes LOW while SCL is HIGH
  digitalWrite(BB_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(BB_SCL, LOW);
  delayMicroseconds(10);
}
 
void i2c_stop(uint8_t sensorNum) {
  uint8_t BB_SDA = (sensorNum == 1) ? BB_SDA_1 : BB_SDA_2;
  uint8_t BB_SCL = (sensorNum == 1) ? BB_SCL_1 : BB_SCL_2;
 
  // Ensure SDA is LOW first
  digitalWrite(BB_SDA, LOW);
  delayMicroseconds(10);
 
  // Stop condition: SDA goes HIGH while SCL is HIGH
  digitalWrite(BB_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(BB_SDA, HIGH);
  delayMicroseconds(10);
}
 
bool i2c_write_byte(uint8_t sensorNum, uint8_t data) {
  uint8_t BB_SDA = (sensorNum == 1) ? BB_SDA_1 : BB_SDA_2;
  uint8_t BB_SCL = (sensorNum == 1) ? BB_SCL_1 : BB_SCL_2;
 
  // Write 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    // SCL LOW while changing SDA
    digitalWrite(BB_SCL, LOW);
    delayMicroseconds(5);
   
    // Set SDA to bit value
    digitalWrite(BB_SDA, (data >> i) & 0x01);
    delayMicroseconds(5);
   
    // Clock pulse
    digitalWrite(BB_SCL, HIGH);
    delayMicroseconds(10);
    digitalWrite(BB_SCL, LOW);
    delayMicroseconds(5);
  }
 
  // Release SDA for ACK
  pinMode(BB_SDA, INPUT_PULLUP);
  delayMicroseconds(5);
 
  digitalWrite(BB_SCL, HIGH);
  delayMicroseconds(10);
 
  // Read ACK (should be LOW for ACK)
  bool ack = !digitalRead(BB_SDA);
 
  digitalWrite(BB_SCL, LOW);
  delayMicroseconds(5);
 
  // Restore SDA as output
  pinMode(BB_SDA, OUTPUT);
 
  return ack;
}
 
uint8_t i2c_read_byte(uint8_t sensorNum, bool sendAck) {
  uint8_t BB_SDA = (sensorNum == 1) ? BB_SDA_1 : BB_SDA_2;
  uint8_t BB_SCL = (sensorNum == 1) ? BB_SCL_1 : BB_SCL_2;
 
  uint8_t data = 0;
 
  // Release SDA for reading
  pinMode(BB_SDA, INPUT_PULLUP);
  delayMicroseconds(5);
 
  // Read 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    digitalWrite(BB_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(BB_SCL, HIGH);
    delayMicroseconds(10);
   
    // Read bit
    if (digitalRead(BB_SDA)) {
      data |= (1 << i);
    }
   
    digitalWrite(BB_SCL, LOW);
    delayMicroseconds(5);
  }
 
  // Set SDA back to output for ACK/NACK
  pinMode(BB_SDA, OUTPUT);
 
  // Send ACK (LOW) or NACK (HIGH)
  digitalWrite(BB_SDA, !sendAck);
  delayMicroseconds(5);
 
  // Clock pulse for ACK/NACK
  digitalWrite(BB_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(BB_SCL, LOW);
  delayMicroseconds(5);
 
  return data;
}
 
void i2c_reset(uint8_t sensorNum) {
  uint8_t BB_SDA = (sensorNum == 1) ? BB_SDA_1 : BB_SDA_2;
  uint8_t BB_SCL = (sensorNum == 1) ? BB_SCL_1 : BB_SCL_2;
 
  // Ensure pins are outputs
  pinMode(BB_SDA, OUTPUT);
  pinMode(BB_SCL, OUTPUT);
 
  // First, make sure both lines are HIGH
  digitalWrite(BB_SDA, HIGH);
  digitalWrite(BB_SCL, HIGH);
  delayMicroseconds(20);
 
  // Toggle SCL up to 9 times to unlock a stuck slave device
  // Keep SDA HIGH during this process
  for (int i = 0; i < 9; i++) {
    digitalWrite(BB_SCL, LOW);
    delayMicroseconds(20);
    digitalWrite(BB_SCL, HIGH);
    delayMicroseconds(20);
  }
 
  // Generate a proper STOP condition
  digitalWrite(BB_SDA, LOW);  // Prepare for STOP
  delayMicroseconds(20);
  digitalWrite(BB_SCL, HIGH);
  delayMicroseconds(20);
  digitalWrite(BB_SDA, HIGH); // STOP condition
  delayMicroseconds(20);
 
  // Do a few more clock cycles to ensure everything is reset
  for (int i = 0; i < 3; i++) {
    digitalWrite(BB_SCL, LOW);
    delayMicroseconds(20);
    digitalWrite(BB_SCL, HIGH);
    delayMicroseconds(20);
  }
}
 
void i2c_write_register(uint8_t sensorNum, uint8_t addr, uint8_t reg, uint8_t value) {
  i2c_start(sensorNum);
  i2c_write_byte(sensorNum, addr << 1);  // Address with write bit (0)
  i2c_write_byte(sensorNum, reg);        // Register address
  i2c_write_byte(sensorNum, value);      // Value to write
  i2c_stop(sensorNum);
}
 
uint8_t i2c_read_register(uint8_t sensorNum, uint8_t addr, uint8_t reg) {
  i2c_start(sensorNum);
  i2c_write_byte(sensorNum, addr << 1);         // Address with write bit (0)
  i2c_write_byte(sensorNum, reg);               // Register address
  i2c_start(sensorNum);                         // Repeated start
  i2c_write_byte(sensorNum, (addr << 1) | 0x1); // Address with read bit (1)
  uint8_t data = i2c_read_byte(sensorNum, false); // Read with NACK
  i2c_stop(sensorNum);
  return data;
}
 
void i2c_read_registers(uint8_t sensorNum, uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len) {
  i2c_start(sensorNum);
  i2c_write_byte(sensorNum, addr << 1);         // Address with write bit (0)
  i2c_write_byte(sensorNum, reg);               // Register address
  i2c_start(sensorNum);                         // Repeated start
  i2c_write_byte(sensorNum, (addr << 1) | 0x1); // Address with read bit (1)
 
  for (uint8_t i = 0; i < len - 1; i++) {
    buffer[i] = i2c_read_byte(sensorNum, true);  // Read with ACK
  }
  buffer[len - 1] = i2c_read_byte(sensorNum, false); // Read last byte with NACK
 
  i2c_stop(sensorNum);
}
 
void readCalibrationData(uint8_t sensorNum) {
  uint8_t data[32];
 
  // Read temperature calibration data
  i2c_read_registers(sensorNum, BME280_ADDRESS, 0x88, data, 24);
 
  if (sensorNum == 1) {
    // Extract temperature calibration values for Sensor 1
    dig_T1_1 = (data[1] << 8) | data[0];
    dig_T2_1 = (data[3] << 8) | data[2];
    dig_T3_1 = (data[5] << 8) | data[4];
   
    // Extract pressure calibration values for Sensor 1
    dig_P1_1 = (data[7] << 8) | data[6];
    dig_P2_1 = (data[9] << 8) | data[8];
    dig_P3_1 = (data[11] << 8) | data[10];
    dig_P4_1 = (data[13] << 8) | data[12];
    dig_P5_1 = (data[15] << 8) | data[14];
    dig_P6_1 = (data[17] << 8) | data[16];
    dig_P7_1 = (data[19] << 8) | data[18];
    dig_P8_1 = (data[21] << 8) | data[20];
    dig_P9_1 = (data[23] << 8) | data[22];
   
    // Read humidity calibration data for Sensor 1
    dig_H1_1 = i2c_read_register(sensorNum, BME280_ADDRESS, 0xA1);
   
    i2c_read_registers(sensorNum, BME280_ADDRESS, 0xE1, data, 7);
   
    dig_H2_1 = (data[1] << 8) | data[0];
    dig_H3_1 = data[2];
    dig_H4_1 = (data[3] << 4) | (data[4] & 0x0F);
    dig_H5_1 = (data[5] << 4) | (data[4] >> 4);
    dig_H6_1 = data[6];
  } else {
    // Extract temperature calibration values for Sensor 2
    dig_T1_2 = (data[1] << 8) | data[0];
    dig_T2_2 = (data[3] << 8) | data[2];
    dig_T3_2 = (data[5] << 8) | data[4];
   
    // Extract pressure calibration values for Sensor 2
    dig_P1_2 = (data[7] << 8) | data[6];
    dig_P2_2 = (data[9] << 8) | data[8];
    dig_P3_2 = (data[11] << 8) | data[10];
    dig_P4_2 = (data[13] << 8) | data[12];
    dig_P5_2 = (data[15] << 8) | data[14];
    dig_P6_2 = (data[17] << 8) | data[16];
    dig_P7_2 = (data[19] << 8) | data[18];
    dig_P8_2 = (data[21] << 8) | data[20];
    dig_P9_2 = (data[23] << 8) | data[22];
   
    // Read humidity calibration data for Sensor 2
    dig_H1_2 = i2c_read_register(sensorNum, BME280_ADDRESS, 0xA1);
   
    i2c_read_registers(sensorNum, BME280_ADDRESS, 0xE1, data, 7);
   
    dig_H2_2 = (data[1] << 8) | data[0];
    dig_H3_2 = data[2];
    dig_H4_2 = (data[3] << 4) | (data[4] & 0x0F);
    dig_H5_2 = (data[5] << 4) | (data[4] >> 4);
    dig_H6_2 = data[6];
  }
 
  Serial.print("Calibration data loaded for Sensor ");
  Serial.println(sensorNum);
}
 
// Read temperature from specified sensor
float readTemperature(uint8_t sensorNum) {
  uint8_t data[3];
  i2c_read_registers(sensorNum, BME280_ADDRESS, BME280_REG_TEMP_MSB, data, 3);
 
  int32_t adc_T = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
 
  int32_t var1, var2;
 
  if (sensorNum == 1) {  
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1_1 << 1))) * ((int32_t)dig_T2_1)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1_1)) * ((adc_T >> 4) - ((int32_t)dig_T1_1))) >> 12) * ((int32_t)dig_T3_1)) >> 14;
   
    t_fine_1 = var1 + var2;
    float T = (t_fine_1 * 5 + 128) >> 8;
    return T / 100.0;
  } else {
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1_2 << 1))) * ((int32_t)dig_T2_2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1_2)) * ((adc_T >> 4) - ((int32_t)dig_T1_2))) >> 12) * ((int32_t)dig_T3_2)) >> 14;
   
    t_fine_2 = var1 + var2;
    float T = (t_fine_2 * 5 + 128) >> 8;
    return T / 100.0;
  }
}
