/* Implement the mode 0 functionality here
 */
#include "Mode0.h"

#include <SPI.h>
#include <Arduino.h>
#include <math.h>
#include <Utils.h>
#include <Adafruit_LSM6DS3TRC.h>


// SPIClass SPI_LSM(HSPI);

// void writeRegister(uint8_t reg, uint8_t value) {
//     digitalWrite(LSM6DS3_CS, LOW);
//     SPI_LSM.transfer(reg & 0x7F);  // Write operation
//     SPI_LSM.transfer(value);
//     digitalWrite(LSM6DS3_CS, HIGH);
// }

// uint8_t readRegister(uint8_t reg) {
//     digitalWrite(LSM6DS3_CS, LOW);
//     SPI_LSM.transfer(reg | 0x80);  // Read operation
//     uint8_t value = SPI_LSM.transfer(0x00);
//     digitalWrite(LSM6DS3_CS, HIGH);
//     return value;
// }
#define LSM6DS3_CS 10  // Chip Select Pin
#define LSM6DS3_WHO_AM_I 0x0F
#define LSM6DS3_CTRL1_XL 0x10
#define LSM6DS3_CTRL2_G 0x11
#define LSM6DS3_STATUS_REG 0x1E
#define LSM6DS3_OUTX_L_G 0x22  // Gyro X-axis low byte
#define LSM6DS3_OUTX_L_XL 0x28 // Accel X-axis low byte
#define LSM_SCK 12
#define LSM_MISO 13
#define LSM_MOSI 11
void readData(uint8_t reg, uint8_t *buffer, uint8_t length) {
    digitalWrite(LSM6DS3_CS, LOW);
    SPI_LSM.transfer(reg | 0x80);
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = SPI_LSM.transfer(0x00);
    }
    digitalWrite(LSM6DS3_CS, HIGH);
}


// Adafruit_ISM330DHCX ism330dhcx;
// void setup(void) {
//   Serial.begin(115200);
//   while (!Serial)
//     delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit ISM330DHCX test!");

//   if (!ism330dhcx.begin_I2C()) {
//     // if (!ism330dhcx.begin_SPI(LSM_CS)) {
//     // if (!ism330dhcx.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
//     Serial.println("Failed to find ISM330DHCX chip");
//     while (1) {
//       delay(10);
//     }
//   }

//   Serial.println("ISM330DHCX Found!");

//   // ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
//   Serial.print("Accelerometer range set to: ");
//   switch (ism330dhcx.getAccelRange()) {
//   case LSM6DS_ACCEL_RANGE_2_G:
//     Serial.println("+-2G");
//     break;
//   case LSM6DS_ACCEL_RANGE_4_G:
//     Serial.println("+-4G");
//     break;
//   case LSM6DS_ACCEL_RANGE_8_G:
//     Serial.println("+-8G");
//     break;
//   case LSM6DS_ACCEL_RANGE_16_G:
//     Serial.println("+-16G");
//     break;
//   }

//   // ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
//   Serial.print("Gyro range set to: ");
//   switch (ism330dhcx.getGyroRange()) {
//   case LSM6DS_GYRO_RANGE_125_DPS:
//     Serial.println("125 degrees/s");
//     break;
//   case LSM6DS_GYRO_RANGE_250_DPS:
//     Serial.println("250 degrees/s");
//     break;
//   case LSM6DS_GYRO_RANGE_500_DPS:
//     Serial.println("500 degrees/s");
//     break;
//   case LSM6DS_GYRO_RANGE_1000_DPS:
//     Serial.println("1000 degrees/s");
//     break;
//   case LSM6DS_GYRO_RANGE_2000_DPS:
//     Serial.println("2000 degrees/s");
//     break;
//   case ISM330DHCX_GYRO_RANGE_4000_DPS:
//     Serial.println("4000 degrees/s");
//     break;
//   }

//   // ism330dhcx.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
//   Serial.print("Accelerometer data rate set to: ");
//   switch (ism330dhcx.getAccelDataRate()) {
//   case LSM6DS_RATE_SHUTDOWN:
//     Serial.println("0 Hz");
//     break;
//   case LSM6DS_RATE_12_5_HZ:
//     Serial.println("12.5 Hz");
//     break;
//   case LSM6DS_RATE_26_HZ:
//     Serial.println("26 Hz");
//     break;
//   case LSM6DS_RATE_52_HZ:
//     Serial.println("52 Hz");
//     break;
//   case LSM6DS_RATE_104_HZ:
//     Serial.println("104 Hz");
//     break;
//   case LSM6DS_RATE_208_HZ:
//     Serial.println("208 Hz");
//     break;
//   case LSM6DS_RATE_416_HZ:
//     Serial.println("416 Hz");
//     break;
//   case LSM6DS_RATE_833_HZ:
//     Serial.println("833 Hz");
//     break;
//   case LSM6DS_RATE_1_66K_HZ:
//     Serial.println("1.66 KHz");
//     break;
//   case LSM6DS_RATE_3_33K_HZ:
//     Serial.println("3.33 KHz");
//     break;
//   case LSM6DS_RATE_6_66K_HZ:
//     Serial.println("6.66 KHz");
//     break;
//   }

//   // ism330dhcx.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
//   Serial.print("Gyro data rate set to: ");
//   switch (ism330dhcx.getGyroDataRate()) {
//   case LSM6DS_RATE_SHUTDOWN:
//     Serial.println("0 Hz");
//     break;
//   case LSM6DS_RATE_12_5_HZ:
//     Serial.println("12.5 Hz");
//     break;
//   case LSM6DS_RATE_26_HZ:
//     Serial.println("26 Hz");
//     break;
//   case LSM6DS_RATE_52_HZ:
//     Serial.println("52 Hz");
//     break;
//   case LSM6DS_RATE_104_HZ:
//     Serial.println("104 Hz");
//     break;
//   case LSM6DS_RATE_208_HZ:
//     Serial.println("208 Hz");
//     break;
//   case LSM6DS_RATE_416_HZ:
//     Serial.println("416 Hz");
//     break;
//   case LSM6DS_RATE_833_HZ:
//     Serial.println("833 Hz");
//     break;
//   case LSM6DS_RATE_1_66K_HZ:
//     Serial.println("1.66 KHz");
//     break;
//   case LSM6DS_RATE_3_33K_HZ:
//     Serial.println("3.33 KHz");
//     break;
//   case LSM6DS_RATE_6_66K_HZ:
//     Serial.println("6.66 KHz");
//     break;
//   }

//   ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
//   ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2
// }

// void loop() {
//   //  /* Get a new normalized sensor event */
//   sensors_event_t accel;
//   sensors_event_t gyro;
//   sensors_event_t temp;
//   ism330dhcx.getEvent(&accel, &gyro, &temp);

//   Serial.print("\t\tTemperature ");
//   Serial.print(temp.temperature);
//   Serial.println(" deg C");

//   /* Display the results (acceleration is measured in m/s^2) */
//   Serial.print("\t\tAccel X: ");
//   Serial.print(accel.acceleration.x);
//   Serial.print(" \tY: ");
//   Serial.print(accel.acceleration.y);
//   Serial.print(" \tZ: ");
//   Serial.print(accel.acceleration.z);
//   Serial.println(" m/s^2 ");

//   /* Display the results (rotation is measured in rad/s) */
//   Serial.print("\t\tGyro X: ");
//   Serial.print(gyro.gyro.x);
//   Serial.print(" \tY: ");
//   Serial.print(gyro.gyro.y);
//   Serial.print(" \tZ: ");
//   Serial.print(gyro.gyro.z);
//   Serial.println(" radians/s ");
//   Serial.println();

//   delay(100);

  //  // serial plotter friendly format

  //  Serial.print(temp.temperature);
  //  Serial.print(",");

  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //  Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();
  //  delayMicroseconds(10000);
// }

void mode0(){
  delay(100);
  uint8_t data[12];

  readData(LSM6DS3_OUTX_L_XL, data, 6);  // Read Accelerometer Data
  int16_t accelX = (int16_t)(data[1] << 8 | data[0]);
  int16_t accelY = (int16_t)(data[3] << 8 | data[2]);
  int16_t accelZ = (int16_t)(data[5] << 8 | data[4]);

  readData(LSM6DS3_OUTX_L_G, data, 6);  // Read Gyroscope Data
  int16_t gyroX = (int16_t)(data[1] << 8 | data[0]);
  int16_t gyroY = (int16_t)(data[3] << 8 | data[2]);
  int16_t gyroZ = (int16_t)(data[5] << 8 | data[4]);

  // Convert raw acceleration data to mg (milli-g)
  float ax = accelX * 0.061;  // Scale factor for ±2g range
  float ay = accelY * 0.061;
  float az = accelZ * 0.061;

  // Calculate resultant acceleration magnitude
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
if(accelMagnitude<1000){
  ledcWrite(0, 127); // Start buzzer
  delay(200); // Beep duration
  ledcWrite(0, 0);   // Stop buzzer
  delay(200); // Pause between beeps
}
  Serial.print("Accel (mg): X=");
  Serial.print(ax, 2);
  Serial.print(" Y=");
  Serial.print(ay, 2);
  Serial.print(" Z=");
  Serial.print(az, 2);
  
  Serial.print(" | Resultant Accel (mg): ");
  Serial.println(accelMagnitude, 2);

  Serial.print("Gyro (dps): X=");
  Serial.print(gyroX * 8.75, 2);  // Scale factor for ±250dps range
  Serial.print(" Y=");
  Serial.print(gyroY * 8.75, 2);
  Serial.print(" Z=");
  Serial.println(gyroZ * 8.75, 2);

  delay(500);
  // if( modeChanged() ){
    return;
  // }
}