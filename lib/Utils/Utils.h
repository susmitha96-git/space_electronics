/* Use this file for common code that's shared between the modes
 * 
 * If necessary for better readability, you can also add additional 
 * files for common code (e.g. one contains all I/O related functions and 
 * another contains the sensor readout functionality)
 */
#ifndef UTILS_H
#define UTILS_H

#include <SPI.h>
#include<Wire.h>
#include <Adafruit_BME280.h>

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

const int BME280_I2C = 0x76;
const byte CHIP_ID = 0xD0;
const byte CTRL_MEAS = 0xF4;
const byte TEMP_MSB = 0xFA;
const byte PRES_MSB = 0xF7;
const byte CALIB00 = 0x88;
// int dig_T1, dig_T2, dig_T3;
// Adafruit_BME280 bme; 

extern SPIClass SPI_LSM;  // Declare but do not define
// ------ servo + ldr ------
static const int servoPin = 7;
static const int channelServo = 0;
static const int ldrPin = 6;

// ----- touch pads -----
static const int touchUpPin = 4;
static const int touchDownPin = 1;
static const int touchLeftPin = 5;
static const int touchRightPin = 2;
static const int touchXPin = 3;
const int touchThreshold = 40000;


// ------ buzzer + rotary encoder ------
static const int channelBuzzer = 1; // 0 is used by servo
static const int buzzerPin = 14;


// ------ rest of switches + leds ------
static const int sw1Pin = 0;
static const int ledGreenPin = 35;
static const int ledYellowPin = 36;
static const int ledRedPin = 37;  

// global variable to manage modes and mode changes
extern int currentMode;   
extern volatile int nextMode;

void increaseModeNumber();
void decreaseModeNumber();
void initializeTouchbuttons();
void initializeGPIOs();
bool modeChanged();
void modeStartup(int mode);
#endif // UTILS_H
