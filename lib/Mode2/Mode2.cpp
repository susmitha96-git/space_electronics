/* Implement the mode 2 functionality here
 */
#include "Mode2.h"

#include <Arduino.h>
#include <Utils.h>
Adafruit_BME280 bme; 

void mode2(){
  delay(100);
    // Set "forced mode" to read temp
    Wire.beginTransmission(BME280_I2C);
    Wire.write(CTRL_MEAS);   // Access control measurement config register
    Wire.write(0b00100010);  // Write into register: set for force conversion 0b00100001 as well, see table 25
    Wire.endTransmission();
  
    // Read temp registers -- two out of three possible
    Wire.beginTransmission(BME280_I2C);
    Wire.write(PRES_MSB);
    Wire.endTransmission();
    Wire.requestFrom(BME280_I2C, 3);
    byte press_msb = Wire.read();  // get first byte
    byte press_lsb = Wire.read();  // get second byte
    byte press_xlsb = Wire.read();
    // Printout temp on serial monitor
    //Serial.println(temp_msb, HEX);
    //Serial.println(temp_lsb, HEX);
    //Serial.println(temp_xlsb, HEX);
  
    // int press_adc = press_msb << 8 | press_lsb;  // 16 Bit ADC value
    // //Serial.println(temp_adc);
    // long adc_T = (press_msb << 8 | press_lsb) << 4;  // 20 Bit, calculate float for debugging
  
    // // Bonus: Calculate temp in degrees Celsius
    // //long t_fine;
    // double var1, var2, T;
    // var1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
    // var2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
    // //t_fine = var1 + var2;
    // //t_fine = (BME280_S32_t)(var1 + var2);
    // T = (var1 + var2) / 5120.0;
    //float t  = (var1 + var2) / 5120.0;
    
    // Printout temp on serial monitor (Teleplot)
    Serial.print(">Pressure:");
    float pressure = bme.readPressure() / 100.0F;  // Convert to hPa

    Serial.println(pressure); 
  
    /* Transmit bytes to Octave
    Serial.write(tempByte1);
    Serial.write(tempByte2);
    */
  // if( modeChanged() ){
  //   return;
  // }
}