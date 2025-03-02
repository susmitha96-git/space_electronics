#include "Utils.h"
#include <Arduino.h>
SPIClass SPI_LSM(HSPI);  // Define SPI_LSM once


/* Use this file for common code that's shared between the modes
 * 
 * If necessary for better readability, you can also add additional 
 * files for common code (e.g. one contains all I/O related functions and 
 * another contains the sensor readout functionality)
 */

int currentMode = -1;   
int volatile nextMode = 0;

/* The following two functions are used as ISRs for the touch buttons
 * The additional check for the touchRead value prevents the touch 
 * buttons to cause two mode changes since the interrupt is 
 * triggered on both falling and rising edges.
 */
void increaseModeNumber(){
  int maxTouch = max(touchRead(touchUpPin), touchRead(touchRightPin));
  if( currentMode < 5 && maxTouch > touchThreshold ){
    nextMode = currentMode + 1;
  }
}
void decreaseModeNumber(){
  int maxTouch = max(touchRead(touchDownPin), touchRead(touchLeftPin));
  if( currentMode > 0 && maxTouch > touchThreshold ){
    nextMode = currentMode - 1;
  }
}



/* The initialization routines are collected here to keep the 
 * setup function in the main file more readable
 */
void initializeTouchbuttons(){
  touchAttachInterrupt(touchUpPin, increaseModeNumber, touchThreshold);
  touchAttachInterrupt(touchRightPin, increaseModeNumber, touchThreshold);
  touchAttachInterrupt(touchDownPin, decreaseModeNumber, touchThreshold);
  touchAttachInterrupt(touchLeftPin, decreaseModeNumber, touchThreshold);
}
void initializeGPIOs(){
  pinMode(ledYellowPin, OUTPUT);
  
  ledcSetup(channelBuzzer,1E6,12);
  ledcAttachPin(buzzerPin,channelBuzzer);
}



bool modeChanged(){
  return (currentMode != nextMode);
}
void modeStartup(int mode){
  Serial.print("Switching to mode ");
  Serial.println(mode);
  digitalWrite(ledYellowPin, HIGH);
  delay(100);
  digitalWrite(ledYellowPin, LOW);  
}