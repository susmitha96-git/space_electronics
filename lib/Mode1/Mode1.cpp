/* Implement the mode 1 functionality here
 */
#include "Mode1.h"

#include <Arduino.h>
#include <Utils.h>

void mode1(){
  delay(100);
  if( modeChanged() ){
    return;
  }
}