/* Implement the mode 4 functionality here
 */
#include "Mode4.h"

#include <Arduino.h>
#include <Utils.h>

void mode4(){
  delay(100);
  if( modeChanged() ){
    return;
  }
}