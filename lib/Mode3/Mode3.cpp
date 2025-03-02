/* Implement the mode 3 functionality here
 */
#include "Mode3.h"

#include <Arduino.h>
#include <Utils.h>

void mode3(){
  delay(100);
  if( modeChanged() ){
    return;
  }
}