/* Implement the mode 5 functionality here
 */
#include "Mode5.h"

#include <Arduino.h>
#include <Utils.h>
#include <Melodies.h>

void mode5(){
  delay(100);
  if( modeChanged() ){
    return;
  }
  if (touchRead(touchXPin) > touchThreshold){
    play_melody(imperial_march, sizeof(imperial_march), channelBuzzer);
  }
}