/*
 * lightFading.ino
 *
 *  Created on: May 4, 2016
 *      Author: hodai
 */

#include <ACdimmer.h>

#define STEPS 1

int outPin = 9;   // the pin connected to the triac gate
uint8_t level = AC_DIMMER_VALUE_MIN;

/* zero cros detector connected to pin 3 */
ACdimmer myDimmer(outPin, AC_DIMMER_PIN_3);

void setup() {
  myDimmer.begin();
  pinMode(10 , OUTPUT); // indication led
}

void loop() {

  level+=STEPS;

  if (level > AC_DIMMER_VALUE_MAX) {
    level = AC_DIMMER_VALUE_MIN;
  }

  myDimmer.setValue(level);
  
  digitalWrite(10, level < 128 );
  delay(50);

}