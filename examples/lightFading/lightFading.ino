/*
 * lightFading.ino
 *
 *  Created on: May 4, 2016
 *      Author: hodai
 */

#include <ACdimmer.h>

#define STEPS 1

int outPin = 4;   // the pin connected to the triac gate
int level = AC_DIMMER_VALUE_MIN;

/* zero cros detector connected to pin 3 */
ACdimmer myDimmer(outPin, AC_DIMMER_PIN_3);

void setup() {
  myDimmer.begin();
}

void loop() {

  level+=STEPS;

  if (level > AC_DIMMER_VALUE_MAX) {
    level = AC_DIMMER_VALUE_MIN;
  }

  myDimmer.setValue(level);

  delay(50);

}
