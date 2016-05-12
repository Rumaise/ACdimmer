/*
 * lightFading.ino
 *
 *  Created on: May 4, 2016
 *      Author: hodai
 */

#include <RTevents.h>
#include <ACdimmer.h>

#define STEPS ((AC_DIMMER_VALUE_MAX - AC_DIMMER_VALUE_MIN)/0xFF)

int outPin = 4;   // the pin connected to the triac gate
int val = AC_DIMMER_VALUE_MIN;

/* zero cros detector connected to pin 3 */
ACdimmer myDimmer(outPin, AC_DIMMER_PIN_3);

void setup() {
  myDimmer.begin();
}

void loop() {

  val+=STEPS;

  if (val > AC_DIMMER_VALUE_MAX) {
    val = AC_DIMMER_VALUE_MIN;
  }

  myDimmer.setValue(val);

  delay(50);

}
