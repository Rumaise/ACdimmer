/*
 * lightFading.ino
 *
 *  Created on: May 4, 2016
 *      Author: hodai
 */


#include <MsTimer2.h>
#include <RTevents.h>
#include <ACdimmer.h>

#define STEPS ((AC_DIMMER_VALUE_MAX - AC_DIMMER_VALUE_MIN)/100)
int outPin = 4;   // the pin connected to the triac gate
int i = AC_DIMMER_VALUE_MIN;

ACdimmer myDimmer(outPin, AC_DIMMER_PIN_3);

void setup() {
  myDimmer.begin();
}

void loop() {

  i+=STEPS;

  if (i > AC_DIMMER_VALUE_MAX) {
    i = AC_DIMMER_VALUE_MIN;
  }

  myDimmer.setValue(i);

  delay(50);

}
