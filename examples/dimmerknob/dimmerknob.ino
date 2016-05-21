/*
 * dimmerKnob.ino
 *
 *  Created on: May 21, 2016
 *      Author: hodai
 */

#include <ACdimmer.h>

int outPin = 9;   // the pin connected to the triac gate
uint8_t level;

/* zero cros detector connected to pin 3 */
ACdimmer myDimmer(outPin, AC_DIMMER_PIN_3);

void setup() {
  myDimmer.begin();
}

void loop() {

  level = map(analogRead(A0), 0, 1023,0,255);
  myDimmer.setValue(level);

  delay(50);

}
