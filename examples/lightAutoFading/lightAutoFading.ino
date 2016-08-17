/*
 * lightAutoFading.ino
 *
 *  Created on: Aug 17, 2016
 *      Author: hodai
 */

#include <ACdimmer.h>

int outPin = 9;   // the pin connected to the triac gate
uint8_t level = AC_DIMMER_VALUE_MIN;

/* zero cros detector connected to pin 3 */
ACdimmer myDimmer(outPin, AC_DIMMER_PIN_3);

void setup() {
  myDimmer.begin();
  pinMode(10 , OUTPUT);
  
}

void loop() {
  myDimmer.setFadeToValue(level, 50);
  digitalWrite(10, level == AC_DIMMER_VALUE_MIN );

  level = AC_DIMMER_VALUE_MIN + AC_DIMMER_VALUE_MAX - level;
  
  delay(5000);

}
