/*
 * ACdimmer.h
 *
 *  Created on: May 4, 2016
 *      Author: hodai
 */

#ifndef ACDIMMER_H_
#define ACDIMMER_H_

//#include "RTevents.h"

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif


enum ACdimmerIntPin {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)
    AC_DIMMER_PIN_2 = 2,
    AC_DIMMER_PIN_3 = 3,
#elif defined (__AVR_ATmega128__)

#elif defined (__AVR_ATmega8__)

#endif

};

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)
#define AC_DIMMER_PIN_TO_INTERRUPT(pin) ((pin) == AC_DIMMER_PIN_2 ? 0 : \
                                        ((pin) == AC_DIMMER_PIN_3 ? 1 : \
                                         /*else*/ (-1)))
#elif defined (__AVR_ATmega128__)

#elif defined (__AVR_ATmega8__)

#endif

#define AC_DIMMER_VALUE_MIN     0
#define AC_DIMMER_VALUE_MAX     0xFF

class ACdimmer {
private:
    static ACdimmer* theDimmer;     // used for the ISR functions
    static bool firstDimmer;

    static ACdimmerIntPin zeroCrossPin;

    uint8_t _outputPin;
    volatile uint8_t _outValue;
    volatile uint8_t _fadingSpeed;

public:

    ACdimmer(uint8_t outputPin, ACdimmerIntPin zeroCrossPin);

    void begin();

    bool setValue(uint8_t newValue);

    bool setFadeToValue(uint8_t newValue, uint8_t speed);

private:

    static void zeroDetectorISR();

    static void trigerTheTriacISR();

    void trigerTheTriac();

};

#endif /* ACDIMMER_H_ */
