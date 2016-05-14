/*
 * ACdimmer.cpp
 *
 *  Created on: May 4, 2016
 *      Author: hodai
 */

#include "ACdimmer.h"
#include "RTtimer.h"

// TODO - is better to use the follow syntax:
// #define AC_DIMMER_DETTACH(pin) detachInterrupt(digitalPinToInterrupt(pin));
// #define AC_DIMMER_ATTACH(pin) attachInterrupt(digitalPinToInterrupt(pin), ACdimmer::zeroDetectorISR, FALLING);
#define AC_DIMMER_DETACH(pin) detachInterrupt(AC_DIMMER_PIN_TO_INTERRUPT(pin));
#define AC_DIMMER_ATTACH(pin) attachInterrupt(AC_DIMMER_PIN_TO_INTERRUPT(pin), ACdimmer::zeroDetectorISR, FALLING);

//#define AC_DIMMER_ATTACH_TIMER_US(us) RTevents::addTask(&ACdimmer::trigerTheTriacISR, us, 0);
#define AC_DIMMER_ATTACH_TIMER_US(us) RTtimer_schedNext_us( us);

#define AC_DIMMER_LEVEL_TO_OFSET(level) (map((AC_DIMMER_VALUE_MAX-(level)), \
                                         AC_DIMMER_VALUE_MIN , AC_DIMMER_VALUE_MAX, \
                                         AC_DIMMER_TRIGER_DELAY_US_MIN, AC_DIMMER_TRIGER_DELAY_US_MAX))
#define AC_DIMMER_OFSET_TO_LEVEL(ofset) /* TODO */

#define AC_DIMMER_WAVE_FRIQ_HZ              60   // 50 for USA
#define AC_DIMMER_WAVE_LENGTH_US            ((1000*1000L) / AC_DIMMER_WAVE_FRIQ_HZ)
#define AC_DIMMER_ZERO_DETECT_DELAY_US      100                                     // TODO - need to adjust

#define AC_DIMMER_TRIGER_INTERVAL_US        100
#define AC_DIMMER_TRIGER_DELAY_US_MIN       100
#define AC_DIMMER_TRIGER_DELAY_US_MAX       ((AC_DIMMER_WAVE_LENGTH_US / 2)  )
                                             //- AC_DIMMER_TRIGER_INTERVAL_US \
                                             - AC_DIMMER_ZERO_DETECT_DELAY_US)


/* statics members */
ACdimmer* ACdimmer::theDimmer;     // used for the ISR functions , TODO - replace with array of dimmers
ACdimmerIntPin ACdimmer::zeroCrossPin;
bool ACdimmer::firstDimmer = true;

ACdimmer::ACdimmer(uint8_t outputPin, ACdimmerIntPin zeroCrossPin) : _outputPin(outputPin),
            _outValue(0),
            _fadingSpeed(0){
    this->zeroCrossPin = zeroCrossPin;
}

void ACdimmer::begin(){
    theDimmer = this;   // save a pointer to myself for my static members (ISRs)

    if (firstDimmer) {
        RTtimer_begin();
        RTtimer_attachInterrupt(&ACdimmer::trigerTheTriacISR);

        if(AC_DIMMER_PIN_TO_INTERRUPT(zeroCrossPin) < 0) {
            // TODO - error!
            return;
        }

        // initialize the zeroCross pin as an input:
        pinMode(zeroCrossPin, INPUT);
        // by default start with value = 0 so no need to attach the interrupt
        // attachInterrupt(AC_DIMMER_PIN_TO_INTERRUPT(zeroCrossPin), ACdimmer::zeroDetectorISR, FALLING);
    }

    pinMode(_outputPin, OUTPUT);

    // starting at minimum value
    _outValue = AC_DIMMER_VALUE_MIN;
    digitalWrite(_outputPin, LOW);

}

bool ACdimmer::setValue(uint8_t newValue){
    if (AC_DIMMER_VALUE_MIN >= newValue) {
        _outValue = AC_DIMMER_VALUE_MIN;
        digitalWrite(_outputPin, LOW);
        AC_DIMMER_DETACH(zeroCrossPin);
        return true;
    }
    if (AC_DIMMER_VALUE_MAX <= newValue){
        _outValue = AC_DIMMER_VALUE_MIN;
        digitalWrite(_outputPin, HIGH);
        AC_DIMMER_DETACH(zeroCrossPin);
        return true;
    }

    if(AC_DIMMER_VALUE_MIN >= _outValue || AC_DIMMER_VALUE_MAX <= _outValue) {
        // the interrupt is disabled, need to activate it
        // digitalWrite(_outputPin, LOW); TODO - the next iteration will fix that
        AC_DIMMER_ATTACH(zeroCrossPin);
    }

    _outValue = newValue;

    return true;
}

bool ACdimmer::setFadeToValue(uint8_t newValue, uint8_t speed){
    // TODO - need to implement
    return false;
}

void ACdimmer::zeroDetectorISR(){
    uint16_t trigerDelay;
    trigerDelay = AC_DIMMER_LEVEL_TO_OFSET(theDimmer->_outValue);
    AC_DIMMER_ATTACH_TIMER_US(trigerDelay);
}

void ACdimmer::trigerTheTriacISR(){
    theDimmer->trigerTheTriac();
}

void ACdimmer::trigerTheTriac(){
    digitalWrite(_outputPin, HIGH);
    // delay 50 uSec on output pulse to turn on triac
    delayMicroseconds(AC_DIMMER_TRIGER_INTERVAL_US);
    digitalWrite(_outputPin, LOW);

}



