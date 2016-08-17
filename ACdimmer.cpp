/*
 * ACdimmer.cpp
 *
 *  Created on: May 4, 2016
 *      Author: hodai
 */

#include "ACdimmer.h"

#define RT_TIMER_AUTOSTOP
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
#define AC_DIMMER_OFSET_TO_LEVEL(ofset) /* TODO - not needed for now*/

#define AC_DIMMER_WAVE_FRIQ_HZ              50   // 60 for USA
#define AC_DIMMER_WAVE_LENGTH_US            ((1000*1000L) / AC_DIMMER_WAVE_FRIQ_HZ)

#define AC_DIMMER_ZERO_DETECT_DELAY_US      2000   // this is the delay of the zero detection hardware - need to be adjusted

#define AC_DIMMER_TRIGER_INTERVAL_US        5      // minimum time needed for the Triac to be triggered in uSec

#define AC_DIMMER_TRIGER_DELAY_US_MIN       20
#define AC_DIMMER_TRIGER_DELAY_US_MAX       ((AC_DIMMER_WAVE_LENGTH_US / 2)\
                                             - AC_DIMMER_TRIGER_INTERVAL_US \
                                             - AC_DIMMER_ZERO_DETECT_DELAY_US)


/* statics members */
ACdimmer* ACdimmer::theDimmer;     // used for the ISR functions , TODO - replace with array of dimmers
ACdimmerIntPin ACdimmer::zeroCrossPin;
bool ACdimmer::firstDimmer = true;

ACdimmer::ACdimmer(uint8_t outputPin, ACdimmerIntPin zeroCrossPin) : _outputPin(outputPin),
            _outDelay(0),
            _fadingSteps(0){
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
    _outDelay = AC_DIMMER_TRIGER_DELAY_US_MAX;
    digitalWrite(_outputPin, LOW);

}

bool ACdimmer::setValue(uint8_t newValue){

    // cancel the fading if any
    _fadingFinalDelay = 0;
    _fadingSteps = 0;

    // change the trigger delay
    setDelay(AC_DIMMER_LEVEL_TO_OFSET(newValue));

    return true;
}

void ACdimmer::setDelay(uint16_t newDelay){

    if (AC_DIMMER_TRIGER_DELAY_US_MIN >= newDelay){
        // in cases that zero detection device connected directly to the Triac Anodes
        // if the Triac gate is Set (like in this case) the zero cross will not be detected
        // (will see always 0v)
        // if this is the case we must Clear the Triac gate in order to detect the zero cross
        // when changing the value.
        _fadingSteps = 0;
        _outDelay = AC_DIMMER_TRIGER_DELAY_US_MIN;
        AC_DIMMER_DETACH(zeroCrossPin);
        RTtimer_stop();
        digitalWrite(_outputPin, HIGH);
        return;
    }

    if (AC_DIMMER_TRIGER_DELAY_US_MAX <= newDelay) {
        _fadingSteps = 0;
        _outDelay = AC_DIMMER_TRIGER_DELAY_US_MAX;
        AC_DIMMER_DETACH(zeroCrossPin);
        RTtimer_stop();
        digitalWrite(_outputPin, LOW);
        return;
    }

    if(AC_DIMMER_TRIGER_DELAY_US_MIN >= _outDelay || AC_DIMMER_TRIGER_DELAY_US_MAX <= _outDelay) {
        // the interrupt is disabled, need to activate it
        // digitalWrite(_outputPin, LOW); TODO - the next iteration will fix that
        AC_DIMMER_ATTACH(zeroCrossPin);
    }

    // fading handle
    if ( (0 > _fadingSteps && _fadingFinalDelay >= newDelay) ||
                (0 < _fadingSteps && _fadingFinalDelay <= newDelay) ){
        _fadingSteps = 0;
        newDelay = _fadingFinalDelay;
    }

    digitalWrite(_outputPin, LOW);  // Clear the Triac gate in order to detect the zero cross

    _outDelay = newDelay;

    return;
}

bool ACdimmer::setFadeToValue(uint8_t newValue, uint16_t speed){
    long tempNewDelay;

    if(0 == speed){
        return setValue(newValue);
    }

    _fadingFinalDelay = AC_DIMMER_LEVEL_TO_OFSET(newValue);

    if (_outDelay < _fadingFinalDelay){
        _fadingSteps = speed;
    } else {
        _fadingSteps = -speed;
    }

    // change the trigger delay (first step)
    tempNewDelay = _outDelay + _fadingSteps;
    if (AC_DIMMER_TRIGER_DELAY_US_MIN > tempNewDelay || AC_DIMMER_TRIGER_DELAY_US_MAX < tempNewDelay){
        tempNewDelay = theDimmer->_fadingFinalDelay;
    }
    setDelay(tempNewDelay);

    return true;
}

void ACdimmer::zeroDetectorISR(){
    long tempNewDelay;

    AC_DIMMER_ATTACH_TIMER_US(theDimmer->_outDelay);

    // after attaching the trigger handle the fading mechanism
    if (0 != theDimmer->_fadingSteps) {
        tempNewDelay = theDimmer->_outDelay + theDimmer->_fadingSteps;

        // check for uint16 overflow
        if (AC_DIMMER_TRIGER_DELAY_US_MIN > tempNewDelay || AC_DIMMER_TRIGER_DELAY_US_MAX < tempNewDelay){
            tempNewDelay = theDimmer->_fadingFinalDelay;
        }

        // update the trigger delay
        theDimmer->setDelay(tempNewDelay);
    }

}

void ACdimmer::trigerTheTriacISR(){
    theDimmer->trigerTheTriac();
}

void ACdimmer::trigerTheTriac(){
    digitalWrite(_outputPin, HIGH);
    // delay on output pulse to turn on Triac
    delayMicroseconds(AC_DIMMER_TRIGER_INTERVAL_US);
    digitalWrite(_outputPin, LOW);

}


