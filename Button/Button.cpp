#include "Button.h"
#include <inttypes.h>
#include "WProgram.h"

Button::Button(uint8_t pin) {
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
    _pin = pin;
    _begin_time = 0;
}

uint8_t Button::pressed(unsigned long threshold) {
    if (_begin_time) {
        unsigned long elapsed = millis() - _begin_time;
        if (elapsed < DEBOUNCE_TIME) return NOT;
        if (digitalRead(_pin) == LOW) return NOT;
        _begin_time = 0;
        return elapsed > threshold ? LONG : SHORT;
    }
    if (digitalRead(_pin) != LOW) return NOT;
    _begin_time = millis();
    return NOT;
}
