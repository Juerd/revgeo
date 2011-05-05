#include "PowerPin.h"
#include <inttypes.h>
#include "WProgram.h"

PowerPin::PowerPin(uint8_t pin, uint8_t off_state) {
    _pin       = pin;
    _off_state = off_state;

    pinMode(_pin, OUTPUT);
    _set(0, 0);
};


void PowerPin::_set(unsigned long end_time, uint8_t state) {
    _end_time = end_time;
    _state = state;
    digitalWrite(_pin, _state ^ _off_state);
}

void PowerPin::on()                 { _set(0, 1); }
void PowerPin::off()                { _set(0, 0); }
void PowerPin::on( unsigned int ms) { _set(millis() + ms, 1); }
void PowerPin::off(unsigned int ms) { _set(millis() + ms, 0); }
void PowerPin::toggle()             { if (_state) off(); else on(); }

void PowerPin::check() {
    if (_end_time && millis() >= _end_time) toggle();
}
