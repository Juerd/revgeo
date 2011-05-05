/*
  PowerPin - Simple Arduino library for powering things, with timeouts.
  Juerd Waalboer
*/

#ifndef PowerPin_h
#define PowerPin_h

#include <inttypes.h>
#include <WProgram.h>

class PowerPin {
    public:
        PowerPin(uint8_t pin, uint8_t off_state = LOW);
        void on();
        void off();
        void on(unsigned int ms);
        void off(unsigned int ms);
        void toggle();
        void check();
    private:
        void _set(unsigned long end_time, uint8_t state);
        uint8_t _pin;
        uint8_t _off_state;
        unsigned long _end_time;
        uint8_t _state;
};

#endif
