/*
  Button - Simple Arduino library for simple buttons.
  Note: will enable the Arduino's built-in pull-up resistor!

  Juerd Waalboer
*/

#ifndef Button_h
#define Button_h

#define DEBOUNCE_TIME 100

#define NOT 0
#define SHORT 1
#define LONG 2

#include <inttypes.h>

class Button {
    public:
        Button(uint8_t pin);
        uint8_t pressed(unsigned long threshold = 500);
    private:
        uint8_t _pin;
        unsigned long _begin_time;
};

#endif
