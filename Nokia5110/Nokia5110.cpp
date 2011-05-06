#include "Nokia5110.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "WProgram.h"


Nokia5110::Nokia5110(
    uint8_t sce_pin, uint8_t rst_pin, uint8_t dc_pin,
    uint8_t sdin_pin, uint8_t sclk_pin
) {
    _sce_pin  = sce_pin;
    _rst_pin  = rst_pin;
    _dc_pin   = dc_pin;
    _sdin_pin = sdin_pin;
    _sclk_pin = sclk_pin;

    pinMode(_sce_pin,  OUTPUT);
    pinMode(_rst_pin,  OUTPUT);
    pinMode(_dc_pin,   OUTPUT);
    pinMode(_sdin_pin, OUTPUT);
    pinMode(_sclk_pin, OUTPUT);

    digitalWrite(_rst_pin, LOW);
    digitalWrite(_rst_pin, HIGH);

    command(0x21);         // Extended command mode
    command(0x80 | 0xB9);  // Contrast
    command(0x04);         // Temperature coefficient
    command(0x13);         // LCD bias mode 1:48
    command(0x20);         // Standard command mode
    command(0x0C);         // Black on transparent

    clear();
};

void Nokia5110::_send(uint8_t value) {
    digitalWrite(_sce_pin, LOW);
    shiftOut(_sdin_pin, _sclk_pin, MSBFIRST, value);
    digitalWrite(_sce_pin, HIGH);
}

void Nokia5110::command(uint8_t value) {
    digitalWrite(_dc_pin, LOW);
    _send(value);
}

void Nokia5110::data(uint8_t value) {
    // For efficiency, write() and clear() do digitalWrite + _send themselves
    digitalWrite(_dc_pin, HIGH);
    _send(value);
}

void Nokia5110::write(uint8_t character) {
    if (character == '\n') {
        setCursor(0, _y + 1);
        return;
    }
    if (character == '\r') {
        setCursor(0, _y);
        return;
    }
    char glyph[5];
    memcpy_P(glyph, GLYPHS + (character - 0x20) * 5, 5);
    digitalWrite(_dc_pin, HIGH);
    for (uint8_t i = 0; i < 5; i++) _send(glyph[i]);
    _send(0);
}

void Nokia5110::setCursor(uint8_t x, uint8_t y) {
    command(0x80 | x * 6);
    command(0x40 | y);
    _y = y;
}

void Nokia5110::home() {
    setCursor(0, _y);
}

void Nokia5110::clear () {
    digitalWrite(_dc_pin, HIGH);
    for (int i = 0; i < LCD_X * LCD_Y / 8; i++) _send(0);
    setCursor(0, 0);
}

void Nokia5110::setDisplay() {
    command(0x20);
}

void Nokia5110::noDisplay() {
    command(0x24);
}

void Nokia5110::setInverse() {
    command(0x0D);
}

void Nokia5110::noInverse() {
    command(0x0C);
}
