#pragma once
#include <SPI.h>
#include <cmath>

class MAX6675 {
public:
    MAX6675(SPIClass& spi, uint8_t csPin) : _spi(spi), _csPin(csPin) {}

    void begin() {
        pinMode(_csPin, OUTPUT);
        digitalWrite(_csPin, HIGH);
    }

    // Returns °C, or NAN if thermocouple is disconnected
    float read() {
        digitalWrite(_csPin, LOW);
        delayMicroseconds(5);
        uint16_t raw = _spi.transfer16(0x0000);
        digitalWrite(_csPin, HIGH);

        if (raw & 0x4) return NAN;  // Fault bit: open thermocouple
        return (raw >> 3) * 0.25f;
    }

private:
    SPIClass& _spi;
    uint8_t   _csPin;
};
