#include "MAX6675.h"

namespace {
// MAX6675 read-frame bit definitions (see header for the full diagram).
constexpr uint16_t kOpenThermocoupleBit = 1 << 2;  // D2: 1 = probe open
constexpr uint8_t kReadingShift = 3;               // drop D0..D2
constexpr uint16_t kReadingMask = 0x0FFF;          // 12-bit reading
constexpr float kCelsiusPerLsb = 0.25f;            // 0.25 C resolution

// The MAX6675 tolerates SCK up to 4.3 MHz and clocks data out on the falling
// edge (read on the rising edge) -> SPI mode 0.
const SPISettings kSpiSettings(4000000, MSBFIRST, SPI_MODE0);
}  // namespace

MAX6675::MAX6675(SPIClass& spi, uint8_t cs_pin)
    : spi_(spi), cs_pin_(cs_pin), last_read_valid_(false) {}

void MAX6675::begin() {
  pinMode(cs_pin_, OUTPUT);
  digitalWrite(cs_pin_, HIGH);  // idle: deselected
}

uint16_t MAX6675::readRaw16() {
  spi_.beginTransaction(kSpiSettings);
  digitalWrite(cs_pin_, LOW);
  delayMicroseconds(5);  // /CS to first SCK setup time
  // The MAX6675 ignores MOSI; we send zeros purely to generate clock edges.
  const uint16_t raw = spi_.transfer16(0x0000);
  digitalWrite(cs_pin_, HIGH);
  spi_.endTransaction();
  return raw;
}

float MAX6675::readCelsius() {
  const uint16_t raw = readRaw16();

  if (raw & kOpenThermocoupleBit) {
    last_read_valid_ = false;
    return NAN;
  }

  const uint16_t counts = (raw >> kReadingShift) & kReadingMask;
  last_read_valid_ = true;
  return counts * kCelsiusPerLsb;
}
