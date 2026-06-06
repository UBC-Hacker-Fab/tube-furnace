// MAX6675.h — Driver for the Maxim MAX6675 K-type thermocouple-to-digital
// converter.
//
// The MAX6675 is a cold-junction-compensated, read-only SPI sensor. There is
// no command/register interface: asserting chip select and clocking out 16
// bits yields one temperature sample. Because it is read-only there is no MOSI
// line — only SCK, SO (MISO), and /CS are wired.
//
// Read frame (16 bits, MSB first):
//
//   Bit   15   14 . . . . . . . . . . 3    2          1          0
//        +----+---------------------------+----------+----------+-----------+
//        | D15| D14                    D3 |    D2    |    D1    |    D0     |
//        +----+---------------------------+----------+----------+-----------+
//          |            |                      |          |          |
//          |            |                      |          |          +-- Three-state
//          |            |                      |          +------------- Device ID (0)
//          |            |                      +------------------------- Thermocouple
//          |            |                                                  input: 1 = OPEN
//          |            +------------------------------------------------- 12-bit reading
//          |                                                               0.25 C / LSB
//          +-------------------------------------------------------------- Dummy sign (0)
//
//   * Temperature = ((raw >> 3) & 0x0FFF) * 0.25  degrees Celsius.
//     Range 0 .. +1023.75 C.
//   * If bit D2 is set the thermocouple is open (disconnected); readCelsius()
//     returns NAN in that case.
//
// Timing notes:
//   * SCK up to 4.3 MHz; this driver uses a 4 MHz SPI transaction, MODE0.
//   * A conversion takes ~0.17-0.22 s. Do not poll faster than ~5 Hz or you
//     will re-read a stale conversion. /CS must be high for >= 0.22 s between
//     reads to start a fresh conversion.
//
// Usage:
//   SPIClass spi(VSPI);
//   spi.begin(SCK, MISO, /*MOSI=*/-1, CS);
//   MAX6675 tc(spi, CS);
//   tc.begin();
//   float c = tc.readCelsius();   // NAN if the probe is open

#pragma once

#include <Arduino.h>
#include <SPI.h>

class MAX6675 {
 public:
  // The SPI bus must be shared and already begin()'d by the caller (the same
  // bus may host other devices). `cs_pin` is this sensor's chip select.
  MAX6675(SPIClass& spi, uint8_t cs_pin);

  // Configure the chip-select pin (idle high). Call once from setup().
  void begin();

  // Perform one read. Returns degrees Celsius, or NAN if the thermocouple
  // input is open. Check lastReadValid() after a NAN to distinguish an open
  // probe (the only failure this sensor reports).
  float readCelsius();

  // True if the most recent readCelsius() returned a valid sample.
  bool lastReadValid() const { return last_read_valid_; }

 private:
  // Clock out the raw 16-bit frame with a proper SPI transaction.
  uint16_t readRaw16();

  SPIClass& spi_;
  uint8_t cs_pin_;
  bool last_read_valid_;
};
