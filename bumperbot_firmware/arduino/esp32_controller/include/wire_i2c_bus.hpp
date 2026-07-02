#ifndef WIRE_I2C_BUS_HPP
#define WIRE_I2C_BUS_HPP

#include <Arduino.h>
#include <Wire.h>

#include <concepts>

// ============================================================
// I2C BUS CONCEPT
// ============================================================
template <typename T>
concept I2CBusConcept =
    requires(T a, uint8_t address, uint8_t reg, uint8_t value, uint8_t* buffer,
             size_t length) {
      { a.writeByte(address, reg, value) } -> std::same_as<bool>;
      { a.readBlock(address, reg, buffer, length) } -> std::same_as<bool>;
    };

// ============================================================
// CONCRETE ARDUINO WIRE I2C BUS POLICY
// ============================================================
class WireI2cBus {
 public:
  explicit WireI2cBus(TwoWire& i2cBus) : wire_(i2cBus) {}

  bool writeByte(uint8_t address, uint8_t reg, uint8_t value) {
    wire_.beginTransmission(address);
    wire_.write(reg);
    wire_.write(value);
    // 0 indicates success, non-zero indicates an error
    return (wire_.endTransmission() == 0);
  }

  bool readBlock(uint8_t address, uint8_t reg, uint8_t* buffer, size_t length) {
    wire_.beginTransmission(address);
    wire_.write(reg);

    // endTransmission(false) sends a restart message, keeping the connection
    // active
    if (wire_.endTransmission(false) != 0) {
      return false;  // Hardware I2C error (e.g., NACK)
    }

    wire_.requestFrom(static_cast<uint16_t>(address),
                      static_cast<uint8_t>(length), static_cast<uint8_t>(true));

    size_t index = 0;
    while (wire_.available() && index < length) {
      buffer[index++] = wire_.read();
    }

    return (index == length);
  }

 private:
  TwoWire& wire_;
};

static_assert(I2CBusConcept<WireI2cBus>,
              "WireI2cBus does not satisfy the I2CBusConcept requirements.");

#endif  // WIRE_I2C_BUS_HPP