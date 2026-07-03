#ifndef WIRE_I2C_BUS_HPP
#define WIRE_I2C_BUS_HPP

#include <Arduino.h>
#include <Wire.h>

#include <span>
#include <concepts>
#include <algorithm>


// ============================================================
// I2C BUS CONCEPT
// ============================================================
template <typename T>
concept I2CBusConcept =
    requires(T a, uint8_t address, uint8_t reg, uint8_t value, std::span<uint8_t> buffer) {
      { a.writeByte(address, reg, value) } -> std::same_as<bool>;
      { a.readBlock(address, reg, buffer) } -> std::same_as<bool>;
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

  bool readBlock(uint8_t address, uint8_t reg, std::span<uint8_t> buffer) {
    // Guard against empty spans early
    if (buffer.empty()) {
        return true;
    }

    wire_.beginTransmission(address);
    wire_.write(reg);

    // endTransmission(false) sends a restart message, keeping the connection active
    if (wire_.endTransmission(false) != 0) {
        return false;  // Hardware I2C error (e.g., NACK)
    }

    // requestFrom returns the actual number of bytes successfully read into the internal Wire buffer
    const size_t bytes_received = wire_.requestFrom(
        static_cast<uint16_t>(address),
        static_cast<uint8_t>(buffer.size()),
        static_cast<uint8_t>(true)
    );

    // Fail immediately if the bus did not deliver the requested frame size
    if (bytes_received != buffer.size()) {
        return false;
    }

    std::generate(buffer.begin(), buffer.end(), [this]() {
        return static_cast<uint8_t>(wire_.read());
    });

    return true;
}

private:
  TwoWire& wire_;
};

static_assert(I2CBusConcept<WireI2cBus>,
              "WireI2cBus does not satisfy the I2CBusConcept requirements.");

#endif  // WIRE_I2C_BUS_HPP