#pragma once
#include <cstddef>
#include <cstdint>

#include "esphome/components/uart/uart.h"
#include "esphome/core/hal.h"

namespace esphome::dlms_cosem {

static const uint32_t TIMEOUT = 20;  // ms; default timeout in the uart implementation is 100ms

// Thin helper around the configured UART bus.
//
// It exists only to (a) change the baud rate at runtime -- the IEC/DLMS handshake negotiates a new
// speed in the middle of a session -- and (b) read a single byte with a short timeout instead of
// the default 100ms. Everything goes through the public UARTComponent API, so the same wrapper
// works for a real hardware UART (ESP-IDF, ESP8266) and for a BLE NUS-backed UART, without any
// assumptions about the concrete driver.
class DlmsCosemUart {
 public:
  // is_ble_nus: the parent is a BLE NUS link with no hardware UART, so baud rate cannot be changed.
  explicit DlmsCosemUart(uart::UARTComponent *parent, bool is_ble_nus = false)
      : parent_(parent), is_ble_nus_(is_ble_nus) {}

  // Reconfigure the bus to a new baud rate. No-op for BLE, which has no notion of baud rate.
  void update_baudrate(uint32_t baudrate) {
    if (this->is_ble_nus_)
      return;
    this->parent_->set_baud_rate(baudrate);
    this->parent_->load_settings(false);
  }

  // Read one byte, waiting at most TIMEOUT ms for it to arrive.
  bool read_one_byte(uint8_t *data) {
    if (!this->wait_for_data_(1))
      return false;
    return this->parent_->read_byte(data);
  }

 protected:
  bool wait_for_data_(size_t len) {
    uint32_t start_time = millis();
    while (this->parent_->available() < len) {
      if (millis() - start_time > TIMEOUT) {
        return false;
      }
      yield();
    }
    return true;
  }

  uart::UARTComponent *parent_;
  bool is_ble_nus_;
};

}  // namespace esphome::dlms_cosem
