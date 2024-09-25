#pragma once

#include "esphome/components/sensor/sensor.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace dlms_cosem {

static constexpr uint8_t MAX_TRIES = 10;

enum SensorType { SENSOR, TEXT_SENSOR };

class DlmsCosemSensorBase {
 public:
  static const uint8_t MAX_REQUEST_SIZE = 15;

  virtual SensorType get_type() const = 0;
  //  virtual const c &get_sensor_name() const { return ""; };
  virtual EntityBase *get_base() = 0;
  virtual void publish() = 0;

  void set_obis_code(const char *obis_code) { obis_code_ = obis_code; }
  const std::string &get_obis_code() const { return this->obis_code_; }

  void set_dont_publish(bool dont_publish) { dont_publish_ = dont_publish; }
  bool get_dont_publish() const { return dont_publish_; }

  void reset() {
    has_value_ = false;
    tries_ = 0;
  }

  bool has_value() { return has_value_; }

  void record_failure() {
    if (tries_ < MAX_TRIES) {
      tries_++;
    } else {
      has_value_ = false;
    }
  }
  bool is_failed() { return tries_ == MAX_TRIES; }

 protected:
  std::string obis_code_;
  bool has_value_;
  uint8_t tries_{0};
  bool dont_publish_{false};
};

class DlmsCosemSensor : public DlmsCosemSensorBase, public sensor::Sensor {
 public:
  SensorType get_type() const override { return SENSOR; }
  //  const StringRef &get_sensor_name() { this->get_name(); }
  EntityBase *get_base() { return this;}
  void publish() override { publish_state(value_); }

  void set_multiplier(float multiplier) { multiplier_ = multiplier; }

  void set_value(float value) {
    value_ = value * multiplier_;
    has_value_ = true;
    tries_ = 0;
  }

 protected:
  float value_;
  float multiplier_{1.0f};
};

#ifdef USE_TEXT_SENSOR
class DlmsCosemTextSensor : public DlmsCosemSensorBase, public text_sensor::TextSensor {
 public:
  SensorType get_type() const override { return TEXT_SENSOR; }
  //  const StringRef &get_sensor_name() const { return get_name(); }
  EntityBase *get_base() { return this;}
  void publish() override { publish_state(value_); }

  void set_value(const char *value) {
    value_ = std::string(value);
    has_value_ = true;
    tries_ = 0;
  }

 protected:
  std::string value_;
};
#endif

}  // namespace dlms_cosem
}  // namespace esphome
