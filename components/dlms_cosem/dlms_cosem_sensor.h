#pragma once

#include "esphome/components/sensor/sensor.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace dlms_cosem {

static constexpr uint8_t MAX_TRIES = 10;

enum SensorType { SENSOR, TEXT_SENSOR };

//const char * UNIT_STR_UNKNOWN = "Unknown unit";
#define UNIT_STR_UNKNOWN "Unknown unit"

class DlmsCosemSensorBase {
 public:
  static const uint8_t MAX_REQUEST_SIZE = 15;

  virtual SensorType get_type() const = 0;
  virtual const StringRef &get_sensor_name() = 0;
  virtual EntityBase *get_base() = 0;
  virtual void publish() = 0;

  void set_obis_code(const char *obis_code) { this->obis_code_ = obis_code; }
  const std::string &get_obis_code() const { return this->obis_code_; }

  void set_dont_publish(bool dont_publish) { this->we_shall_publish_ = !dont_publish; }
  bool shall_we_publish() const { return this->we_shall_publish_; }

  void set_attribute(uint8_t attr) { this->attribute_ = attr;}
  uint8_t get_attribute() { return this->attribute_;}

  void reset() {
    has_value_ = false;
    tries_ = 0;
  }

  bool has_value() { return has_value_; }
  virtual bool has_got_scale_and_unit() { return scale_and_unit_detected_; }

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
  bool we_shall_publish_{true};
  uint8_t attribute_{2};
  bool scale_and_unit_detected_{false};

};

class DlmsCosemSensor : public DlmsCosemSensorBase, public sensor::Sensor {
 public:
  SensorType get_type() const override { return SENSOR; }
  const StringRef &get_sensor_name() { return this->get_name(); }
  EntityBase *get_base() { return this;}
  void publish() override { publish_state(this->value_); }

  void set_scale_and_unit(int8_t scaler, uint8_t unit, const char * unit_s) {
    this->scale_and_unit_detected_ = true;
    this->scaler_ = scaler;
    this->scale_f_ = std::pow(10, scaler);
    this->unit_ = unit;
    this->unit_s_ = unit_s ? unit_s : UNIT_STR_UNKNOWN;
    ESP_LOGW("dlms_cosem_sensor", "scaler pow: %d, scale_f: %f, unit: %d (%s)", scaler_, scale_f_, unit_, unit_s_);
  }
  
  float get_scale() const { return scale_f_; }

  const char * get_unit() const { return unit_s_; }

  void set_multiplier(float multiplier) { this->multiplier_ = multiplier; }

  void set_value(float value) {
    this->value_ = value * scale_f_ * multiplier_;
    this->has_value_ = true;
    this->tries_ = 0;
  }

 protected:
  float value_;
  float multiplier_{1.0f};
  int8_t scaler_{0};
  float scale_f_{1.0f};
  uint8_t unit_{0};
  const char * unit_s_ = UNIT_STR_UNKNOWN;
};

#ifdef USE_TEXT_SENSOR
class DlmsCosemTextSensor : public DlmsCosemSensorBase, public text_sensor::TextSensor {
 public:
  SensorType get_type() const override { return TEXT_SENSOR; }
  const StringRef &get_sensor_name() { return this->get_name(); }
  EntityBase *get_base() { return this;}
  void publish() override { publish_state(value_); }

  bool has_got_scale_and_unit() override { return true; }

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
