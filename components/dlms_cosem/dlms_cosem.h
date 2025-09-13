#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>

#include "dlms_cosem_sensor.h"
#include "dlms_cosem_uart.h"
#include "object_locker.h"

//##include "gxignore-arduino.h"

#include <client.h>
#include <converters.h>
#include <cosem.h>
#include <dlmssettings.h>

//#define IEC_HANDSHAKE

namespace esphome {
namespace dlms_cosem {

static const size_t DEFAULT_IN_BUF_SIZE = 256;
static const size_t DEFAULT_IN_BUF_SIZE_PUSH = 2048;
static const size_t MAX_OUT_BUF_SIZE = 128;

using SensorMap = std::multimap<std::string, DlmsCosemSensorBase *>;

using FrameStopFunction = std::function<bool(uint8_t *buf, size_t size)>;
using ReadFunction = std::function<size_t()>;

using DlmsRequestMaker = std::function<int()>;
using DlmsResponseParser = std::function<int()>;

struct CosemObject;

using RegisterCosemObjectFunction = std::function<void(const CosemObject &object)>;

class DlmsCosemComponent : public PollingComponent, public uart::UARTDevice {
 public:
  DlmsCosemComponent() : tag_(generateTag()){};

  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; };

  void set_client_address(uint16_t addr) { this->client_address_ = addr; };

  void set_server_address(uint16_t addr);
  uint16_t set_server_address(uint16_t logicalAddress, uint16_t physicalAddress, unsigned char addressSize);
  void update_server_address(uint16_t addr);
  uint16_t update_server_address(uint16_t logicalAddress, uint16_t physicalAddress, unsigned char addressSize);

  void set_auth_required(bool auth) { this->auth_required_ = auth; };
  void set_password(const std::string &addr) { this->password_ = addr; };

  void set_baud_rates(uint32_t baud_rate_handshake, uint32_t baud_rate) {
    this->baud_rate_handshake_ = baud_rate_handshake;
    this->baud_rate_ = baud_rate;
  };
  void set_receive_timeout_ms(uint32_t timeout) { this->receive_timeout_ms_ = timeout; };
  void set_delay_between_requests_ms(uint32_t delay) { this->delay_between_requests_ms_ = delay; };
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; };

  void register_sensor(DlmsCosemSensorBase *sensor);

  void set_reboot_after_failure(uint16_t number_of_failures) { this->failures_before_reboot_ = number_of_failures; }
  void set_cp1251_conversion_required(bool required) { this->cp1251_conversion_required_ = required; }

  void set_push_mode(bool push_mode) { this->operation_mode_push_ = push_mode; }
  void set_push_show_log(bool show_log) { this->push_show_log_ = show_log; }

  bool has_error{true};

#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(transmission)
  SUB_BINARY_SENSOR(session)
  SUB_BINARY_SENSOR(connection)
#endif

#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(last_scan)
#endif

 protected:
  uint16_t client_address_{16};
  uint16_t server_address_{1};
  bool auth_required_{false};
  std::string password_{""};
  bool operation_mode_push_{false};
  bool push_show_log_{false};

  uint32_t receive_timeout_ms_{500};
  uint32_t delay_between_requests_ms_{50};
  bool cp1251_conversion_required_{true};

  GPIOPin *flow_control_pin_{nullptr};
  std::unique_ptr<DlmsCosemUart> iuart_;

  SensorMap sensors_;

  sensor::Sensor *crc_errors_per_session_sensor_{};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    TRY_LOCK_BUS,
    WAIT,
    COMMS_TX,
    COMMS_RX,
    MISSION_FAILED,
    //    WAITING_FOR_RESPONSE,
    OPEN_SESSION,
    BUFFERS_REQ,
    BUFFERS_RCV,
    ASSOCIATION_REQ,
    ASSOCIATION_RCV,
    // OPEN_SESSION_GET_ID,
    // SET_BAUD,
    // ACK_START_GET_INFO,
    DATA_ENQ_UNIT,
    DATA_ENQ,
    DATA_RECV,
    DATA_NEXT,
    SESSION_RELEASE,
    DISCONNECT_REQ,
    PUSH_DATA_PROCESS,  // Process received push data
    PUBLISH,
  } state_{State::NOT_INITIALIZED};
  State last_reported_state_{State::NOT_INITIALIZED};

  struct {
    uint32_t start_time{0};
    uint32_t delay_ms{0};
    State next_state{State::IDLE};
  } wait_;

  enum class OperationMode { NORMAL = 0, SCANNING } operation_mode_{OperationMode::NORMAL};

  bool is_idling() const { return this->state_ == State::WAIT || this->state_ == State::IDLE; };

  void set_next_state_(State next_state) { state_ = next_state; };
  void set_next_state_delayed_(uint32_t ms, State next_state);

  void prepare_and_send_dlms_buffers();
  void prepare_and_send_dlms_aarq();
  void prepare_and_send_dlms_auth();
  void prepare_and_send_dlms_data_unit_request(const char *obis, int type);
  void prepare_and_send_dlms_data_request(const char *obis, int type, bool reg_init = true);
  void prepare_and_send_dlms_release();
  void prepare_and_send_dlms_disconnect();

  void process_push_data();

  void send_dlms_req_and_next(DlmsRequestMaker maker, DlmsResponseParser parser, State next_state,
                              bool mission_critical = false, bool clear_buffer = true);

  int set_sensor_scale_and_unit(DlmsCosemSensor *sensor);
  int set_sensor_value(DlmsCosemSensorBase *sensor, const char *obis);

  int set_sensor_value(uint16_t, const uint8_t *, uint8_t, DLMS_DATA_TYPE, const uint8_t *, uint8_t);

  void indicate_transmission(bool transmission_on);
  void indicate_session(bool session_on);
  void indicate_connection(bool connection_on);

  bool is_push_mode() const { return this->operation_mode_push_; }

  // void read_reply_and_go_next_state_(ReadFunction read_fn, State next_state,
  // uint8_t retries, bool mission_critical,
  //                                    bool check_crc);
  struct {
    ReadFunction read_fn;
    State next_state;
    bool mission_critical;
    bool check_crc;
    uint8_t tries_max;
    uint8_t tries_counter;
    uint32_t err_crc;
    uint32_t err_invalid_frames;
  } reading_state_{nullptr, State::IDLE, false, false, 0, 0, 0, 0};
  size_t received_frame_size_{0};
  bool received_complete_reply_{false};

  struct {
    DlmsRequestMaker maker_fn;
    DlmsResponseParser parser_fn;
    State next_state;
    bool mission_critical;
    bool reply_is_complete;
    int last_error;
  } dlms_reading_state_{nullptr, nullptr, State::IDLE, false, false, DLMS_ERROR_CODE_OK};

  uint32_t baud_rate_handshake_{9600};
  uint32_t baud_rate_{9600};

  uint32_t last_rx_time_{0};

  struct LoopState {
    uint32_t session_started_ms{0};             // start of session
    SensorMap::iterator request_iter{nullptr};  // talking to meter
    SensorMap::iterator sensor_iter{nullptr};   // publishing sensor values

  } loop_state_;

  struct InOutBuffers {
    message out_msg;
    uint16_t out_msg_index{0};
    uint16_t out_msg_data_pos{0};
    gxByteBuffer in;
    size_t in_position;

    gxReplyData reply;

    void init(size_t default_in_buf_size);
    void reset();
    void check_and_grow_input(uint16_t more_data);
    // next function shows whether there are still messages to send
    bool has_more_messages_to_send() const { return out_msg_index < out_msg.size; }

    gxRegister gx_register;
    unsigned char gx_attribute{2};

  } buffers_;

 protected:
  dlmsSettings dlms_settings_;

  void clear_rx_buffers_();

  void set_baud_rate_(uint32_t baud_rate);
  bool are_baud_rates_different_() const { return baud_rate_handshake_ != baud_rate_; }

  void send_dlms_messages_();

  size_t receive_frame_(FrameStopFunction stop_fn);
  size_t receive_frame_ascii_();
  size_t receive_frame_hdlc_();

  size_t receive_frame_raw_();
  uint32_t time_raw_limit_{0};

  inline void update_last_rx_time_() { this->last_rx_time_ = millis(); }
  bool check_wait_timeout_() { return millis() - wait_.start_time >= wait_.delay_ms; }
  bool check_rx_timeout_() { return millis() - this->last_rx_time_ >= receive_timeout_ms_; }

  char *extract_meter_id_(size_t frame_size);
  char *get_nth_value_from_csv_(char *line, uint8_t idx);

  void report_failure(bool failure);
  void abort_mission_();

  const char *state_to_string(State state);
  void log_state_(State *next_state = nullptr);

  struct Stats {
    uint32_t connections_tried_{0};
    uint32_t crc_errors_{0};
    uint32_t crc_errors_recovered_{0};
    uint32_t invalid_frames_{0};
    uint8_t failures_{0};

    float crc_errors_per_session() const { return (float) crc_errors_ / connections_tried_; }
  } stats_;
  void stats_dump();

  uint8_t failures_before_reboot_{0};

  //const char *dlms_error_to_string(int error);

  bool try_lock_uart_session_();
  void unlock_uart_session_();

 public:
  //static const char *dlms_data_type_to_string(DLMS_DATA_TYPE vt);

 private:
  static uint8_t next_obj_id_;
  std::string tag_;

  static std::string generateTag();
};

}  // namespace dlms_cosem
}  // namespace esphome
