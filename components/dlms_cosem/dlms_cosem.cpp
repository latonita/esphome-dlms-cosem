
#include "dlms_cosem.h"
#include "axdr_parser.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <sstream>

namespace esphome {
namespace dlms_cosem {

static const char *TAG0 = "dlms_cosem_";

#define TAG (this->tag_.c_str())

static constexpr uint8_t SOH = 0x01;
static constexpr uint8_t STX = 0x02;
static constexpr uint8_t ETX = 0x03;
static constexpr uint8_t EOT = 0x04;
static constexpr uint8_t ENQ = 0x05;
static constexpr uint8_t ACK = 0x06;
static constexpr uint8_t CR = 0x0D;
static constexpr uint8_t LF = 0x0A;
static constexpr uint8_t NAK = 0x15;

static constexpr uint8_t HDLC_FLAG = 0x7E;

static const uint8_t CMD_ACK_SET_BAUD_AND_MODE[] = {ACK, '0', '5', '1', CR, LF};
static const uint8_t CMD_CLOSE_SESSION[] = {SOH, 0x42, 0x30, ETX, 0x75};

static constexpr uint8_t BOOT_WAIT_S = 10;

static char empty_str[] = "";

/*
static char format_hex_char(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' +
v; }

static std::string format_frame_pretty(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  std::ostringstream ss(ret);

  for (size_t i = 0; i < length; i++) {
    switch (data[i]) {
      case 0x00:
        ss << "<NUL>";
        break;
      case 0x01:
        ss << "<SOH>";
        break;
      case 0x02:
        ss << "<STX>";
        break;
      case 0x03:
        ss << "<ETX>";
        break;
      case 0x04:
        ss << "<EOT>";
        break;
      case 0x05:
        ss << "<ENQ>";
        break;
      case 0x06:
        ss << "<ACK>";
        break;
      case 0x0d:
        ss << "<CR>";
        break;
      case 0x0a:
        ss << "<LF>";
        break;
      case 0x15:
        ss << "<NAK>";
        break;
      case 0x20:
        ss << "<SP>";
        break;
      case 0x7E:
        ss << "<FLAG>";
        break;
      default:
        if (data[i] <= 0x20 || data[i] >= 0x7f) {
          ss << "<" << format_hex_char((data[i] & 0xF0) >> 4) <<
format_hex_char(data[i] & 0x0F) << ">"; } else { ss << (char) data[i];
        }
        break;
    }
  }
  if (length > 4)
    ss << " (" << length << ")";
  return ss.str();
}

uint8_t baud_rate_to_byte(uint32_t baud) {
  constexpr uint16_t BAUD_BASE = 300;
  constexpr uint8_t BAUD_MULT_MAX = 6;

  uint8_t idx = 0;  // 300
  for (size_t i = 0; i <= BAUD_MULT_MAX; i++) {
    if (baud == BAUD_BASE * (1 << i)) {
      idx = i;
      break;
    }
  }
  return idx + '0';
}
*/

void DlmsCosemComponent::set_baud_rate_(uint32_t baud_rate) {
  ESP_LOGV(TAG, "Setting baud rate %u bps", baud_rate);
  iuart_->update_baudrate(baud_rate);
}

void DlmsCosemComponent::set_server_address(uint16_t address) { this->server_address_ = address; };

uint16_t DlmsCosemComponent::set_server_address(uint16_t logicalAddress, uint16_t physicalAddress,
                                                unsigned char addressSize) {
  this->server_address_ = cl_getServerAddress(logicalAddress, physicalAddress, addressSize);

  ESP_LOGD(TAG,
           "Server address = %d (based on logical_address=%d, "
           "physical_address=%d, address_size=%d)",
           this->server_address_, logicalAddress, physicalAddress, addressSize);
  return this->server_address_;
}

void DlmsCosemComponent::update_server_address(uint16_t addr) {
  this->server_address_ = addr;
  cl_clear(&dlms_settings_);
  cl_init(&dlms_settings_, true, this->client_address_, this->server_address_,
          this->auth_required_ ? DLMS_AUTHENTICATION_LOW : DLMS_AUTHENTICATION_NONE,
          this->auth_required_ ? this->password_.c_str() : NULL, DLMS_INTERFACE_TYPE_HDLC);

  this->update();
}

uint16_t DlmsCosemComponent::update_server_address(uint16_t logicalAddress, uint16_t physicalAddress,
                                                   unsigned char addressSize) {
  this->set_server_address(logicalAddress, physicalAddress, addressSize);
  this->update_server_address(this->server_address_);
  return this->server_address_;
}

void DlmsCosemComponent::setup() {
  ESP_LOGD(TAG, "setup");

  cl_init(&dlms_settings_, true, this->client_address_, this->server_address_,
          this->auth_required_ ? DLMS_AUTHENTICATION_LOW : DLMS_AUTHENTICATION_NONE,
          this->auth_required_ ? this->password_.c_str() : NULL, DLMS_INTERFACE_TYPE_HDLC);

  this->buffers_.init(this->is_push_mode() ? DEFAULT_IN_BUF_SIZE_PUSH : DEFAULT_IN_BUF_SIZE);

  this->indicate_transmission(false);

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  iuart_ = make_unique<DlmsCosemUart>(*static_cast<uart::ESP32ArduinoUARTComponent *>(this->parent_));
#endif

#ifdef USE_ESP_IDF
  iuart_ = make_unique<DlmsCosemUart>(*static_cast<uart::IDFUARTComponent *>(this->parent_));
#endif

#if USE_ESP8266
  iuart_ = make_unique<DlmsCosemUart>(*static_cast<uart::ESP8266UartComponent *>(this->parent_));
#endif
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }

  this->set_baud_rate_(this->baud_rate_handshake_);

  if (this->is_push_mode()) {
    bool locked = false;
    for (int i = 0; i < 3; i++)
      if (this->try_lock_uart_session_()) {
        locked = true;
        break;
      }

    if (!locked) {
      ESP_LOGE(TAG, "Failed to lock UART session. Aborting setup.");
      this->mark_failed();
      return;
    }
  }

  this->set_timeout(BOOT_WAIT_S * 1000, [this]() {
    ESP_LOGD(TAG, "Boot timeout, component is ready to use");
    this->clear_rx_buffers_();
    this->set_next_state_(State::IDLE);
  });
}

void DlmsCosemComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DLMS-COSEM (SPODES):");
  LOG_UPDATE_INTERVAL(this);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Receive Timeout: %ums", this->receive_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Supported Meter Types: DLMS/COSEM (SPODES)");
  ESP_LOGCONFIG(TAG, "  Client address: %d", this->client_address_);
  ESP_LOGCONFIG(TAG, "  Server address: %d", this->server_address_);
  ESP_LOGCONFIG(TAG, "  Authentication: %s", this->auth_required_ == DLMS_AUTHENTICATION_NONE ? "None" : "Low");
  ESP_LOGCONFIG(TAG, "  P*ssword: %s", this->password_.c_str());
  ESP_LOGCONFIG(TAG, "  Sensors:");
  for (const auto &sensors : sensors_) {
    auto &s = sensors.second;
    ESP_LOGCONFIG(TAG, "    OBIS code: %s, Name: %s", s->get_obis_code().c_str(), s->get_sensor_name().c_str());
  }
}

void DlmsCosemComponent::register_sensor(DlmsCosemSensorBase *sensor) {
  this->sensors_.insert({sensor->get_obis_code(), sensor});
}

void DlmsCosemComponent::abort_mission_() {
  if (this->is_push_mode()) {
    ESP_LOGV(TAG, "Push mode error, returning to listening");
    this->clear_rx_buffers_();
    this->set_next_state_(State::IDLE);
    return;
  }

  // Existing pull mode logic
  ESP_LOGE(TAG, "Abort mission. Closing session");
  this->set_next_state_(State::MISSION_FAILED);
}

void DlmsCosemComponent::report_failure(bool failure) {
  if (!failure) {
    this->stats_.failures_ = 0;
    return;
  }

  this->stats_.failures_++;
  if (this->failures_before_reboot_ > 0 && this->stats_.failures_ > this->failures_before_reboot_) {
    ESP_LOGE(TAG, "Too many failures in a row. Let's try rebooting device.");
    delay(100);
    App.safe_reboot();
  }
}

void DlmsCosemComponent::loop() {
  if (!this->is_ready() || this->state_ == State::NOT_INITIALIZED)
    return;

  static bool hdlc_test = false;

  switch (this->state_) {
    case State::IDLE: {
      this->update_last_rx_time_();

      if (!this->is_push_mode()) {
        this->indicate_transmission(false);
        this->indicate_session(false);
      }

      // Push mode listening logic
      if (this->is_push_mode()) {
        if (this->available() > 0) {
          hdlc_test = false;
          // Set up for receiving push data
          memset(this->buffers_.in.data, 0, buffers_.in.capacity);
          this->buffers_.in.size = 0;
          // read what we can then move forward to avoid buffer overflow
          this->receive_frame_raw_();
          hdlc_test = this->buffers_.in.data[0] == HDLC_FLAG;

          ESP_LOGV(TAG, "Push mode: incoming data detected");
          this->stats_.connections_tried_++;
          this->loop_state_.session_started_ms = millis();

          this->indicate_transmission(true);

          reading_state_.next_state = State::PUSH_DATA_PROCESS;
          reading_state_.mission_critical = false;  // Never critical in push mode
          this->set_next_state_(State::COMMS_RX);
        }
      }
    } break;

    case State::TRY_LOCK_BUS: {
      this->log_state_();
      if (this->try_lock_uart_session_()) {
        this->indicate_session(true);
        this->indicate_connection(true);
        this->set_next_state_(State::OPEN_SESSION);
      } else {
        ESP_LOGV(TAG, "UART Bus is busy, waiting ...");
        this->set_next_state_delayed_(1000, State::TRY_LOCK_BUS);
      }
    } break;

    case State::WAIT:
      if (this->check_wait_timeout_()) {
        this->set_next_state_(this->wait_.next_state);
        this->update_last_rx_time_();
      }
      break;

    case State::COMMS_TX: {
      this->log_state_();
      this->indicate_transmission(true);
      if (buffers_.has_more_messages_to_send()) {
        send_dlms_messages_();
      } else {
        this->set_next_state_(State::COMMS_RX);
      }
    } break;

    case State::COMMS_RX: {
      this->log_state_();

      if (this->check_rx_timeout_()) {
        if (this->is_push_mode()) {
          ESP_LOGI(TAG, "Push data reception completed (timeout reached)");
        } else {
          ESP_LOGE(TAG, "RX timeout.");
          this->has_error = true;
          this->dlms_reading_state_.last_error = DLMS_ERROR_CODE_HARDWARE_FAULT;
          this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
        }

        this->indicate_connection(false);
        this->indicate_transmission(false);

        if (this->is_push_mode()) {
          // check if we received any data at all
          this->indicate_connection(true);
          if (this->buffers_.in.size > 0) {
            ESP_LOGV(TAG, "Push mode RX data available, len =%d, processing", this->buffers_.in.size);
            this->set_next_state_(State::PUSH_DATA_PROCESS);
          } else {
            ESP_LOGV(TAG, "Push mode RX timeout, returning to listening");
            this->set_next_state_(State::IDLE);
          }

        } else if (reading_state_.mission_critical) {
          ESP_LOGE(TAG, "Mission critical RX timeout.");
          this->abort_mission_();
        } else {
          // if not move forward
          reading_state_.err_invalid_frames++;
          this->set_next_state_(reading_state_.next_state);
        }
        return;
      }

      if (this->is_push_mode()) {
        received_frame_size_ = this->receive_frame_raw_();
        // this->update_last_rx_time_();
        //  keep reading until timeout
        return;
      }

      // the following basic algorithm to be implemented to read DLMS packet
      // first version, no retries
      // 1. receive proper hdlc frame
      // 2. get data from hdlc frame
      // 3. if ret = 0 or ret = DLMS_ERROR_CODE_FALSE then stop
      // 4. check reply->complete. if it is 0 then continue reading, go to 1
      //
      // read hdlc frame
      received_frame_size_ = this->receive_frame_hdlc_();

      if (received_frame_size_ == 0) {
        // keep reading until proper frame is received
        return;
      }

      this->update_last_rx_time_();

      // this->set_next_state_(reading_state_.next_state);

      auto ret = dlms_getData2(&dlms_settings_, &buffers_.in, &buffers_.reply, 0);
      if (ret != DLMS_ERROR_CODE_OK || buffers_.reply.complete == 0) {
        ESP_LOGVV(TAG, "dlms_getData2 ret = %d %s reply.complete = %d", ret, dlms_error_to_string(ret),
                  buffers_.reply.complete);
      }

      if (ret != DLMS_ERROR_CODE_OK && ret != DLMS_ERROR_CODE_FALSE) {
        ESP_LOGE(TAG, "dlms_getData2 failed. ret %d %s", ret, dlms_error_to_string(ret));
        this->reading_state_.err_invalid_frames++;
        this->set_next_state_(reading_state_.next_state);
        return;
      }

      if (buffers_.reply.complete == 0) {
        ESP_LOGD(TAG, "DLMS Reply not complete, need more HDLC frames. "
                      "Continue reading.");
        // data in multiple frames.
        // we just keep reading until full reply is received.
        return;  // keep reading
      }

      this->update_last_rx_time_();
      this->set_next_state_(reading_state_.next_state);

      auto parse_ret = this->dlms_reading_state_.parser_fn();
      this->dlms_reading_state_.last_error = parse_ret;

      if (parse_ret == DLMS_ERROR_CODE_OK) {
        //        ESP_LOGD(TAG, "DLSM parser fn result == DLMS_ERROR_CODE_OK");

      } else {
        ESP_LOGE(TAG, "DLMS parser fn error %d %s", parse_ret, dlms_error_to_string(parse_ret));

        if (this->is_push_mode()) {
          ESP_LOGV(TAG, "Push mode parse error, returning to listening");
          this->set_next_state_(State::IDLE);
        } else if (reading_state_.mission_critical) {
          this->abort_mission_();
        }
        // if not critical - just move forward
        // set_next_state_(State::IDLE);
      }
    } break;

    case State::MISSION_FAILED: {
      //  this->send_frame_(CMD_CLOSE_SESSION, sizeof(CMD_CLOSE_SESSION));
      if (!this->is_push_mode()) {
        this->unlock_uart_session_();
      }
      this->set_next_state_(State::IDLE);
      this->report_failure(true);
      this->stats_dump();
    } break;

    case State::OPEN_SESSION: {
      this->stats_.connections_tried_++;
      this->loop_state_.session_started_ms = millis();
      this->log_state_();
      this->clear_rx_buffers_();
      this->loop_state_.request_iter = this->sensors_.begin();

      this->set_next_state_(State::BUFFERS_REQ);

      // if (false) {
      //   // TODO. check if IEC handshake is needed

      //   uint8_t open_cmd[32]{0};
      //   uint8_t open_cmd_len = snprintf((char *) open_cmd, 32, "/?%s!\r\n",
      //   this->meter_address_.c_str()); request_iter = this->sensors_.begin();
      //   this->send_frame_(open_cmd, open_cmd_len);
      //   this->set_next_state_(State::OPEN_SESSION_GET_ID);
      //   auto read_fn = [this]() { return this->receive_frame_ascii_(); };
      //   // mission crit, no crc
      //   this->read_reply_and_go_next_state_(read_fn,
      //   State::OPEN_SESSION_GET_ID, 0, true, false);
      // }

    } break;

      // case State::OPEN_SESSION_GET_ID:
      //   this->log_state_();

      //   if (received_frame_size_) {
      //     char *id = nullptr;  //
      //     this->extract_meter_id_(received_frame_size_); if (id == nullptr) {
      //       ESP_LOGE(TAG, "Invalid meter identification frame");
      //       this->stats_.invalid_frames_++;
      //       this->abort_mission_();
      //       return;
      //     }

      //     this->update_last_rx_time_();
      //   }
      //   break;

    case State::BUFFERS_REQ: {
      this->log_state_();
      this->prepare_and_send_dlms_buffers();

    } break;

    case State::BUFFERS_RCV: {
      this->log_state_();
      // check the reply and go to next stage
      // todo smth with buffers reply
      this->set_next_state_(State::ASSOCIATION_REQ);

    } break;

    case State::ASSOCIATION_REQ: {
      this->log_state_();
      // this->start_comms_and_next(&aarq_rr_, State::ASSOCIATION_RCV);
      this->prepare_and_send_dlms_aarq();
    } break;

    case State::ASSOCIATION_RCV: {
      // check the reply and go to next stage
      // todo smth with aarq reply
      this->set_next_state_(State::DATA_ENQ_UNIT);
    } break;

    case State::DATA_ENQ_UNIT: {
      this->log_state_();
      if (this->loop_state_.request_iter == this->sensors_.end()) {
        ESP_LOGD(TAG, "All requests done");
        this->set_next_state_(State::SESSION_RELEASE);
        break;
      } else {
        auto req = this->loop_state_.request_iter->first;
        auto sens = this->loop_state_.request_iter->second;
        auto type = sens->get_obis_class();

        ESP_LOGD(TAG, "OBIS code: %s, Sensor: %s", req.c_str(), sens->get_sensor_name().c_str());

        // request units for numeric sensors only and only once
        if (sens->get_type() == SensorType::SENSOR && type == DLMS_OBJECT_TYPE_REGISTER &&
            !sens->has_got_scale_and_unit()) {
          // if (type == DLMS_OBJECT_TYPE_REGISTER)
          //        if (sens->get_attribute() != 2) {
          this->buffers_.gx_attribute = 3;
          this->prepare_and_send_dlms_data_unit_request(req.c_str(), type);
        } else {
          // units not working so far... so we are requesting just data
          this->set_next_state_(State::DATA_ENQ);
        }
      }
    } break;

    case State::DATA_ENQ: {
      this->log_state_();
      if (this->loop_state_.request_iter == this->sensors_.end()) {
        ESP_LOGD(TAG, "All requests done");
        this->set_next_state_(State::SESSION_RELEASE);
        break;
      } else {
        auto req = this->loop_state_.request_iter->first;
        auto sens = this->loop_state_.request_iter->second;
        auto type = sens->get_obis_class();
        auto units_were_requested = (sens->get_type() == SensorType::SENSOR && type == DLMS_OBJECT_TYPE_REGISTER &&
                                     !sens->has_got_scale_and_unit());
        if (units_were_requested) {
          auto ret = this->set_sensor_scale_and_unit(static_cast<DlmsCosemSensor *>(sens));
        }

        this->buffers_.gx_attribute = 2;
        this->prepare_and_send_dlms_data_request(req.c_str(), type, !units_were_requested);
      }
    } break;

    case State::DATA_RECV: {
      this->log_state_();
      this->set_next_state_(State::DATA_NEXT);

      auto req = this->loop_state_.request_iter->first;
      auto sens = this->loop_state_.request_iter->second;
      auto ret = this->set_sensor_value(sens, req.c_str());

    } break;

    case State::DATA_NEXT: {
      this->log_state_();
      this->loop_state_.request_iter = this->sensors_.upper_bound(this->loop_state_.request_iter->first);
      if (this->loop_state_.request_iter != this->sensors_.end()) {
        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::DATA_ENQ_UNIT);
      } else {
        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::SESSION_RELEASE);
      }
    } break;

    case State::SESSION_RELEASE: {
      this->loop_state_.sensor_iter = this->sensors_.begin();

      this->log_state_();
      ESP_LOGD(TAG, "Session release request");
      if (this->auth_required_) {
        this->prepare_and_send_dlms_release();
        break;
      } else {
        this->set_next_state_(State::DISCONNECT_REQ);
      }
    } break;

    case State::DISCONNECT_REQ: {
      this->log_state_();
      ESP_LOGD(TAG, "Disconnect request");
      this->prepare_and_send_dlms_disconnect();
    } break;

    case State::PUSH_DATA_PROCESS: {
      this->log_state_();
      ESP_LOGD(TAG, "Processing received push data");
      this->loop_state_.sensor_iter = this->sensors_.begin();
      this->set_next_state_(State::PUBLISH);
      this->process_push_data();
      this->clear_rx_buffers_();
    } break;

    case State::PUBLISH: {
      this->log_state_();
      ESP_LOGD(TAG, "Publishing data");
      this->update_last_rx_time_();

      if (this->loop_state_.sensor_iter != this->sensors_.end()) {
        if (this->loop_state_.sensor_iter->second->shall_we_publish()) {
          this->loop_state_.sensor_iter->second->publish();
        }
        this->loop_state_.sensor_iter++;
      } else {
        this->stats_dump();
        if (this->crc_errors_per_session_sensor_ != nullptr) {
          this->crc_errors_per_session_sensor_->publish_state(this->stats_.crc_errors_per_session());
        }
        this->report_failure(false);
        if (!this->is_push_mode()) {
          this->unlock_uart_session_();
        }
        this->set_next_state_(State::IDLE);
        ESP_LOGD(TAG, "Total time: %u ms", millis() - this->loop_state_.session_started_ms);
      }
    } break;

    default:
      break;
  }
}

void DlmsCosemComponent::update() {
  if (this->is_push_mode()) {
    // publish?
    return;
  }

  if (this->state_ != State::IDLE) {
    ESP_LOGD(TAG, "Starting data collection impossible - component not ready");
    return;
  }
  ESP_LOGD(TAG, "Starting data collection");
  this->has_error = false;
  this->set_next_state_(State::TRY_LOCK_BUS);
}

bool char2float(const char *str, float &value) {
  char *end;
  value = strtof(str, &end);
  return *end == '\0';
}

void DlmsCosemComponent::set_next_state_delayed_(uint32_t ms, State next_state) {
  if (ms == 0) {
    set_next_state_(next_state);
  } else {
    ESP_LOGV(TAG, "Short delay for %u ms", ms);
    set_next_state_(State::WAIT);
    wait_.start_time = millis();
    wait_.delay_ms = ms;
    wait_.next_state = next_state;
  }
}

void DlmsCosemComponent::InOutBuffers::init(size_t default_in_buf_size) {
  BYTE_BUFFER_INIT(&in);
  bb_capacity(&in, default_in_buf_size);
  mes_init(&out_msg);
  reply_init(&reply);
  this->reset();
}

void DlmsCosemComponent::InOutBuffers::reset() {
  mes_clear(&out_msg);
  reply_clear(&reply);
  reply.complete = 1;
  out_msg_index = 0;
  out_msg_data_pos = 0;
  in.size = 0;
  in.position = 0;
  //  amount_in = 0;
}

void DlmsCosemComponent::InOutBuffers::check_and_grow_input(uint16_t more_data) {
  const uint16_t GROW_EPSILON = 20;
  if (in.size + more_data > in.capacity) {
    ESP_LOGVV(TAG0, "Growing input buffer from %d to %d", in.capacity, in.size + more_data + GROW_EPSILON);
    bb_capacity(&in, in.size + more_data + GROW_EPSILON);
  }
}

void DlmsCosemComponent::prepare_and_send_dlms_buffers() {
  auto make = [this]() {
    ESP_LOGD(TAG0, "cl_snrmRequest %p ", this->buffers_.out_msg.data);
    return cl_snrmRequest(&this->dlms_settings_, &this->buffers_.out_msg);
  };
  auto parse = [this]() { return cl_parseUAResponse(&this->dlms_settings_, &this->buffers_.reply.data); };
  this->send_dlms_req_and_next(make, parse, State::BUFFERS_RCV, true);
}

void DlmsCosemComponent::prepare_and_send_dlms_aarq() {
  auto make = [this]() { return cl_aarqRequest(&this->dlms_settings_, &this->buffers_.out_msg); };
  auto parse = [this]() { return cl_parseAAREResponse(&this->dlms_settings_, &this->buffers_.reply.data); };
  this->send_dlms_req_and_next(make, parse, State::ASSOCIATION_RCV);
}

void DlmsCosemComponent::prepare_and_send_dlms_data_unit_request(const char *obis, int type) {
  auto ret = cosem_init(BASE(this->buffers_.gx_register), (DLMS_OBJECT_TYPE) type, obis);
  if (ret != DLMS_ERROR_CODE_OK) {
    ESP_LOGE(TAG, "cosem_init error %d '%s'", ret, dlms_error_to_string(ret));
    this->set_next_state_(State::DATA_ENQ);
    return;
  }

  auto make = [this]() {
    return cl_read(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
                   &this->buffers_.out_msg);
  };
  auto parse = [this]() {
    return cl_updateValue(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
                          &this->buffers_.reply.dataValue);
  };
  this->send_dlms_req_and_next(make, parse, State::DATA_ENQ, false, false);
}

void DlmsCosemComponent::prepare_and_send_dlms_data_request(const char *obis, int type, bool reg_init) {
  int ret = DLMS_ERROR_CODE_OK;
  if (reg_init) {
    ret = cosem_init(BASE(this->buffers_.gx_register), (DLMS_OBJECT_TYPE) type, obis);
  }
  if (ret != DLMS_ERROR_CODE_OK) {
    ESP_LOGE(TAG, "cosem_init error %d '%s'", ret, dlms_error_to_string(ret));
    this->set_next_state_(State::DATA_NEXT);
    return;
  }

  auto make = [this]() {
    return cl_read(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
                   &this->buffers_.out_msg);
  };
  auto parse = [this]() {
    return cl_updateValue(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
                          &this->buffers_.reply.dataValue);
  };
  this->send_dlms_req_and_next(make, parse, State::DATA_RECV);
}

void DlmsCosemComponent::prepare_and_send_dlms_release() {
  auto make = [this]() { return cl_releaseRequest(&this->dlms_settings_, &this->buffers_.out_msg); };
  auto parse = []() { return DLMS_ERROR_CODE_OK; };
  this->send_dlms_req_and_next(make, parse, State::DISCONNECT_REQ);
}

void DlmsCosemComponent::prepare_and_send_dlms_disconnect() {
  auto make = [this]() { return cl_disconnectRequest(&this->dlms_settings_, &this->buffers_.out_msg); };
  auto parse = []() { return DLMS_ERROR_CODE_OK; };
  this->send_dlms_req_and_next(make, parse, State::PUBLISH);
}

void DlmsCosemComponent::send_dlms_req_and_next(DlmsRequestMaker maker, DlmsResponseParser parser, State next_state,
                                                bool mission_critical, bool clear_buffer) {
  dlms_reading_state_.maker_fn = maker;
  dlms_reading_state_.parser_fn = parser;
  dlms_reading_state_.next_state = next_state;
  dlms_reading_state_.mission_critical = mission_critical;
  dlms_reading_state_.reply_is_complete = false;
  dlms_reading_state_.last_error = DLMS_ERROR_CODE_OK;

  // if (clear_buffer) {
  buffers_.reset();
  // }
  int ret = DLMS_ERROR_CODE_OK;
  if (maker != nullptr) {
    ret = maker();
    if (ret != DLMS_ERROR_CODE_OK) {
      ESP_LOGE(TAG, "Error in DLSM request maker function %d '%s'", ret, dlms_error_to_string(ret));
      this->set_next_state_(State::IDLE);
      return;
    }
  }

  reading_state_ = {};
  //  reading_state_.read_fn = read_fn;
  reading_state_.mission_critical = mission_critical;
  reading_state_.tries_max = 1;  // retries;
  reading_state_.tries_counter = 0;
  //  reading_state_.check_crc = check_crc;
  reading_state_.next_state = next_state;
  received_frame_size_ = 0;

  received_complete_reply_ = false;

  set_next_state_(State::COMMS_TX);
}

void DlmsCosemComponent::process_push_data() {
  ESP_LOGD(TAG, "Processing PUSH data frame with AXDR parser");

  // Ensure we parse from the beginning of the collected frame
  this->buffers_.in.position = 0;
  const auto total_size = this->buffers_.in.size;
  ESP_LOGD(TAG, "PUSH frame size: %u bytes", static_cast<unsigned>(total_size));

  AttributeDescriptorCallback fn = [this](auto... args) { (void) this->set_sensor_value(args...); };

  AxdrStreamParser axdr_parser(&this->buffers_.in, fn, this->push_show_log_);
  size_t total_objects = 0;
  size_t iterations = 0;

  while (this->buffers_.in.position < this->buffers_.in.size) {
    auto before = this->buffers_.in.position;
    auto parsed_now = axdr_parser.parse();
    auto after = this->buffers_.in.position;
    iterations++;

    if (parsed_now == 0 && after == before) {
      // No progress, avoid potential infinite loop on malformed frames
      ESP_LOGW(TAG, "AXDR parser made no progress at pos=%u/%u, aborting", static_cast<unsigned>(after),
               static_cast<unsigned>(this->buffers_.in.size));
      break;
    }
    total_objects += parsed_now;
    ESP_LOGV(TAG, "AXDR iteration %u: parsed=%u, pos=%u/%u, objects_total=%u", static_cast<unsigned>(iterations),
             static_cast<unsigned>(parsed_now), static_cast<unsigned>(after),
             static_cast<unsigned>(this->buffers_.in.size), static_cast<unsigned>(total_objects));
  }

  ESP_LOGD(TAG, "PUSH data parsing complete: %u objects, bytes consumed %u/%u", static_cast<unsigned>(total_objects),
           static_cast<unsigned>(this->buffers_.in.position), static_cast<unsigned>(total_size));
}

int DlmsCosemComponent::set_sensor_value(uint16_t class_id, const uint8_t *obis_code, uint8_t attribute_id,
                                         DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr,
                                         uint8_t value_length) {
  static char obis_buf[32];
  auto er = hlp_getLogicalNameToString(obis_code, obis_buf);

  std::string obis_str(obis_buf);

  auto range = this->sensors_.equal_range(obis_str);
  int found_count = 0;
  for (auto it = range.first; it != range.second; ++it) {
    DlmsCosemSensorBase *sensor = it->second;
    if (!sensor->shall_we_publish()) {
      continue;
    }
    ESP_LOGD(TAG, "Found sensor for OBIS code %s: '%s' ", obis_buf, sensor->get_sensor_name().c_str());
    found_count++;

#ifdef USE_SENSOR
    if (sensor->get_type() == SensorType::SENSOR) {
      float val = dlms_data_as_float(value_type, value_buffer_ptr, value_length);
      static_cast<DlmsCosemSensor *>(sensor)->set_value(val);
    }
#endif

#ifdef USE_TEXT_SENSOR
    if (sensor->get_type() == SensorType::TEXT_SENSOR) {
      auto val = dlms_data_as_string(value_type, value_buffer_ptr, value_length);
      static_cast<DlmsCosemTextSensor *>(sensor)->set_value(val.c_str(), this->cp1251_conversion_required_);
    }
#endif
  }

  if (found_count == 0) {
    ESP_LOGVV(TAG, "No sensor found for OBIS code: '%s'", (char *) obis_buf);
  } else {
    ESP_LOGVV(TAG, "Updated %d sensors for OBIS code: '%s'", found_count, (char *) obis_buf);
  }

  return DLMS_ERROR_CODE_OK;
}

int DlmsCosemComponent::set_sensor_scale_and_unit(DlmsCosemSensor *sensor) {
  ESP_LOGD(TAG, "set_sensor_scale_and_unit");
  if (!buffers_.reply.complete)
    return DLMS_ERROR_CODE_FALSE;
  auto vt = buffers_.reply.dataType;
  ESP_LOGD(TAG, "DLMS_DATA_TYPE: %s (%d)", dlms_data_type_to_string(vt), vt);
  if (vt != 0) {
    return DLMS_ERROR_CODE_FALSE;
  }

  auto scal = this->buffers_.gx_register.scaler;
  auto unit = this->buffers_.gx_register.unit;
  auto unit_s = obj_getUnitAsString(unit);
  sensor->set_scale_and_unit(scal, unit, unit_s);

  return DLMS_ERROR_CODE_OK;
}

int DlmsCosemComponent::set_sensor_value(DlmsCosemSensorBase *sensor, const char *obis) {
  if (!buffers_.reply.complete || !sensor->shall_we_publish()) {
    return this->dlms_reading_state_.last_error;
  }

  auto vt = buffers_.reply.dataType;
  ESP_LOGD(TAG, "OBIS code: %s, DLMS_DATA_TYPE: %s (%d)", obis, dlms_data_type_to_string(vt), vt);

  //      if (cosem_rr_.result().has_value()) {
  if (this->dlms_reading_state_.last_error == DLMS_ERROR_CODE_OK) {
    // result is okay, value shall be there

    if (sensor->get_type() == SensorType::SENSOR) {
      if (sensor->get_obis_class() == DLMS_OBJECT_TYPE_REGISTER) {
        auto var = &this->buffers_.gx_register.value;
        auto scale = static_cast<DlmsCosemSensor *>(sensor)->get_scale();
        auto unit = static_cast<DlmsCosemSensor *>(sensor)->get_unit();
        if (vt == DLMS_DATA_TYPE_FLOAT32 || vt == DLMS_DATA_TYPE_FLOAT64) {
          float val = var_toDouble(var);
          ESP_LOGD(TAG, "OBIS code: %s, Value: %f, Scale: %f, Unit: %s", obis, val, scale, unit);
          static_cast<DlmsCosemSensor *>(sensor)->set_value(val);
        } else {
          int val = var_toInteger(var);
          ESP_LOGD(TAG, "OBIS code: %s, Value: %d, Scale: %f, Unit: %s", obis, val, scale, unit);
          static_cast<DlmsCosemSensor *>(sensor)->set_value(val);
        }
      } else {
        ESP_LOGW(TAG, "Wrong OBIS class. Regular numberic sensors can only "
                      "handle Registers (class = 3)");
      }
    }

#ifdef USE_TEXT_SENSOR
    if (sensor->get_type() == SensorType::TEXT_SENSOR) {
      auto var = &this->buffers_.gx_register.value;
      if (var && var->byteArr && var->byteArr->size > 0) {
        auto arr = var->byteArr;

        ESP_LOGV(TAG, "data size=%d", arr->size);

        bb_setInt8(arr, 0);     // add null-termination
        if (arr->size > 128) {  // clip the string
          ESP_LOGW(TAG, "String is too long %d, clipping to 128 bytes", arr->size);
          arr->data[127] = '\0';
        }
        ESP_LOGV(TAG, "DATA: %s", format_hex_pretty(arr->data, arr->size).c_str());
        auto type = sensor->get_obis_class();

        if (type == DLMS_OBJECT_TYPE_DATA) {
          static_cast<DlmsCosemTextSensor *>(sensor)->set_value(reinterpret_cast<const char *>(arr->data),
                                                                this->cp1251_conversion_required_);
        } else if (type == DLMS_OBJECT_TYPE_CLOCK) {
          ESP_LOGD(TAG, "Clock sensor");
        }
      }
    }
#endif
  } else {
    ESP_LOGD(TAG, "OBIS code: %s, result != DLMS_ERROR_CODE_OK = %d", obis, this->dlms_reading_state_.last_error);
  }
  return this->dlms_reading_state_.last_error;
}

void DlmsCosemComponent::indicate_transmission(bool transmission_on) {
#ifdef USE_BINARY_SENSOR
  if (this->transmission_binary_sensor_) {
    this->transmission_binary_sensor_->publish_state(transmission_on);
  }
#endif
}

void DlmsCosemComponent::indicate_session(bool session_on) {
#ifdef USE_BINARY_SENSOR
  if (this->session_binary_sensor_) {
    this->session_binary_sensor_->publish_state(session_on);
  }
#endif
}

void DlmsCosemComponent::indicate_connection(bool connection_on) {
#ifdef USE_BINARY_SENSOR
  if (this->connection_binary_sensor_) {
    this->connection_binary_sensor_->publish_state(connection_on);
  }
#endif
}

void DlmsCosemComponent::send_dlms_messages_() {
  const int MAX_BYTES_IN_ONE_SHOT = 64;
  gxByteBuffer *buffer = buffers_.out_msg.data[buffers_.out_msg_index];

  int bytes_to_send = buffer->size - buffers_.out_msg_data_pos;
  if (bytes_to_send > 0) {
    if (bytes_to_send > MAX_BYTES_IN_ONE_SHOT)
      bytes_to_send = MAX_BYTES_IN_ONE_SHOT;

    if (this->flow_control_pin_ != nullptr)
      this->flow_control_pin_->digital_write(true);

    this->write_array(buffer->data + buffers_.out_msg_data_pos, bytes_to_send);

    if (this->flow_control_pin_ != nullptr)
      this->flow_control_pin_->digital_write(false);

    ESP_LOGVV(TAG, "TX: %s", format_hex_pretty(buffer->data + buffers_.out_msg_data_pos, bytes_to_send).c_str());

    this->update_last_rx_time_();
    buffers_.out_msg_data_pos += bytes_to_send;
  }
  if (buffers_.out_msg_data_pos >= buffer->size) {
    buffers_.out_msg_index++;
  }
}

size_t DlmsCosemComponent::receive_frame_(FrameStopFunction stop_fn) {
  const uint32_t read_time_limit_ms = 45;
  size_t ret_val;

  auto count_available = this->available();
  if (count_available <= 0)
    return 0;

  uint32_t read_start = millis();
  uint8_t *p;

  // ESP_LOGVV(TAG, "avail RX: %d", count_available);
  buffers_.check_and_grow_input(count_available);

  while (count_available-- > 0) {
    if (millis() - read_start > read_time_limit_ms) {
      return 0;
    }

    p = &this->buffers_.in.data[this->buffers_.in.size];
    if (!iuart_->read_one_byte(p)) {
      return 0;
    }
    this->buffers_.in.size++;
    // this->buffers_.amount_in++;

    if (stop_fn(this->buffers_.in.data, this->buffers_.in.size)) {
      //      ESP_LOGVV(TAG, "RX: %s",
      //      format_frame_pretty(this->buffers_.in.data,
      //      this->buffers_.in.size).c_str());
      ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(this->buffers_.in.data, this->buffers_.in.size).c_str());
      ret_val = this->buffers_.in.size;

      // this->buffers_.amount_in = 0;
      this->update_last_rx_time_();
      return ret_val;
    }

    yield();
    App.feed_wdt();
  }
  return 0;
}

size_t DlmsCosemComponent::receive_frame_hdlc_() {
  // HDLC frame: <FLAG>data<FLAG>
  auto frame_end_check_hdlc = [](uint8_t *b, size_t s) {
    auto ret = s >= 2 && b[0] == HDLC_FLAG && b[s - 1] == HDLC_FLAG;
    return ret;
  };
  return receive_frame_(frame_end_check_hdlc);
}

size_t DlmsCosemComponent::receive_frame_raw_() {
  auto frame_end_check_timeout = [](uint8_t *b, size_t s) {
    return false; // never stop by content, only by timeout
  };
  return receive_frame_(frame_end_check_timeout);
}

#ifdef IEC_HANDSHAKE
size_t DlmsCosemComponent::receive_frame_ascii_() {
  // "data<CR><LF>"
  ESP_LOGVV(TAG, "Waiting for ASCII frame");
  auto frame_end_check_crlf = [](uint8_t *b, size_t s) {
    auto ret = s >= 2 && b[s - 1] == '\n' && b[s - 2] == '\r';
    if (ret) {
      ESP_LOGVV(TAG, "Frame CRLF Stop");
    }
    return ret;
  };
  return receive_frame_(frame_end_check_crlf);
}
#endif

void DlmsCosemComponent::clear_rx_buffers_() {
  int available = this->available();
  if (available > 0) {
    ESP_LOGVV(TAG, "Cleaning garbage from UART input buffer: %d bytes", available);
  }

  int len;
  while (available > 0) {
    len = std::min(available, (int) buffers_.in.capacity);
    this->read_array(this->buffers_.in.data, len);
    available -= len;
  }
  memset(this->buffers_.in.data, 0, buffers_.in.capacity);
  this->buffers_.in.size = 0;
  this->buffers_.in.position = 0;
}

const char *DlmsCosemComponent::state_to_string(State state) {
  switch (state) {
    case State::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case State::IDLE:
      return "IDLE";
    case State::TRY_LOCK_BUS:
      return "TRY_LOCK_BUS";
    case State::WAIT:
      return "WAIT";
    case State::COMMS_TX:
      return "COMMS_TX";
    case State::COMMS_RX:
      return "COMMS_RX";
    case State::MISSION_FAILED:
      return "MISSION_FAILED";
    case State::OPEN_SESSION:
      return "OPEN_SESSION";
    case State::BUFFERS_REQ:
      return "BUFFERS_REQ";
    case State::BUFFERS_RCV:
      return "BUFFERS_RCV";
    case State::ASSOCIATION_REQ:
      return "ASSOCIATION_REQ";
    case State::ASSOCIATION_RCV:
      return "ASSOCIATION_RCV";
    case State::DATA_ENQ_UNIT:
      return "DATA_ENQ_UNIT";
    case State::DATA_ENQ:
      return "DATA_ENQ";
    case State::DATA_RECV:
      return "DATA_RECV";
    case State::DATA_NEXT:
      return "DATA_NEXT";
    case State::SESSION_RELEASE:
      return "SESSION_RELEASE";
    case State::DISCONNECT_REQ:
      return "DISCONNECT_REQ";
    case State::PUBLISH:
      return "PUBLISH";
    case State::PUSH_DATA_PROCESS:
      return "PUSH_DATA_PROCESS";
    default:
      return "UNKNOWN";
  }
}

void DlmsCosemComponent::log_state_(State *next_state) {
  if (this->state_ != this->last_reported_state_) {
    if (next_state == nullptr) {
      ESP_LOGV(TAG, "State::%s", state_to_string(this->state_));
    } else {
      ESP_LOGV(TAG, "State::%s -> %s", state_to_string(this->state_), state_to_string(*next_state));
    }
    this->last_reported_state_ = this->state_;
  }
}

void DlmsCosemComponent::stats_dump() {
  ESP_LOGV(TAG, "============================================");
  ESP_LOGV(TAG, "Data collection and publishing finished.");
  ESP_LOGV(TAG, "Total number of sessions ............. %u", this->stats_.connections_tried_);
  ESP_LOGV(TAG, "Total number of invalid frames ....... %u", this->stats_.invalid_frames_);
  ESP_LOGV(TAG, "Total number of CRC errors ........... %u", this->stats_.crc_errors_);
  ESP_LOGV(TAG, "Total number of CRC errors recovered . %u", this->stats_.crc_errors_recovered_);
  ESP_LOGV(TAG, "CRC errors per session ............... %f", this->stats_.crc_errors_per_session());
  ESP_LOGV(TAG, "Number of failures ................... %u", this->stats_.failures_);
  ESP_LOGV(TAG, "============================================");
}

bool DlmsCosemComponent::try_lock_uart_session_() {
  if (AnyObjectLocker::try_lock(this->parent_)) {
    ESP_LOGV(TAG, "UART bus %p locked by %s", this->parent_, this->tag_.c_str());
    return true;
  }
  ESP_LOGV(TAG, "UART bus %p busy", this->parent_);
  return false;
}

void DlmsCosemComponent::unlock_uart_session_() {
  AnyObjectLocker::unlock(this->parent_);
  ESP_LOGV(TAG, "UART bus %p released by %s", this->parent_, this->tag_.c_str());
}

uint8_t DlmsCosemComponent::next_obj_id_ = 0;

std::string DlmsCosemComponent::generateTag() { return str_sprintf("%s%03d", TAG0, ++next_obj_id_); }

// bool ApduParser::is_numeric_type(DLMS_DATA_TYPE type) {
//   return type == DLMS_DATA_TYPE_BOOLEAN || type == DLMS_DATA_TYPE_INT32 || type == DLMS_DATA_TYPE_UINT32 ||
//          type == DLMS_DATA_TYPE_INT8 || type == DLMS_DATA_TYPE_INT16 || type == DLMS_DATA_TYPE_UINT8 ||
//          type == DLMS_DATA_TYPE_UINT16 || type == DLMS_DATA_TYPE_INT64 || type == DLMS_DATA_TYPE_UINT64 ||
//          type == DLMS_DATA_TYPE_ENUM || type == DLMS_DATA_TYPE_FLOAT32 || type == DLMS_DATA_TYPE_FLOAT64;
//   //  // ||
//   //  type == DLMS_DATA_TYPE_DELTA_INT8 ||
//   //  type == DLMS_DATA_TYPE_DELTA_INT16 ||
//   //  type == DLMS_DATA_TYPE_DELTA_INT32 ||
//   //  type == DLMS_DATA_TYPE_DELTA_UINT8 ||
//   //  type == DLMS_DATA_TYPE_DELTA_UINT16 ||
//   //  type == DLMS_DATA_TYPE_DELTA_UINT32;
// }

// float ApduParser::read_numeric_to_float(DLMS_DATA_TYPE type) {
//   float val = 0;
//   auto type_size = hlp_getDataTypeSize(type);
//   if (type_size <= 0) {
//     ESP_LOGE(TAG, "Type %d has variable size, cannot read as float", (uint8_t) type);
//     return val;
//   }

//   if (this->buffer->position + type_size > this->buffer->size) {
//     ESP_LOGE(TAG, "Not enough data to read type %d as float", type);
//     return val;
//   }

//   switch (type) {
//       // one byte unsigned
//     case DLMS_DATA_TYPE_BOOLEAN:
//     case DLMS_DATA_TYPE_UINT8:
//     case DLMS_DATA_TYPE_ENUM:
//       // case DLMS_DATA_TYPE_DELTA_UINT8:
//       {
//         uint8_t b1 = read_byte();
//         val = static_cast<float>(b1);
//       }
//       break;

//     // one byte signed
//     case DLMS_DATA_TYPE_INT8:
//       // case DLMS_DATA_TYPE_DELTA_INT8:
//       {
//         int8_t b1 = static_cast<int8_t>(read_byte());
//         val = static_cast<float>(b1);
//       }
//       break;

//     // two byte unsigned
//     case DLMS_DATA_TYPE_UINT16:
//       // case DLMS_DATA_TYPE_DELTA_UINT16:
//       {
//         uint16_t uint16Value = read_u16();
//         val = static_cast<float>(uint16Value);
//       }
//       break;

//     // two byte signed
//     case DLMS_DATA_TYPE_INT16:
//       // case DLMS_DATA_TYPE_DELTA_INT16:
//       {
//         uint16_t uint16Value = read_u16();
//         int16_t int16Value = static_cast<int16_t>(uint16Value);
//         val = static_cast<float>(int16Value);
//       }
//       break;

//     // four byte unsigned
//     case DLMS_DATA_TYPE_UINT32:
//       // case DLMS_DATA_TYPE_DELTA_UINT32:
//       {
//         uint32_t uint32Value = read_u32();
//         val = static_cast<float>(uint32Value);
//       }
//       break;

//     // four byte signed
//     case DLMS_DATA_TYPE_INT32:
//       // case DLMS_DATA_TYPE_DELTA_INT32:
//       {
//         uint32_t uint32Value = read_u32();
//         int32_t int32Value = static_cast<int32_t>(uint32Value);
//         val = static_cast<float>(int32Value);
//       }
//       break;

//     case DLMS_DATA_TYPE_UINT64:
//     case DLMS_DATA_TYPE_INT64: {
//       uint64_t raw = 0;
//       for (int i = 0; i < 8; i++) {
//         raw = (raw << 8) | read_byte();
//       }
//       if (type == DLMS_DATA_TYPE_UINT64)
//         val = static_cast<float>(raw);
//       else
//         val = static_cast<float>((int64_t) raw);
//     } break;

//     case DLMS_DATA_TYPE_FLOAT32: {
//       uint32_t raw = read_u32();
//       float f{};
//       if (sizeof(float) == 4) {
//         std::memcpy(&f, &raw, sizeof(float));
//         val = f;
//       } else {
//         ESP_LOGW(TAG, "Float size is not 4 bytes on this platform!");
//       }
//     } break;

//     case DLMS_DATA_TYPE_FLOAT64: {
//       uint64_t raw = 0;
//       for (int i = 0; i < 8; i++) {
//         raw = (raw << 8) | read_byte();
//       }
//       if (sizeof(double) == 8) {
//         double d{};
//         std::memcpy(&d, &raw, sizeof(double));
//         val = static_cast<float>(d);
//       } else {
//         ESP_LOGW(TAG, "Double size is not 8 bytes on this platform!");
//       }
//     } break;

//     default:
//       ESP_LOGE(TAG, "Unsupported numeric type: %d at position %d", (uint8_t) (type), this->buffer->position);
//   }

//   return val;
// }

// uint8_t ApduParser::peek_byte() {
//   if (this->buffer->position + 1 > this->buffer->size) {
//     ESP_LOGE(TAG, "Buffer overflow in read_byte at position %d", this->buffer->position);
//     return 0xFF;
//   }
//   // ESP_LOGVV(TAG, "Peek [%04d]: %02X", this->buffer->position, this->buffer->data[this->buffer->position]);
//   return this->buffer->data[this->buffer->position];
// }

// uint8_t ApduParser::read_byte() {
//   if (this->buffer->position + 1 > this->buffer->size) {
//     ESP_LOGE(TAG, "Buffer overflow in read_byte at position %d", this->buffer->position);
//     return 0xFF;
//   }
//   // ESP_LOGVV(TAG, "Read [%04d]: %02X", this->buffer->position, this->buffer->data[this->buffer->position]);
//   return this->buffer->data[this->buffer->position++];
// }

// uint16_t ApduParser::read_u16() {
//   if (this->buffer->position + 2 > this->buffer->size) {
//     ESP_LOGE(TAG, "Buffer overflow in read_u16 at position %d", this->buffer->position);
//     return 0;
//   }
//   uint16_t value = (this->buffer->data[this->buffer->position] << 8) | this->buffer->data[this->buffer->position +
//   1]; this->buffer->position += 2; return value;
// }

// uint32_t ApduParser::read_u32() {
//   if (this->buffer->position + 4 > this->buffer->size) {
//     ESP_LOGE(TAG, "Buffer overflow in read_u32 at position %d", this->buffer->position);
//     return 0;
//   }
//   uint32_t value =
//       (this->buffer->data[this->buffer->position] << 24) | (this->buffer->data[this->buffer->position + 1] << 16) |
//       (this->buffer->data[this->buffer->position + 2] << 8) | this->buffer->data[this->buffer->position + 3];
//   this->buffer->position += 4;
//   return value;
// }

// bool ApduParser::read_obis_bytes(uint8_t *dest_string, const size_t dest_size) {
//   if (this->buffer->position + 6 > this->buffer->size) {
//     return false;
//   }
//   uint8_t a = read_byte();
//   uint8_t b = read_byte();
//   uint8_t c = read_byte();
//   uint8_t d = read_byte();
//   uint8_t e = read_byte();
//   uint8_t f = read_byte();
//   if (dest_string != nullptr && dest_size > 0) {
//     snprintf((char *) dest_string, dest_size, "%d.%d.%d.%d.%d.%d\0", a, b, c, d, e, f);
//   }
//   return true;
// }

// size_t ApduParser::test_if_date_time() {
//   // only 12 bytes for now

//   if (this->buffer->position + 12 > this->buffer->size) {
//     return 0;
//   }

//   const uint8_t *buf = this->buffer->data + this->buffer->position;
//   if (!buf)
//     return 0;

//   // Year
//   uint16_t year = (buf[0] << 8) | buf[1];
//   if (!(year == 0x0000 || (year >= 1970 && year <= 2100)))
//     return 0;

//   // Month
//   uint8_t month = buf[2];
//   if (!(month == 0xFF || (month >= 1 && month <= 12)))
//     return 0;

//   // Day of month
//   uint8_t day = buf[3];
//   if (!(day == 0xFF || (day >= 1 && day <= 31)))
//     return 0;

//   // Day of week
//   uint8_t dow = buf[4];
//   if (!(dow == 0xFF || (dow >= 1 && dow <= 7)))
//     return 0;

//   // Hour
//   uint8_t hour = buf[5];
//   if (!(hour == 0xFF || hour <= 23))
//     return 0;

//   // Minute
//   uint8_t minute = buf[6];
//   if (!(minute == 0xFF || minute <= 59))
//     return 0;

//   // Second
//   uint8_t second = buf[7];
//   if (!(second == 0xFF || second <= 59))
//     return 0;

//   // Hundredths of second
//   uint16_t ms = (buf[8] << 8) | buf[9];
//   if (!(ms == 0xFFFF || ms <= 99))
//     return 0;

//   // Deviation (timezone offset, signed, 2 bytes)
//   uint16_t u_dev = buf[10] | (buf[11] << 8);
//   int16_t s_dev = (int16_t) (u_dev);
//   if (!(s_dev == (int16_t) 0x8000 || (s_dev >= -720 && s_dev <= 720)))
//     return 0;

//   return 12;
// }

// bool ApduParser::scan_cosem_objects(uint8_t lvl, DLMS_DATA_TYPE parent_type, uint8_t idx_in_structure) {
//   if (this->buffer->position >= this->buffer->size) {
//     ESP_LOGE(TAG, "Buffer overflow in scan_cosem_objects at position %d, level %d, parent type %d",
//              this->buffer->position, lvl, parent_type);
//     return false;
//   }

//   DLMS_DATA_TYPE type = (DLMS_DATA_TYPE) read_byte();

//   ESP_LOGVV(TAG, "Position %d: Found type %s (%d). Lvl %d", this->buffer->position - 1,
//   dlms_data_type_to_string(type),
//             (int) type, lvl);

//   if (type == DLMS_DATA_TYPE_NONE) {
//     // ZPA AM375 sends structures with no elements, just a NONE type then goes attribute descriptor
//     // Cosem object := {Attribute descriptor, Value}
//     // Attribute descriptor: := <NULL>, OBIS code, ATTR, VALUE
//     // Value: := <NONE> | <simple type> | <array> | <structure> | <enum>
//     if (idx_in_structure == 0 && parent_type == DLMS_DATA_TYPE_STRUCTURE) {
//       object.class_id = read_byte();
//       uint8_t a = read_byte();
//       uint8_t b = read_byte();
//       uint8_t c = read_byte();
//       uint8_t d = read_byte();
//       uint8_t e = read_byte();
//       uint8_t f = read_byte();
//       snprintf((char *) object.obis_code, sizeof(object.obis_code), "%d.%d.%d.%d.%d.%d\0", a, b, c, d, e, f);
//       object.attribute = read_byte();
//       object.octet_obis_detected = true;
//       object.lvl_when_obis_detected = lvl;
//       ESP_LOGI(TAG, "Found COSEM object: class=%d, obis=%s, attr=%d", object.class_id, (char *) object.obis_code,
//                object.attribute);
//       // Now read the value
//     }

//     return true;
//   } else if (type == DLMS_DATA_TYPE_ARRAY) {
//     // simple iteration
//     uint8_t elements_count = read_byte();
//     while (elements_count-- > 0) {
//       if (!scan_cosem_objects(lvl + 1, type))
//         return false;
//     }
//     return true;
//   } else if (type == DLMS_DATA_TYPE_STRUCTURE) {
//     // here be dragons!!!
//     // since we never know we need to guess
//     //
//     //
//     //
//     uint8_t elements_count = read_byte();
//     if (elements_count == 0) {
//       printf("Structure with zero elements at position %d", this->buffer->position);
//       return true;
//     }
//     uint8_t peek_first_object_type = peek_byte();
//     uint8_t peek_object_type;
//     for (uint8_t i = 0; i < elements_count; i++) {
//       peek_object_type = peek_byte();
//       if (!scan_cosem_objects(lvl + 1, type, i))
//         return false;
//     }
//     if (this->object.octet_obis_detected && this->object.lvl_when_obis_detected - 1 == lvl) {
//       // we had detected OBIS code in octet string, so we can assume this is a COSEM object
//       peek_first_object_type = DLMS_DATA_TYPE_NONE;
//     }

//     if (peek_first_object_type == DLMS_DATA_TYPE_NONE) {
//       // we have just read a structure/array that contains COSEM object(s)
//       printf(
//           "Completed reading COSEM object: class=%d, obis=%s, attr=%d, value type=%s (%d), numeric val=%f,
//           str='%s'\n", object.class_id, (char *) object.obis_code, object.attribute,
//           dlms_data_type_to_string(object.value_type), (int) object.value_type, object.value_numeric,
//           object.value_str.c_str());
//       // TODO: we might want to check attribute 2 or 3 and do some actions

//       this->objects_found++;
//       if (this->register_object_fn_) {
//         this->register_object_fn_(this->object);
//       }
//       this->object.reset();
//     }

//     return true;
//   } else if (is_numeric_type(type)) {
//     this->object.value_type = type;
//     this->object.value_numeric = read_numeric_to_float(type);
//     return true;
//   } else if (type == DLMS_DATA_TYPE_OCTET_STRING || type == DLMS_DATA_TYPE_STRING ||
//              type == DLMS_DATA_TYPE_STRING_UTF8) {
//     this->object.value_type = type;
//     uint8_t len = read_byte();
//     // OBIS code is often sent as octet string of length 6 inside a structure
//     if (len == 6 && type == DLMS_DATA_TYPE_OCTET_STRING && parent_type == DLMS_DATA_TYPE_STRUCTURE &&
//         this->object.obis_code[0] == 0) {
//       uint8_t a = read_byte();
//       uint8_t b = read_byte();
//       uint8_t c = read_byte();
//       uint8_t d = read_byte();
//       uint8_t e = read_byte();
//       uint8_t f = read_byte();
//       snprintf((char *) object.obis_code, sizeof(object.obis_code), "%d.%d.%d.%d.%d.%d\0", a, b, c, d, e, f);
//       this->object.octet_obis_detected = true;
//       this->object.lvl_when_obis_detected = lvl;
//     } else {
//       this->object.value_str =
//           std::string(this->buffer->data + this->buffer->position, this->buffer->data + this->buffer->position +
//           len);
//       this->buffer->position += len;
//     }

//     return true;
//   } else if (type == DLMS_DATA_TYPE_BIT_STRING) {
//     this->object.value_type = type;
//     // length of the bit string
//     uint8_t len = read_byte();
//     // Calculate bytes needed to hold the bits
//     uint8_t bytes_needed = (len + 7) / 8;
//     while (bytes_needed-- > 0) {
//       read_byte();  // Just consume the bytes for now
//     }
//     ESP_LOGW(TAG, "Bit string not yet supported. Bit length %d bits", static_cast<int>(len));
//     return true;
//   } else if (type == DLMS_DATA_TYPE_COMPACT_ARRAY) {
//     this->object.value_type = type;
//     uint8_t len = read_byte();
//     while (len-- > 0) {
//       read_byte();  // Just consume the bytes for now
//     }
//     ESP_LOGW(TAG, "Compact Array type not yet supported.");
//     return true;
//   } else if (type == DLMS_DATA_TYPE_DATETIME || type == DLMS_DATA_TYPE_DATE || type == DLMS_DATA_TYPE_TIME) {
//     this->object.value_type = type;
//     uint8_t len = read_byte();
//     while (len-- > 0) {
//       read_byte();  // Just consume the bytes for now
//     }
//     return true;
//   } else {
//     // here be dragons
//     ESP_LOGE(TAG, "Unknown data type. Parsing failed. %d at position %d", static_cast<int>(type),
//              this->buffer->position);
//     return false;
//   }
//   if (this->buffer->position == this->buffer->size) {
//     ESP_LOGD(TAG, "End of buffer reached in scan_cosem_objects at position %d, level %d, parent type %d",
//              this->buffer->position, lvl, parent_type);
//     return true;
//   }

//   /*
//   // else if (type == DLMS_DATA_TYPE_BINARY_CODED_DECIMAL) {
//   //   this->object.value_type = type;
//   //   uint8_t len = read_byte();
//   //   //    std::vector<uint8_t> bcdData = readBytes(len);
//   //   std::stringstream ss;
//   //   for (uint8_t i = 0; i < len; i++) {
//   //     ss << std::hex << std::setw(2) << std::setfill('0')
//   //        << static_cast<int>(this->buffer->data[this->buffer->position + i]);
//   //   }
//   //   this->object.value_str = ss.str();
//   //   uint32_t value32 = std::stoul(this->object.value_str, nullptr, 10);
//   //   this->object.numeric_value = static_cast<float>(value32);
//   //   return true;
//   */
//   return false;
// }

// size_t ApduParser::parse() {
//   static const size_t minimum_push_frame_size = 1 + 4 + 1 + 2;
//   // flag +
//   // invoke id +
//   // priority +
//   // [optional] datetime
//   // at least one empty structure/array

//   uint8_t flag = 0;

//   // skip to notification flag
//   while (this->buffer->position < this->buffer->size) {
//     flag = read_byte();
//     if (flag == 0x0F)
//       break;
//   }

//   if (flag != 0x0F) {
//     ESP_LOGE(TAG, "No push notification flag found in the frame");
//     return 0;
//   }

//   if (this->buffer->position + minimum_push_frame_size >= this->buffer->size) {
//     ESP_LOGV(TAG, "Push notification frame too short: %d bytes", this->buffer->size);
//     return 0;
//   }
//   uint32_t invoke_id = read_u32();
//   uint8_t priority = read_byte();

//   // sometimes there is a DateTime object here, sometimes not
//   size_t date_time_to_skip = test_if_date_time();
//   this->buffer->position += date_time_to_skip;

//   this->scan_cosem_objects();
// }

}  // namespace dlms_cosem
}  // namespace esphome
