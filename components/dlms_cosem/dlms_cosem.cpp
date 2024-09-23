
#include "dlms_cosem.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <sstream>

namespace esphome {
namespace dlms_cosem {

// static const char *TAG = "dlms_cosem";

// constexpr uint16_t HDLC_CRC16[256] = {
//     0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48,
//     0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108,
//     0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E, 0x9CC9, 0x8D40, 0xBFDB,
//     0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102, 0x308B, 0x0210, 0x1399,
//     0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E,
//     0xFAE7, 0xC87C, 0xD9F5, 0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E,
//     0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD,
//     0xC974, 0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
//     0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3, 0x5285,
//     0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44,
//     0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72, 0x6306, 0x728F, 0x4014,
//     0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5,
//     0xA96A, 0xB8E3, 0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3,
//     0x242A, 0x16B1, 0x0738, 0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862,
//     0x9AF9, 0x8B70, 0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E,
//     0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
//     0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1,
//     0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483,
//     0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5, 0x2942, 0x38CB, 0x0A50,
//     0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD, 0xB58B, 0xA402, 0x9699, 0x8710,
//     0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7,
//     0x6E6E, 0x5CF5, 0x4D7C, 0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1,
//     0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72,
//     0x3EFB, 0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
//     0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A, 0xE70E,
//     0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF,
//     0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9, 0xF78F, 0xE606, 0xD49D,
//     0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C,
//     0x3DE3, 0x2C6A, 0x1EF1, 0x0F78};

// uint16_t crc16_hdlc(const uint8_t *data, size_t index, size_t length) {
//   uint16_t crc = 0xFFFF;
//   for (size_t pos = index; pos < index + length; ++pos) {
//     crc = (crc >> 8) ^ HDLC_CRC16[(crc ^ data[pos]) & 0xFF];
//   }
//   crc = ~crc;
//   crc = ((crc >> 8) & 0xFF) | (crc << 8);
//   return crc;
// }

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

static constexpr uint8_t BOOT_WAIT_S = 15;  // 10;

static char empty_str[] = "";

static char format_hex_char(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }

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
          ss << "<" << format_hex_char((data[i] & 0xF0) >> 4) << format_hex_char(data[i] & 0x0F) << ">";
        } else {
          ss << (char) data[i];
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

void DlmsCosemComponent::set_baud_rate_(uint32_t baud_rate) {
  ESP_LOGV(TAG, "Setting baud rate %u bps", baud_rate);
  iuart_->update_baudrate(baud_rate);
}

void DlmsCosemComponent::setup() {
  ESP_LOGD(TAG, "setup");

  // cl_init(&dlms_settings_, true /*logicalname*/, 2, 0x90,
  // DLMS_AUTHENTICATION_LOW, "00000001",DLMS_INTERFACE_TYPE_HDLC);

  cl_init(&dlms_settings_, true, this->client_address_, this->server_address_,
          this->auth_required_ ? DLMS_AUTHENTICATION_LOW : DLMS_AUTHENTICATION_NONE,
          this->auth_required_ ? this->password_.c_str() : NULL, DLMS_INTERFACE_TYPE_HDLC);

  this->buffers_rr_.set_settings(&dlms_settings_);
  this->aarq_rr_.set_settings(&dlms_settings_);
  this->cosem_rr_.set_settings(&dlms_settings_);
  this->session_release_rr_.set_settings(&dlms_settings_);
  this->disconnect_rr_.set_settings(&dlms_settings_);

  BYTE_BUFFER_INIT(&this->buffers_.in);

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
    ESP_LOGCONFIG(TAG, "    REQUEST: %s", s->get_obis_code().c_str());
  }
}

void DlmsCosemComponent::register_sensor(DlmsCosemSensorBase *sensor) {
  this->sensors_.insert({sensor->get_obis_code(), sensor});
}

void DlmsCosemComponent::abort_mission_() {
  // try close connection ?
  ESP_LOGE(TAG, "Closing session");
  //  this->send_frame_(CMD_CLOSE_SESSION, sizeof(CMD_CLOSE_SESSION));
  this->set_next_state_(State::IDLE);
  this->report_failure(true);
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

// part of the loop() for the DLMS/COSEM component which is responsible for the
// communications:
// - CHECK_METER_BUFFERS - send SNRM request ()
// - READ_METER_UA       - wait for response, parse UA response (UA frame, type
// 73)
// - ASSOCIATION_REQUEST - send HDLC frame (AARQ frame, type 10) with
// association request
// - ASSOCIATION_RESPONSE - wait for association response (AARE frame, type 30)
// - GET_REQUEST         - send HDLC frame (I frame , action request) and wait
// for action response
// - GET_RESPONSE        - (loop back to ACTION_REQUEST)
// - DISCONNECT_REQUEST  - send HDLC frame (DISC frame, type 0x43) and wait for
// disconnect response
// - DISCONNECT_ACCEPTED - then go to IDLE state
void DlmsCosemComponent::loop() {
  if (!this->is_ready() || this->state_ == State::NOT_INITIALIZED)
    return;

  // in-loop static variables
  static uint32_t session_started_ms{0};            // start of session
  static auto request_iter = this->sensors_.end();  // talking to meter
  static auto sensor_iter = this->sensors_.end();   // publishing sensor values
  // static ValueRefsArray vals;                                  // values from brackets, refs to this->buffers_.in
  // static char *in_param_ptr = (char *) &this->buffers_.in[1];  // ref to second byte, first is STX/SOH in R1 requests

  switch (this->state_) {
    case State::IDLE: {
      this->update_last_rx_time_();
    } break;

    case State::WAIT:
      if (this->check_wait_timeout_()) {
        this->set_next_state_(this->wait_.next_state);
        this->update_last_rx_time_();
      }
      break;

    case State::COMMS_TX: {
      this->log_state_();
      if (buffers_.has_more_messages_to_send()) {
        ESP_LOGVV(TAG, "There are messages to send.");
        send_dlms_messages_();
      } else {
        ESP_LOGVV(TAG, "No more messages to send.");
        this->set_next_state_(State::COMMS_RX);
      }
    } break;

    case State::COMMS_RX: {
      this->log_state_();

      if (this->check_rx_timeout_()) {
        ESP_LOGE(TAG, "RX timeout.");
        this->dlms_reading_state_.last_error = DLMS_ERROR_CODE_HARDWARE_FAULT;
        this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
        // if mission critical
        if (reading_state_.mission_critical) {
          this->abort_mission_();
        } else {
          // if not move forward
          reading_state_.err_invalid_frames++;
          this->set_next_state_(reading_state_.next_state);
        }
        return;
      }

      // the folowing basic algorithm to be implemented to read DLMS packet
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
      ESP_LOGVV(TAG, "dlms_getData2 ret = %d %s reply.complete = %d", ret, this->dlms_error_to_string(ret),
                buffers_.reply.complete);

      if (ret != DLMS_ERROR_CODE_OK && ret != DLMS_ERROR_CODE_FALSE) {
        ESP_LOGE(TAG, "dlms_getData2 failed. ret %d %s", ret, this->dlms_error_to_string(ret));
        // current_rr_->set_error();
        this->set_next_state_(reading_state_.next_state);
        return;
      }

      if (buffers_.reply.complete == 0) {
        ESP_LOGD(TAG, "DLMS Reply not complete, need more HDLC frames. Continue reading.");
        // data in multiple frames.
        // never tested.
        // keep reading until full reply is received.
        return;
      }

      // if buffers_.reply.complete != 0
      this->update_last_rx_time_();
      this->set_next_state_(reading_state_.next_state);

      // current_rr_->parse(&buffers_.reply);
      auto parse_ret = this->dlms_reading_state_.parser_fn();
      this->dlms_reading_state_.last_error = parse_ret;

      if (parse_ret == DLMS_ERROR_CODE_OK) {
        ESP_LOGD(TAG, "DLSM parser fn result == DLMS_ERROR_CODE_OK");
      } else {
        ESP_LOGE(TAG, "DLSM parser fn error %d %s", this->dlms_error_to_string(parse_ret));
        set_next_state_(State::IDLE);
      }
      // if (current_rr_->result().has_value()) {
      //   ESP_LOGD(TAG, "In result has value");
      //   if (*current_rr_->result() == DLMS_ERROR_CODE_OK) {
      //     ESP_LOGD(TAG, "In result == DLMS_ERROR_CODE_OK");
      //   } else {
      //     ESP_LOGE(TAG, "Error in state %s", this->state_to_string(state_));
      //     state_ = State::IDLE;
      //   }
      // } else {
      //   ESP_LOGD(TAG, "No result");
      // }
    } break;

      // case State::WAITING_FOR_RESPONSE: {
      //   this->log_state_(&reading_state_.next_state);
      //   received_frame_size_ = reading_state_.read_fn();

      //   bool crc_is_ok = true;
      //   // if (reading_state_.check_crc && received_frame_size_ > 0) {
      //   //   crc_is_ok = check_crc_prog_frame_(this->buffers_.in,
      //   received_frame_size_);
      //   // }

      //   // happy path first
      //   if (received_frame_size_ > 0 && crc_is_ok) {
      //     this->set_next_state_(reading_state_.next_state);
      //     this->update_last_rx_time_();
      //     this->stats_.crc_errors_ += reading_state_.err_crc;
      //     this->stats_.crc_errors_recovered_ += reading_state_.err_crc;
      //     this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
      //     return;
      //   }

      //   // half-happy path
      //   // if not timed out yet, wait for data to come a little more
      //   if (crc_is_ok && !this->check_rx_timeout_()) {
      //     return;
      //   }

      //   if (received_frame_size_ == 0) {
      //     this->reading_state_.err_invalid_frames++;
      //     ESP_LOGW(TAG, "RX timeout.");
      //   } else if (!crc_is_ok) {
      //     this->reading_state_.err_crc++;
      //     ESP_LOGW(TAG, "Frame received, but CRC failed.");
      //   } else {
      //     this->reading_state_.err_invalid_frames++;
      //     ESP_LOGW(TAG, "Frame corrupted.");
      //   }

      //   // if we are here, we have a timeout and no data
      //   // it means we have a failure
      //   // - either no reply from the meter at all
      //   // - or corrupted data and id doesn't trigger stop function
      //   if (this->buffers_.amount_in > 0) {
      //     // most likely its CRC error in STX/SOH/ETX. unclear.
      //     this->stats_.crc_errors_++;
      //     ESP_LOGV(TAG, "RX: %s", format_frame_pretty(this->buffers_.in,
      //     this->buffers_.amount_in).c_str()); ESP_LOGVV(TAG, "RX: %s",
      //     format_hex_pretty(this->buffers_.in,
      //     this->buffers_.amount_in).c_str());
      //   }
      //   this->clear_rx_buffers_();

      //   if (reading_state_.mission_critical) {
      //     this->stats_.crc_errors_ += reading_state_.err_crc;
      //     this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
      //     this->abort_mission_();
      //     return;
      //   }

      //   if (reading_state_.tries_counter < reading_state_.tries_max) {
      //     reading_state_.tries_counter++;
      //     ESP_LOGW(TAG, "Retrying [%d/%d]...", reading_state_.tries_counter,
      //     reading_state_.tries_max); this->send_frame_prepared_();
      //     this->update_last_rx_time_();
      //     return;
      //   }
      //   received_frame_size_ = 0;
      //   // failure, advancing to next state with no data received (frame_size =
      //   0) this->stats_.crc_errors_ += reading_state_.err_crc;
      //   this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
      //   this->set_next_state_(reading_state_.next_state);
      // } break;

    case State::OPEN_SESSION: {
      this->stats_.connections_tried_++;
      session_started_ms = millis();
      this->log_state_();

      this->clear_rx_buffers_();
      request_iter = this->sensors_.begin();

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

    case State::BUFFERS_REQ: {
      this->log_state_();
      this->prepare_and_send_dlms_buffers();
      // this->start_comms_and_next(&buffers_rr_, State::BUFFERS_RCV);

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

      // case State::ACK_START_GET_INFO:
      //   this->log_state_();

      //   if (received_frame_size_ == 0) {
      //     ESP_LOGE(TAG, "No response from meter.");
      //     this->stats_.invalid_frames_++;
      //     this->abort_mission_();
      //     return;
      //   }

      //   ESP_LOGD(TAG, "Meter address: %s", vals[0]);
      //   this->set_next_state_(State::DATA_ENQ);
      //   break;

    case State::DATA_ENQ_UNIT: {
      this->log_state_();
      if (request_iter == this->sensors_.end()) {
        ESP_LOGD(TAG, "All requests done");
        this->set_next_state_(State::SESSION_RELEASE);
        break;
      } else {
        auto req = request_iter->first;
        auto sens = request_iter->second;
        auto type = sens->get_type() == SensorType::TEXT_SENSOR ? DLMS_OBJECT_TYPE_DATA : DLMS_OBJECT_TYPE_REGISTER;

        ESP_LOGD(TAG, "OBIS code: %s", req.c_str());

        if (type == DLMS_OBJECT_TYPE_REGISTER) {
          this->prepare_and_send_dlms_data_unit_request(req.c_str(), type);

        } else {
          // no need for units
          this->set_next_state_(State::DATA_ENQ);
        }
        // this->prepare_and_send_dlms_request(req.c_str(), type, State::DATA_RECV);
      }
    } break;

    case State::DATA_ENQ: {
      this->log_state_();
      auto req = request_iter->first;
      auto sens = request_iter->second;
      if (sens->get_type() == SensorType::SENSOR) {
        auto scal = this->buffers_.gx_register.scaler;
        auto unit = this->buffers_.gx_register.unit;
        // ESP_LOGD(TAG, "scaler: %d, unit: %d", scal, unit);
        ESP_LOGD(TAG, "OBIS code: %s, scaler: %d, unit: %d", req.c_str(), scal, unit);
        const char *unit_str = obj_getUnitAsString(unit);
        if (unit_str != NULL) {
          ESP_LOGD(TAG, "Unit: %s", unit_str);
        } else {
          ESP_LOGD(TAG, "Unit: unknown");
        }
      }
      auto type = sens->get_type() == SensorType::TEXT_SENSOR ? DLMS_OBJECT_TYPE_DATA : DLMS_OBJECT_TYPE_REGISTER;
      // this->prepare_and_send_dlms_request(req.c_str(), type, State::DATA_RECV);
      this->prepare_and_send_dlms_data_request(req.c_str(), type);
    } break;

    case State::DATA_RECV: {
      this->log_state_();
      this->set_next_state_(State::DATA_NEXT);

      auto req = request_iter->first;
      auto sensor = request_iter->second;

      auto ret = this->set_sensor_value(sensor, req.c_str());

      // } else {
      //   ESP_LOGD(TAG, "OBIS code: %s, no result", req.c_str());
      // }

    } break;

    case State::DATA_NEXT:
      this->log_state_();
      request_iter = this->sensors_.upper_bound(request_iter->first);
      if (request_iter != this->sensors_.end()) {
        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::DATA_ENQ_UNIT);
      } else {
        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::SESSION_RELEASE);
      }
      break;

    case State::SESSION_RELEASE:
      sensor_iter = this->sensors_.begin();
      this->log_state_();
      // if (this->auth_required_) {
      //   ESP_LOGD(TAG, "Session release");
      //   this->start_comms_and_next(&session_release_rr_, State::DISCONNECT_REQ);
      // } else {
      // this->set_next_state_(State::DISCONNECT_REQ);
      this->prepare_and_send_dlms_release();
      // }

      break;

    case State::DISCONNECT_REQ:
      this->log_state_();
      ESP_LOGD(TAG, "Disconnect request");
      // this->start_comms_and_next(&disconnect_rr_, State::PUBLISH);
      // this->set_next_state_(State::PUBLISH);
      this->prepare_and_send_dlms_disconnect();
      break;

    case State::PUBLISH:
      this->log_state_();
      ESP_LOGD(TAG, "Publishing data");
      this->update_last_rx_time_();

      if (sensor_iter != this->sensors_.end()) {
        sensor_iter->second->publish();
        sensor_iter++;
      } else {
        this->stats_.dump();
        if (this->crc_errors_per_session_sensor_ != nullptr) {
          this->crc_errors_per_session_sensor_->publish_state(this->stats_.crc_errors_per_session());
        }
        this->report_failure(false);
        this->set_next_state_(State::IDLE);
        ESP_LOGD(TAG, "Total time: %u ms", millis() - session_started_ms);
      }
      break;

    default:
      break;
  }
}

void DlmsCosemComponent::update() {
  if (this->state_ != State::IDLE) {
    ESP_LOGD(TAG, "Starting data collection impossible - component not ready");
    return;
  }
  ESP_LOGD(TAG, "Starting data collection");
  this->set_next_state_(State::OPEN_SESSION);
}

bool char2float(const char *str, float &value) {
  char *end;
  value = strtof(str, &end);
  return *end == '\0';
}

// bool DlmsCosemComponent::set_sensor_value_(DlmsCosemSensorBase *sensor){ //}, ValueRefsArray &vals) {
//   auto type = sensor->get_type();
//   bool ret = true;

//   // uint8_t idx = sensor->get_index() - 1;
//   // if (idx >= VAL_NUM) {
//   //   ESP_LOGE(TAG, "Invalid sensor index %u", idx);
//   //   return false;
//   // }
//   char str_buffer[128] = {'\0'};
//   // strncpy(str_buffer, vals[idx], 128);

//   char *str = str_buffer;
//   // uint8_t sub_idx = sensor->get_sub_index();
//   // if (sub_idx == 0) {
//   //   ESP_LOGD(TAG, "Setting value for sensor '%s', idx = %d to '%s'",
//   //   sensor->get_request().c_str(), idx + 1, str);
//   // } else {
//   //   ESP_LOGD(TAG, "Extracting value for sensor '%s', idx = %d, sub_idx = %d
//   //   from '%s'", sensor->get_request().c_str(),
//   //            idx + 1, sub_idx, str);
//   //   str = this->get_nth_value_from_csv_(str, sub_idx);
//   //   if (str == nullptr) {
//   //     ESP_LOGE(TAG,
//   //              "Cannot extract sensor value by sub-index %d. Is data
//   //              comma-separated? Also note that sub-index starts " "from 1",
//   //              sub_idx);
//   //     str_buffer[0] = '\0';
//   //     str = str_buffer;
//   //   }
//   //   ESP_LOGD(TAG, "Setting value using sub-index = %d, extracted sensor value
//   //   is '%s'", sub_idx, str);
//   // }

//   if (type == SensorType::SENSOR) {
//     float f = 0;
//     ret = str && str[0] && char2float(str, f);
//     if (ret) {
//       static_cast<DlmsCosemSensor *>(sensor)->set_value(f);
//     } else {
//       ESP_LOGE(TAG,
//                "Cannot convert incoming data to a number. Consider using a "
//                "text sensor. Invalid data: '%s'",
//                str);
//     }
//   } else {
// #ifdef USE_TEXT_SENSOR
//     static_cast<DlmsCosemTextSensor *>(sensor)->set_value(str);
// #endif
//   }
//   return ret;
// }

// uint16_t DlmsCosemComponent::calculate_crc_hdlc_frame_(uint8_t *data, size_t
// length, bool set_crc) {
//   // <HDLC frame type 3> := <FLAG> <FORMAT-16> <DA> <SA> <CONTROL> <HCS>
//   <INFORMATION> <FCS-CRC-16> <FLAG> if (length < 6) {
//     return 0;
//   }
//   uint16_t crc = crc16_hdlc(data, 1, length - 4);
//   if (set_crc) {
//     data[length - 3] = crc >> 8;
//     data[length - 2] = crc & 0xFF;
//   }
//   return crc;
// }

// bool DlmsCosemComponent::check_crc_hdlc_frame_(uint8_t *data, size_t length)
// {
//   if (length < 6) {
//     return false;
//   }
//   uint16_t crc = this->calculate_crc_hdlc_frame_(data, length);
//   return crc == (data[length - 3] << 8 | data[length - 2]);
// }

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

void DlmsCosemComponent::InOutBuffers::init() {
  BYTE_BUFFER_INIT(&in);
  bb_capacity(&in, DEFAULT_IN_BUF_SIZE);
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
    ESP_LOGW(TAG, "Growing input buffer from %d to %d", in.capacity, in.size + more_data + GROW_EPSILON);
    bb_capacity(&in, in.size + more_data + GROW_EPSILON);
  }
}

void DlmsCosemComponent::start_comms_and_next(RequestResponse *rr, State next_state, bool mission_critical) {
  // ESP_LOGVV(TAG, "start_comms_and_next: %s", state_to_string(next_state));
  reading_state_.next_state = next_state;

  current_rr_ = rr;

  buffers_.reset();

  // fill in output message array
  current_rr_->start(&buffers_.out_msg);

  for (int i = 0; i < buffers_.out_msg.size; ++i) {
    ESP_LOGVV(TAG, "start_comms_and_next: message %d size %d", i, buffers_.out_msg.data[i]->size);
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

void DlmsCosemComponent::prepare_and_send_dlms_buffers() {
  auto make = [this]() { return cl_snrmRequest(&this->dlms_settings_, &this->buffers_.out_msg); };
  auto parse = [this]() { return cl_parseUAResponse(&this->dlms_settings_, &this->buffers_.reply.data); };
  this->send_dlms_req_and_next(make, parse, State::BUFFERS_RCV);
}

void DlmsCosemComponent::prepare_and_send_dlms_aarq() {
  auto make = [this]() { return cl_aarqRequest(&this->dlms_settings_, &this->buffers_.out_msg); };
  auto parse = [this]() { return cl_parseAAREResponse(&this->dlms_settings_, &this->buffers_.reply.data); };
  this->send_dlms_req_and_next(make, parse, State::ASSOCIATION_RCV);
}

void DlmsCosemComponent::prepare_and_send_dlms_data_unit_request(const char *obis, DLMS_OBJECT_TYPE type) {
  auto ret = cosem_init(BASE(this->buffers_.gx_register), type, obis);
  if (ret != DLMS_ERROR_CODE_OK) {
    ESP_LOGE(TAG, "cosem_init error %d '%s'", ret, this->dlms_error_to_string(ret));
    this->set_next_state_(State::DATA_ENQ);
    return;
  }

  auto make = [this]() {
    return cl_read(&this->dlms_settings_, BASE(this->buffers_.gx_register), 3, &this->buffers_.out_msg);
  };
  auto parse = [this]() {
    return cl_updateValue(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
                          &this->buffers_.reply.dataValue);
  };
  this->send_dlms_req_and_next(make, parse, State::DATA_ENQ, false, false);
}

void DlmsCosemComponent::prepare_and_send_dlms_data_request(const char *obis, DLMS_OBJECT_TYPE type) {
  auto ret = cosem_init(BASE(this->buffers_.gx_register), type, obis);
  if (ret != DLMS_ERROR_CODE_OK) {
    ESP_LOGE(TAG, "cosem_init error %d '%s'", ret, this->dlms_error_to_string(ret));
    this->set_next_state_(State::DATA_NEXT);
    return;
  }

  auto make = [this]() {
    return cl_read(&this->dlms_settings_, BASE(this->buffers_.gx_register), 2, &this->buffers_.out_msg);
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

int DlmsCosemComponent::set_sensor_value(DlmsCosemSensorBase *sensor, const char *obis) {
  ESP_LOGD(TAG, "set_sensor_value %p %p", sensor, obis);
  if (buffers_.reply.complete) {
    auto vt = buffers_.reply.dataType;
    ESP_LOGD(TAG, "OBIS code: %s, DLMS_DATA_TYPE: %s (%d)", obis, this->dlms_data_type_to_string(vt), vt);
  }

  //      if (cosem_rr_.result().has_value()) {
  if (this->dlms_reading_state_.last_error == DLMS_ERROR_CODE_OK) {
    // result is okay, value shall be there
    auto vt = buffers_.reply.dataType;
    auto var = &this->buffers_.gx_register.value;
    auto scal = this->buffers_.gx_register.scaler;
    auto unit = this->buffers_.gx_register.unit;

    ESP_LOGD(TAG, "scaler: %d, unit: %d", scal, unit);
    const char *unit_str = obj_getUnitAsString(unit);
    if (unit_str != NULL) {
      ESP_LOGD(TAG, "Unit: %s", unit_str);
    } else {
      ESP_LOGD(TAG, "Unit: unknown");
    }

    if (sensor->get_type() == SensorType::SENSOR) {
      //
      if (vt == DLMS_DATA_TYPE_FLOAT32 || vt == DLMS_DATA_TYPE_FLOAT64) {
        float val = var_toDouble(var);
        ESP_LOGD(TAG, "OBIS code: %s, Value: %f", obis, val);
        static_cast<DlmsCosemSensor *>(sensor)->set_value(val);
      } else {
        int val = var_toInteger(var);
        ESP_LOGD(TAG, "OBIS code: %s, Value: %d", obis, val);
        static_cast<DlmsCosemSensor *>(sensor)->set_value(val);
      }
    }

#ifdef USE_TEXT_SENSOR
    if (sensor->get_type() == SensorType::TEXT_SENSOR) {
      auto var = &this->buffers_.gx_register.value;
      if (var && var->byteArr && var->byteArr->size > 0) {
        // auto arr = cosem_rr_.data_.value.byteArr;
        auto arr = var->byteArr;

        ESP_LOGV(TAG, "data size=%d", arr->size);
        bb_setInt8(arr, 0);     // add null-termination
        if (arr->size > 128) {  // clip the string
          ESP_LOGW(TAG, "String is too long, clipping to 128 bytes");
          arr->data[127] = '\0';
        }
        static_cast<DlmsCosemTextSensor *>(sensor)->set_value(reinterpret_cast<const char *>(arr->data));
      }
    }
#endif
  } else {
    ESP_LOGD(TAG, "OBIS code: %s, result != DLMS_ERROR_CODE_OK = %d", obis, this->dlms_reading_state_.last_error);
  }
  return this->dlms_reading_state_.last_error;
}

// void DlmsCosemComponent::read_reply_and_go_next_state_(ReadFunction read_fn,
// State next_state, uint8_t retries,
//                                                        bool mission_critical,
//                                                        bool check_crc) {
//   reading_state_ = {};
//   reading_state_.read_fn = read_fn;
//   reading_state_.mission_critical = mission_critical;
//   reading_state_.tries_max = retries;
//   reading_state_.tries_counter = 0;
//   reading_state_.check_crc = check_crc;
//   reading_state_.next_state = next_state;
//   received_frame_size_ = 0;

//   set_next_state_(State::WAITING_FOR_RESPONSE);
// }

// void DlmsCosemComponent::prepare_prog_frame_(const char *request) {
//   // we assume request has format "XXXX(params)"
//   // we assume it always has brackets
//   this->buffers_.amount_out =
//       snprintf((char *) this->buffers_.out, MAX_OUT_BUF_SIZE,
//       "%cR1%c%s%c\xFF", SOH, STX, request, ETX);
//   this->calculate_crc_prog_frame_(this->buffers_.out,
//   this->buffers_.amount_out, true);
// }

// void DlmsCosemComponent::prepare_non_session_prog_frame_(const char *request)
// {
//   // we assume request has format "XXXX(params)"
//   // we assume it always has brackets

//   // "/?!<SOH>R1<STX>NAME()<ETX><BCC>" broadcast
//   // "/?<address>!<SOH>R1<STX>NAME()<ETX><BCC>" direct

//   this->buffers_.amount_out = snprintf((char *) this->buffers_.out,
//   MAX_OUT_BUF_SIZE, "/?%s!%cR1%c%s%c\xFF",
//                                        this->meter_address_.c_str(), SOH,
//                                        STX, request, ETX);
//   // find SOH
//   uint8_t *r1_ptr = std::find(this->buffers_.out, this->buffers_.out +
//   this->buffers_.amount_out, SOH); size_t r1_size = r1_ptr -
//   this->buffers_.out; calculate_crc_prog_frame_(r1_ptr,
//   this->buffers_.amount_out - r1_size, true);
// }

// void DlmsCosemComponent::send_frame_prepared_() {
//   if (this->flow_control_pin_ != nullptr)
//     this->flow_control_pin_->digital_write(true);

//   this->write_array(this->buffers_.out, this->buffers_.amount_out);

//   if (this->flow_control_pin_ != nullptr)
//     this->flow_control_pin_->digital_write(false);

//   ESP_LOGV(TAG, "TX: %s", format_frame_pretty(this->buffers_.out,
//   this->buffers_.amount_out).c_str()); ESP_LOGVV(TAG, "TX: %s",
//   format_hex_pretty(this->buffers_.out, this->buffers_.amount_out).c_str());
// }

// void DlmsCosemComponent::prepare_frame_(const uint8_t *data, size_t length) {
//   memcpy(this->buffers_.out, data, length);
//   this->buffers_.amount_out = length;
// }

// void DlmsCosemComponent::send_frame_(const uint8_t *data, size_t length) {
//   this->prepare_frame_(data, length);
//   this->send_frame_prepared_();
// }

void DlmsCosemComponent::send_dlms_messages_() {
  const int MAX_BYTES_IN_ONE_SHOT = 64;
  gxByteBuffer *buffer = buffers_.out_msg.data[buffers_.out_msg_index];

  int bytes_to_send = buffer->size - buffers_.out_msg_data_pos;
  if (bytes_to_send > 0) {
    if (bytes_to_send > MAX_BYTES_IN_ONE_SHOT)
      bytes_to_send = MAX_BYTES_IN_ONE_SHOT;

    if (this->flow_control_pin_ != nullptr)
      this->flow_control_pin_->digital_write(true);

    // ESP_LOGV(TAG, "send_dlms_messages_: write_array index %d pos %d size %d", buffers_.out_msg_index,
    //          buffers_.out_msg_data_pos, bytes_to_send);
    this->write_array(buffer->data + buffers_.out_msg_data_pos, bytes_to_send);

    if (this->flow_control_pin_ != nullptr)
      this->flow_control_pin_->digital_write(false);

    //    ESP_LOGVV(TAG, "TX: %s", format_frame_pretty(buffer->data + buffers_.out_msg_data_pos,
    //    bytes_to_send).c_str());
    ESP_LOGV(TAG, "TX: %s", format_hex_pretty(buffer->data + buffers_.out_msg_data_pos, bytes_to_send).c_str());

    this->update_last_rx_time_();
    buffers_.out_msg_data_pos += bytes_to_send;
  }
  if (buffers_.out_msg_data_pos >= buffer->size) {
    buffers_.out_msg_index++;
  }
  // if (buffers_.out_msg_index >= buffers_.out_msg.size) {
  //   ESP_LOGV(TAG, "communicate: message sent");
  // }
}

size_t DlmsCosemComponent::receive_frame_(FrameStopFunction stop_fn) {
  const uint32_t read_time_limit_ms = 45;
  size_t ret_val;

  auto count_available = this->available();
  if (count_available <= 0)
    return 0;

  uint32_t read_start = millis();
  uint8_t *p;

  ESP_LOGD(TAG, "avail RX: %d", count_available);
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
      //      ESP_LOGVV(TAG, "RX: %s", format_frame_pretty(this->buffers_.in.data, this->buffers_.in.size).c_str());
      ESP_LOGV(TAG, "RX: %s", format_hex_pretty(this->buffers_.in.data, this->buffers_.in.size).c_str());
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
  // ESP_LOGVV(TAG, "Waiting for HDLC frame");
  auto frame_end_check_hdlc = [](uint8_t *b, size_t s) {
    auto ret = s >= 2 && b[0] == HDLC_FLAG && b[s - 1] == HDLC_FLAG;
    if (ret) {
      ESP_LOGVV(TAG, "Frame HDLC Stop");
    }
    return ret;
  };
  return receive_frame_(frame_end_check_hdlc);
}

// size_t DlmsCosemComponent::receive_frame_ascii_() {
//   // "data<CR><LF>"
//   ESP_LOGVV(TAG, "Waiting for ASCII frame");
//   auto frame_end_check_crlf = [](uint8_t *b, size_t s) {
//     auto ret = s >= 2 && b[s - 1] == '\n' && b[s - 2] == '\r';
//     if (ret) {
//       ESP_LOGVV(TAG, "Frame CRLF Stop");
//     }
//     return ret;
//   };
//   return receive_frame_(frame_end_check_crlf);
// }

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
}

const char *DlmsCosemComponent::state_to_string(State state) {
  switch (state) {
    case State::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case State::IDLE:
      return "IDLE";
    case State::WAIT:
      return "WAIT";
    case State::COMMS_TX:
      return "COMMS_TX";
    case State::COMMS_RX:
      return "COMMS_RX";
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
    default:
      return "UNKNOWN";
  }
}

void DlmsCosemComponent::log_state_(State *next_state) {
  static State last_reported_state{State::NOT_INITIALIZED};
  State current_state = this->state_;
  if (current_state != last_reported_state) {
    if (next_state == nullptr) {
      ESP_LOGV(TAG, "State::%s", state_to_string(current_state));
    } else {
      ESP_LOGV(TAG, "State::%s -> %s", state_to_string(current_state), state_to_string(*next_state));
    }
    last_reported_state = current_state;
  }
}

void DlmsCosemComponent::Stats::dump() {
  ESP_LOGD(TAG, "============================================");
  ESP_LOGD(TAG, "Data collection and publishing finished.");
  ESP_LOGD(TAG, "Total number of sessions ............. %u", this->connections_tried_);
  ESP_LOGD(TAG, "Total number of invalid frames ....... %u", this->invalid_frames_);
  ESP_LOGD(TAG, "Total number of CRC errors ........... %u", this->crc_errors_);
  ESP_LOGD(TAG, "Total number of CRC errors recovered . %u", this->crc_errors_recovered_);
  ESP_LOGD(TAG, "CRC errors per session ............... %f", this->crc_errors_per_session());
  ESP_LOGD(TAG, "Number of failures ................... %u", this->failures_);
  ESP_LOGD(TAG, "============================================");
}

const char *DlmsCosemComponent::dlms_error_to_string(int error) {
  switch (error) {
    case DLMS_ERROR_CODE_OK:
      return "DLMS_ERROR_CODE_OK";
    case DLMS_ERROR_CODE_HARDWARE_FAULT:
      return "DLMS_ERROR_CODE_HARDWARE_FAULT";
    case DLMS_ERROR_CODE_READ_WRITE_DENIED:
      return "DLMS_ERROR_CODE_READ_WRITE_DENIED";
    case DLMS_ERROR_CODE_UNDEFINED_OBJECT:
      return "DLMS_ERROR_CODE_UNDEFINED_OBJECT";
    case DLMS_ERROR_CODE_ACCESS_VIOLATED:
      return "DLMS_ERROR_CODE_ACCESS_VIOLATED";
    default:
      return "";
  }
}

// DLMS_DATA_TYPE_NONE = 0,
// DLMS_DATA_TYPE_BOOLEAN = 3,
// DLMS_DATA_TYPE_BIT_STRING = 4,
// DLMS_DATA_TYPE_INT32 = 5,
// DLMS_DATA_TYPE_UINT32 = 6,
// DLMS_DATA_TYPE_OCTET_STRING = 9,
// DLMS_DATA_TYPE_STRING = 10,
// DLMS_DATA_TYPE_BINARY_CODED_DESIMAL = 13,
// DLMS_DATA_TYPE_STRING_UTF8 = 12,
// DLMS_DATA_TYPE_INT8 = 15,
// DLMS_DATA_TYPE_INT16 = 16,
// DLMS_DATA_TYPE_UINT8 = 17,
// DLMS_DATA_TYPE_UINT16 = 18,
// DLMS_DATA_TYPE_INT64 = 20,
// DLMS_DATA_TYPE_UINT64 = 21,
// DLMS_DATA_TYPE_ENUM = 22,
// DLMS_DATA_TYPE_FLOAT32 = 23,
// DLMS_DATA_TYPE_FLOAT64 = 24,
// DLMS_DATA_TYPE_DATETIME = 25,
// DLMS_DATA_TYPE_DATE = 26,
// DLMS_DATA_TYPE_TIME = 27,
// DLMS_DATA_TYPE_ARRAY = 1,
// DLMS_DATA_TYPE_STRUCTURE = 2,
// DLMS_DATA_TYPE_COMPACT_ARRAY = 19,
// DLMS_DATA_TYPE_BYREF = 0x80
const char *DlmsCosemComponent::dlms_data_type_to_string(DLMS_DATA_TYPE vt) {
  switch (vt) {
    case DLMS_DATA_TYPE_NONE:
      return "DLMS_DATA_TYPE_NONE";
    case DLMS_DATA_TYPE_BOOLEAN:
      return "DLMS_DATA_TYPE_BOOLEAN";
    case DLMS_DATA_TYPE_BIT_STRING:
      return "DLMS_DATA_TYPE_BIT_STRING";
    case DLMS_DATA_TYPE_INT32:
      return "DLMS_DATA_TYPE_INT32";
    case DLMS_DATA_TYPE_UINT32:
      return "DLMS_DATA_TYPE_UINT32";
    case DLMS_DATA_TYPE_OCTET_STRING:
      return "DLMS_DATA_TYPE_OCTET_STRING";
    case DLMS_DATA_TYPE_STRING:
      return "DLMS_DATA_TYPE_STRING";
    case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
      return "DLMS_DATA_TYPE_BINARY_CODED_DESIMAL";
    case DLMS_DATA_TYPE_STRING_UTF8:
      return "DLMS_DATA_TYPE_STRING_UTF8";
    case DLMS_DATA_TYPE_INT8:
      return "DLMS_DATA_TYPE_INT8";
    case DLMS_DATA_TYPE_INT16:
      return "DLMS_DATA_TYPE_INT16";
    case DLMS_DATA_TYPE_UINT8:
      return "DLMS_DATA_TYPE_UINT8";
    case DLMS_DATA_TYPE_UINT16:
      return "DLMS_DATA_TYPE_UINT16";
    case DLMS_DATA_TYPE_INT64:
      return "DLMS_DATA_TYPE_INT64";
    case DLMS_DATA_TYPE_UINT64:
      return "DLMS_DATA_TYPE_UINT64";
    case DLMS_DATA_TYPE_ENUM:
      return "DLMS_DATA_TYPE_ENUM";
    case DLMS_DATA_TYPE_FLOAT32:
      return "DLMS_DATA_TYPE_FLOAT32";
    case DLMS_DATA_TYPE_FLOAT64:
      return "DLMS_DATA_TYPE_FLOAT64";
    case DLMS_DATA_TYPE_DATETIME:
      return "DLMS_DATA_TYPE_DATETIME";
    case DLMS_DATA_TYPE_DATE:
      return "DLMS_DATA_TYPE_DATE";
    case DLMS_DATA_TYPE_TIME:
      return "DLMS_DATA_TYPE_TIME";
    case DLMS_DATA_TYPE_ARRAY:
      return "DLMS_DATA_TYPE_ARRAY";
    case DLMS_DATA_TYPE_STRUCTURE:
      return "DLMS_DATA_TYPE_STRUCTURE";
    case DLMS_DATA_TYPE_COMPACT_ARRAY:
      return "DLMS_DATA_TYPE_COMPACT_ARRAY";
    case DLMS_DATA_TYPE_BYREF:
      return "DLMS_DATA_TYPE_BYREF";
    default:
      return "DMS_DATA_TYPE UNKNOWN";
  }
}
}  // namespace dlms_cosem
}  // namespace esphome
