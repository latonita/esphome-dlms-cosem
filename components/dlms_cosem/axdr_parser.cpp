#include "axdr_parser.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>
#include <sstream>
#include <iomanip>

constexpr const char *TAG = "dlms_cosem.axdr";
// const std::string tag_{"AxdrStreamParser"};

const char *dlms_error_to_string(int error) {
  switch (error) {
    case DLMS_ERROR_CODE_OK:
      return "DLMS_ERROR_CODE_OK";
    case DLMS_ERROR_CODE_HARDWARE_FAULT:
      return "DLMS_ERROR_CODE_HARDWARE_FAULT";
    case DLMS_ERROR_CODE_TEMPORARY_FAILURE:
      return "DLMS_ERROR_CODE_TEMPORARY_FAILURE";
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

const char *dlms_data_type_to_string(DLMS_DATA_TYPE vt) {
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

bool hlp_isValueDataType(DLMS_DATA_TYPE type) {
  switch (type) {
    // Complex/Container types - NOT value types
    case DLMS_DATA_TYPE_ARRAY:
    case DLMS_DATA_TYPE_STRUCTURE:
    case DLMS_DATA_TYPE_COMPACT_ARRAY:
      return false;

    // All other types are value types
    case DLMS_DATA_TYPE_NONE:
    case DLMS_DATA_TYPE_BOOLEAN:
    case DLMS_DATA_TYPE_BIT_STRING:
    case DLMS_DATA_TYPE_INT32:
    case DLMS_DATA_TYPE_UINT32:
    case DLMS_DATA_TYPE_OCTET_STRING:
    case DLMS_DATA_TYPE_STRING:
    case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
    case DLMS_DATA_TYPE_STRING_UTF8:
    case DLMS_DATA_TYPE_INT8:
    case DLMS_DATA_TYPE_INT16:
    case DLMS_DATA_TYPE_UINT8:
    case DLMS_DATA_TYPE_UINT16:
    case DLMS_DATA_TYPE_INT64:
    case DLMS_DATA_TYPE_UINT64:
    case DLMS_DATA_TYPE_ENUM:
    case DLMS_DATA_TYPE_FLOAT32:
    case DLMS_DATA_TYPE_FLOAT64:
    case DLMS_DATA_TYPE_DATETIME:
    case DLMS_DATA_TYPE_DATE:
    case DLMS_DATA_TYPE_TIME:
      return true;

    default:

      return false;
  }
}

uint8_t AxdrStreamParser::peek_byte() {
  if (this->buffer->position + 1 > this->buffer->size) {
    // ESP_LOGVV(TAG, "Buffer overflow in peek_byte at position %d", this->buffer->position);
    return 0xFF;
  }
  return this->buffer->data[this->buffer->position];
}

uint8_t AxdrStreamParser::read_byte() {
  if (this->buffer->position + 1 > this->buffer->size) {
    //    ESP_LOGVV(TAG, "Buffer overflow in read_byte at position %d", this->buffer->position);
    return 0xFF;
  }
  return this->buffer->data[this->buffer->position++];
}

uint16_t AxdrStreamParser::read_u16() {
  if (this->buffer->position + 2 > this->buffer->size) {
    //    ESP_LOGVV(TAG, "Buffer overflow in read_u16 at position %d", this->buffer->position);
    return 0xFFFF;
  }
  uint16_t value = (this->buffer->data[this->buffer->position] << 8) | this->buffer->data[this->buffer->position + 1];
  this->buffer->position += 2;
  return value;
}

uint32_t AxdrStreamParser::read_u32() {
  if (this->buffer->position + 4 > this->buffer->size) {
    //    ESP_LOGV(TAG, "Buffer overflow in read_u32 at position %d", this->buffer->position);
    return 0xFFFFFFFF;
  }
  uint32_t value =
      (this->buffer->data[this->buffer->position] << 24) | (this->buffer->data[this->buffer->position + 1] << 16) |
      (this->buffer->data[this->buffer->position + 2] << 8) | this->buffer->data[this->buffer->position + 3];
  this->buffer->position += 4;
  return value;
}

bool AxdrStreamParser::test_if_date_time_12b() {
  if (this->buffer->position + 12 > this->buffer->size) {
    // ESP_LOGV(TAG, "Buffer overflow in date-time at position %d", this->buffer->position);
    return 0;
  }

  const uint8_t *buf = this->buffer->data + this->buffer->position;
  if (!buf)
    return false;

  // Year
  uint16_t year = (buf[0] << 8) | buf[1];
  if (!(year == 0x0000 || (year >= 1970 && year <= 2100)))
    return false;

  // Month
  uint8_t month = buf[2];
  if (!(month == 0xFF || (month >= 1 && month <= 12)))
    return false;

  // Day of month
  uint8_t day = buf[3];
  if (!(day == 0xFF || (day >= 1 && day <= 31)))
    return false;

  // Day of week
  uint8_t dow = buf[4];
  if (!(dow == 0xFF || (dow >= 1 && dow <= 7)))
    return false;

  // Hour
  uint8_t hour = buf[5];
  if (!(hour == 0xFF || hour <= 23))
    return false;

  // Minute
  uint8_t minute = buf[6];
  if (!(minute == 0xFF || minute <= 59))
    return false;

  // Second
  uint8_t second = buf[7];
  if (!(second == 0xFF || second <= 59))
    return false;

  // Hundredths of second
  uint16_t ms = (buf[8] << 8) | buf[9];
  if (!(ms == 0xFFFF || ms <= 99))
    return false;

  // Deviation (timezone offset, signed, 2 bytes)
  uint16_t u_dev = buf[10] | (buf[11] << 8);
  int16_t s_dev = (int16_t) (u_dev);
  if (!(s_dev == (int16_t) 0x8000 || (s_dev >= -720 && s_dev <= 720)))
    return false;

  return true;
}

constexpr uint16_t MAX_CLASS_ID = 0x00FF;
constexpr uint8_t MAX_ATTRIBUTE_ID = 0x20;
constexpr size_t MIN_UNTAGGED_ATTRIBUTE_DESCRIPTOR_SIZE = 9;
constexpr size_t MIN_TAGGED_ATTRIBUTE_DESCRIPTOR_SIZE = 14;

bool AxdrStreamParser::with_position_rollback(std::function<bool(AxdrStreamParser *)> func) {
  uint32_t saved_position = this->buffer->position;
  bool result = func(this);
  if (!result) {
    this->buffer->position = saved_position;
  }
  return result;
}

bool AxdrStreamParser::parse_attribute_value(uint16_t class_id, const uint8_t *obis_code, uint8_t attribute_id) {
  uint8_t value_type = read_byte();

  if (!hlp_isValueDataType((DLMS_DATA_TYPE) value_type)) {
    ESP_LOGV(TAG, "Unexpected non-value type %02X for attribute value at position %d", value_type,
             this->buffer->position - 1);
    return false;
  };

  const uint8_t *value_ptr = nullptr;
  uint8_t value_len = 0;

  uint16_t value_position = this->buffer->position;

  int data_size = hlp_getDataTypeSize((DLMS_DATA_TYPE) value_type);

  if (data_size > 0) {
    // Fixed size data
    if (this->buffer->position + data_size > this->buffer->size) {
      ESP_LOGV(TAG, "Buffer overflow when reading value at position %d", this->buffer->position);
      return false;
    }
    value_ptr = &this->buffer->data[this->buffer->position];
    value_len = (uint8_t) data_size;
    this->buffer->position += data_size;
  } else if (data_size == 0) {
    // Zero-length data (NONE type)
    value_ptr = nullptr;
    value_len = 0;
    value_position--;
  } else {
    // Variable size data
    uint8_t length = read_byte();
    if (length == 0xFF) {
      ESP_LOGV(TAG, "Invalid variable size data length at position %d", this->buffer->position - 1);
      return false;
    }
    if (this->buffer->position + length > this->buffer->size) {
      ESP_LOGV(TAG, "Buffer overflow when reading variable size value at position %d", this->buffer->position);
      return false;
    }
    value_position = this->buffer->position;
    value_ptr = &this->buffer->data[value_position];
    value_len = length;
    this->buffer->position += length;
  }

  this->objects_found++;

  if (this->show_log_) {
    auto val_f = dlms_data_as_float((DLMS_DATA_TYPE) value_type, value_ptr, value_len);
    auto val_s = dlms_data_as_string((DLMS_DATA_TYPE) value_type, value_ptr, value_len);
    ESP_LOGI(TAG, "# %02d  .................................................................................... ",
             this->objects_found);
    ESP_LOGI(TAG, "Found attribute descriptor: class_id=%d, obis=%d.%d.%d.%d.%d.%d, attr_id=%d, value_type=%02X",
             class_id, obis_code[0], obis_code[1], obis_code[2], obis_code[3], obis_code[4], obis_code[5], attribute_id,
             value_type);
    ESP_LOGI(TAG, "Value type: %s at %d, len %d", dlms_data_type_to_string((DLMS_DATA_TYPE) value_type), value_position,
             value_len);
    ESP_LOGI(TAG, " as hex dump : %s", esphome::format_hex_pretty(value_ptr, value_len).c_str());
    ESP_LOGI(TAG, " as string   :'%s'", val_s.c_str());
    ESP_LOGI(TAG, " as number   : %f", val_f);
  }

  // Call the callback with the parsed attribute descriptor
  if (callback_) {
    callback_(class_id, obis_code, attribute_id, (DLMS_DATA_TYPE) value_type, value_ptr, value_len);
  }
  return true;
}

bool AxdrStreamParser::parse_attribute_descriptor_unsafe(bool tagged) {
  size_t min_size = tagged ? MIN_TAGGED_ATTRIBUTE_DESCRIPTOR_SIZE : MIN_UNTAGGED_ATTRIBUTE_DESCRIPTOR_SIZE;
  if (this->buffer->position + min_size > this->buffer->size) {
    return false;
  }

  uint16_t class_id;
  const uint8_t *obis_code;
  uint8_t attribute_id;
  uint8_t type;

  if (tagged) {
    // Tagged version: Parse class_id (UINT16)
    type = read_byte();
    if (type != DLMS_DATA_TYPE_UINT16) {
      return false;
    }
  }

  class_id = read_u16();
  if (class_id == 0 || class_id > MAX_CLASS_ID) {
    return false;
  }

  if (tagged) {
    // Parse OBIS (OCTET_STRING with length 6)
    type = read_byte();
    uint8_t obis_len = read_byte();
    if (type != DLMS_DATA_TYPE_OCTET_STRING || obis_len != 6) {
      return false;
    }
  }

  obis_code = &this->buffer->data[this->buffer->position];
  this->buffer->position += 6;

  if (tagged) {
    // Parse attribute_id (UINT8 or INT8)
    type = read_byte();
    if (type != DLMS_DATA_TYPE_UINT8 && type != DLMS_DATA_TYPE_INT8) {
      return false;
    }
  }

  attribute_id = read_byte();
  if (attribute_id == 0 || attribute_id > MAX_ATTRIBUTE_ID) {
    return false;
  }

  // Parse the value
  return parse_attribute_value(class_id, obis_code, attribute_id);
}

bool AxdrStreamParser::skip_data(uint8_t type) {
  int data_size = hlp_getDataTypeSize((DLMS_DATA_TYPE) type);

  if (data_size > 0) {
    // Fixed size data
    if (this->buffer->position + data_size > this->buffer->size) {
      return false;
    }
    this->buffer->position += data_size;
  } else if (data_size == 0) {
    // Zero-length data (NONE type) - nothing to skip
  } else {
    // Variable size data
    uint8_t length = read_byte();
    if (length == 0xFF) {
      return false;
    }
    if (this->buffer->position + length > this->buffer->size) {
      return false;
    }
    this->buffer->position += length;
  }

  return true;
}

bool AxdrStreamParser::skip_sequence(uint8_t type) {
  uint8_t elements_count = read_byte();
  if (elements_count == 0xFF) {
    ESP_LOGV(TAG, "Invalid sequence length when skipping at position %d", this->buffer->position - 1);
    return false;
  }

  ESP_LOGD(TAG, "Skipping %s with %d elements at position %d",
           (type == DLMS_DATA_TYPE_STRUCTURE) ? "STRUCTURE" : "ARRAY", elements_count, this->buffer->position - 1);

  for (uint8_t i = 0; i < elements_count; i++) {
    uint8_t elem_type = read_byte();
    if (!parse_element(elem_type)) {
      ESP_LOGV(TAG, "Failed to skip element %d of %s at position %d", i + 1,
               (type == DLMS_DATA_TYPE_STRUCTURE) ? "STRUCTURE" : "ARRAY", this->buffer->position - 1);
      return false;
    }
  }

  return true;
}

bool AxdrStreamParser::parse_data(uint8_t type, uint8_t depth) { return skip_data(type); }

bool AxdrStreamParser::parse_sequence(uint8_t type, uint8_t depth) {
  uint8_t elements_count = read_byte();
  if (elements_count == 0xFF) {
    ESP_LOGVV(TAG, "Invalid sequence length at position %d", this->buffer->position - 1);
    return false;
  }

  // Special handling for structures that might contain attribute descriptors
  // There can be both tagged and untagged AttributeDescriptor structures
  // Detect AttributeDescriptor structure (heuristic: 2 elements and next looks like class_id+obis+attr)
  if (depth > 0 && type == DLMS_DATA_TYPE_STRUCTURE && elements_count >= 2) {
    //  uint16_t saved_position = this->buffer->position;
    // start with tagged - it has higher chance to be correct
    if (elements_count >= 4 && with_position_rollback([this](AxdrStreamParser *parser) {
          return parser->parse_attribute_descriptor_unsafe(true);
        })) {
      // there might be complex cases with multiple descriptors in one structure... skip for now
      // we can read from how many elements were parsed, decrease elements_count, start this outer IF all over.
      // but there are many unknowns - we need to see real-world data first. we might need to be able to
      // read couple more attributes in case there is attr2 and attr3 in one structure
      // just return true for now and dump the rest of the structure
      return true;
    } else if (with_position_rollback(
                   [this](AxdrStreamParser *parser) { return parser->parse_attribute_descriptor_unsafe(false); })) {
      return true;
    }
  }

  // Not an attribute descriptor, continue with normal parsing.
  // Position is reset by 'with_position_rollback'

  // Normal structure/array parsing
  for (uint8_t i = 0; i < elements_count; i++) {
    uint8_t elem_type = read_byte();
    if (!parse_element(elem_type, depth + 1)) {
      //   ESP_LOGV(TAG, "Failed to parse element %d of %s at position %d", i + 1,
      //            (type == DLMS_DATA_TYPE_STRUCTURE) ? "STRUCTURE" : "ARRAY", this->buffer->position - 1);
      return false;
    }
  }

  return true;
}

bool AxdrStreamParser::parse_element(uint8_t type, uint8_t depth) {
  if (type == DLMS_DATA_TYPE_STRUCTURE || type == DLMS_DATA_TYPE_ARRAY) {
    return parse_sequence(type, depth);
  } else {
    return parse_data(type, depth);
  }
}

size_t AxdrStreamParser::parse() {
  if (this->buffer == nullptr || this->buffer->size == 0) {
    ESP_LOGV(TAG, "Buffer is null or empty");
    return 0;
  }

  ESP_LOGI(TAG, "Starting fast AXDR parsing of %d bytes", this->buffer->size);

  // Skip to notification flag 0x0F
  while (this->buffer->position < this->buffer->size) {
    uint8_t flag = read_byte();
    if (flag == 0x0F) {
      ESP_LOGI(TAG, "Found notification flag at position %d", this->buffer->position - 1);
      break;
    }
  }

  // Skip 5 bytes (priority and other fields)
  for (int i = 0; i < 5; i++) {
    uint8_t priority = read_byte();
  }

  // Check for datetime object before the data
  bool is_date_time = test_if_date_time_12b();
  if (is_date_time) {
    ESP_LOGI(TAG, "Skipping datetime at position %d", this->buffer->position);
    this->buffer->position += 12;
  }

  // First byte after flag should be the data type
  uint8_t start_type = read_byte();
  if (start_type != (uint8_t) DLMS_DATA_TYPE_STRUCTURE && start_type != (uint8_t) DLMS_DATA_TYPE_ARRAY) {
    ESP_LOGV(TAG, "Expected structure or array after notification flag, found type %02X at position %d", start_type,
             this->buffer->position);
    return 0;
  }

  // Parse the data, looking for attribute descriptors
  bool success = parse_element(start_type, 0);
  if (!success) {
    ESP_LOGV(TAG, "Some errors occurred parsing AXDR data");
  }

  ESP_LOGI(TAG, "Fast parsing completed, processed %d bytes", this->buffer->position);
  return this->objects_found;
}

float dlms_data_as_float(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length) {
  if (value_buffer_ptr == nullptr || value_length == 0)
    return 0.0f;

  auto be16 = [](const uint8_t *p) -> uint16_t { return (uint16_t) ((p[0] << 8) | p[1]); };
  auto be32 = [](const uint8_t *p) -> uint32_t {
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) | (uint32_t) p[3];
  };
  auto be64 = [](const uint8_t *p) -> uint64_t {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
      v = (v << 8) | p[i];
    return v;
  };

  switch (value_type) {
    case DLMS_DATA_TYPE_BOOLEAN:
    case DLMS_DATA_TYPE_ENUM:
    case DLMS_DATA_TYPE_UINT8:
      return static_cast<float>(value_buffer_ptr[0]);
    case DLMS_DATA_TYPE_INT8:
      return static_cast<float>(static_cast<int8_t>(value_buffer_ptr[0]));
    case DLMS_DATA_TYPE_UINT16:
      if (value_length >= 2)
        return static_cast<float>(be16(value_buffer_ptr));
      return 0.0f;
    case DLMS_DATA_TYPE_INT16:
      if (value_length >= 2) {
        int16_t v = static_cast<int16_t>(be16(value_buffer_ptr));
        return static_cast<float>(v);
      }
      return 0.0f;
    case DLMS_DATA_TYPE_UINT32:
      if (value_length >= 4)
        return static_cast<float>(be32(value_buffer_ptr));
      return 0.0f;
    case DLMS_DATA_TYPE_INT32:
      if (value_length >= 4) {
        int32_t v = static_cast<int32_t>(be32(value_buffer_ptr));
        return static_cast<float>(v);
      }
      return 0.0f;
    case DLMS_DATA_TYPE_UINT64:
      if (value_length >= 8)
        return static_cast<float>(be64(value_buffer_ptr));
      return 0.0f;
    case DLMS_DATA_TYPE_INT64:
      if (value_length >= 8) {
        uint64_t u = be64(value_buffer_ptr);
        int64_t v = static_cast<int64_t>(u);
        return static_cast<float>(v);
      }
      return 0.0f;
    case DLMS_DATA_TYPE_FLOAT32:
      if (value_length >= 4) {
        uint32_t u = be32(value_buffer_ptr);
        float f{};
        std::memcpy(&f, &u, sizeof(f));
        return f;
      }
      return 0.0f;
    case DLMS_DATA_TYPE_FLOAT64:
      if (value_length >= 8) {
        uint8_t b[8];
        for (int i = 0; i < 8; i++)
          b[i] = value_buffer_ptr[i];

        double d{};
        std::memcpy(&d, b, sizeof(d));
        return static_cast<float>(d);
      }
      return 0.0f;
    default:
      return 0.0f;
  }
}

std::string dlms_data_as_string(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length) {
  if (value_buffer_ptr == nullptr && value_length == 0)
    return std::string();

  auto hex_of = [](const uint8_t *p, uint8_t len) -> std::string {
    std::ostringstream ss;
    ss << std::hex << std::setfill('0');
    for (uint8_t i = 0; i < len; i++) {
      ss << std::setw(2) << static_cast<int>(p[i]);
      if (i + 1 < len)
        ss << "";  // compact
    }
    return ss.str();
  };

  switch (value_type) {
    case DLMS_DATA_TYPE_OCTET_STRING:
    case DLMS_DATA_TYPE_STRING:
    case DLMS_DATA_TYPE_STRING_UTF8: {
      return std::string(reinterpret_cast<const char *>(value_buffer_ptr),
                         reinterpret_cast<const char *>(value_buffer_ptr) + value_length);
    }
    case DLMS_DATA_TYPE_BIT_STRING:
    case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
    case DLMS_DATA_TYPE_BYREF:
      return hex_of(value_buffer_ptr, value_length);

    case DLMS_DATA_TYPE_BOOLEAN:
    case DLMS_DATA_TYPE_ENUM:
    case DLMS_DATA_TYPE_UINT8: {
      return std::to_string(static_cast<unsigned>(value_buffer_ptr ? value_buffer_ptr[0] : 0));
    }
    case DLMS_DATA_TYPE_INT8: {
      return std::to_string(static_cast<int>(static_cast<int8_t>(value_buffer_ptr ? value_buffer_ptr[0] : 0)));
    }
    case DLMS_DATA_TYPE_UINT16: {
      if (value_length >= 2) {
        uint16_t v = (uint16_t) ((value_buffer_ptr[0] << 8) | value_buffer_ptr[1]);
        return std::to_string(v);
      }
      return std::string();
    }
    case DLMS_DATA_TYPE_INT16: {
      if (value_length >= 2) {
        int16_t v = (int16_t) ((value_buffer_ptr[0] << 8) | value_buffer_ptr[1]);
        return std::to_string(v);
      }
      return std::string();
    }
    case DLMS_DATA_TYPE_UINT32: {
      if (value_length >= 4) {
        uint32_t v = ((uint32_t) value_buffer_ptr[0] << 24) | ((uint32_t) value_buffer_ptr[1] << 16) |
                     ((uint32_t) value_buffer_ptr[2] << 8) | (uint32_t) value_buffer_ptr[3];
        return std::to_string(v);
      }
      return std::string();
    }
    case DLMS_DATA_TYPE_INT32: {
      if (value_length >= 4) {
        int32_t v = ((int32_t) value_buffer_ptr[0] << 24) | ((int32_t) value_buffer_ptr[1] << 16) |
                    ((int32_t) value_buffer_ptr[2] << 8) | (int32_t) value_buffer_ptr[3];
        return std::to_string(v);
      }
      return std::string();
    }
    case DLMS_DATA_TYPE_UINT64: {
      if (value_length >= 8) {
        uint64_t v = 0;
        for (int i = 0; i < 8; i++)
          v = (v << 8) | value_buffer_ptr[i];
        return std::to_string(v);
      }
      return std::string();
    }
    case DLMS_DATA_TYPE_INT64: {
      if (value_length >= 8) {
        uint64_t u = 0;
        for (int i = 0; i < 8; i++)
          u = (u << 8) | value_buffer_ptr[i];
        int64_t v = static_cast<int64_t>(u);
        return std::to_string(v);
      }
      return std::string();
    }
    case DLMS_DATA_TYPE_FLOAT32:
    case DLMS_DATA_TYPE_FLOAT64: {
      float f = dlms_data_as_float(value_type, value_buffer_ptr, value_length);
      // Use minimal formatting
      std::ostringstream ss;
      ss << f;
      return ss.str();
    }
    case DLMS_DATA_TYPE_DATETIME:
    case DLMS_DATA_TYPE_DATE:
    case DLMS_DATA_TYPE_TIME:
      // For now, return hex. Higher-level layers may decode properly.
      return hex_of(value_buffer_ptr, value_length);

    case DLMS_DATA_TYPE_NONE:
    default:
      return std::string();
  }
}