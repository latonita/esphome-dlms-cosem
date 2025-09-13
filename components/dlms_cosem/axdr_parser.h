#pragma once

#include <cstdint>
#include <string>
#include <functional>

#include <client.h>
#include <converters.h>
#include <cosem.h>
#include <dlmssettings.h>

const char *dlms_data_type_to_string(DLMS_DATA_TYPE vt);
const char *dlms_error_to_string(int error);
float dlms_data_as_float(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length);
std::string dlms_data_as_string(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length);

bool hlp_isValueDataType(DLMS_DATA_TYPE type);

// Callback function for attribute descriptor found
// Parameters: class_id, obis_code, attribute_id, value_type, value_buffer_ptr, value_length
using AttributeDescriptorCallback =
    std::function<void(uint16_t, const uint8_t *, uint8_t, DLMS_DATA_TYPE, const uint8_t *, uint8_t)>;

class AxdrStreamParser {
  gxByteBuffer *buffer;

  AttributeDescriptorCallback callback_;
  size_t objects_found = 0;
  bool show_log_ = false;

  uint8_t peek_byte();
  uint8_t read_byte();
  uint16_t read_u16();
  uint32_t read_u32();

  bool test_if_date_time_12b();

  bool parse_element(uint8_t type, uint8_t depth = 0);
  bool parse_sequence(uint8_t type, uint8_t depth = 0);
  bool parse_data(uint8_t type, uint8_t depth = 0);

  bool parse_attribute_descriptor_unsafe(bool tagged);
  bool parse_attribute_value(uint16_t class_id, const uint8_t *obis_code, uint8_t attribute_id);

  bool with_position_rollback(std::function<bool(AxdrStreamParser *)> func);

  // Skip over data without parsing
  bool skip_data(uint8_t type);
  bool skip_sequence(uint8_t type);

 public:
  AxdrStreamParser(gxByteBuffer *buf, AttributeDescriptorCallback callback, bool show_log) : buffer(buf), callback_(callback), show_log_(show_log) {}
  size_t parse();
};
