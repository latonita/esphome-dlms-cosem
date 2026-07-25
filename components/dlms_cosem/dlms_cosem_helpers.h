#pragma once

#include <string>
#include <dlmssettings.h>

#include "esphome/core/log.h"

namespace esphome::dlms_cosem {

float dlms_data_as_float(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length);
std::string dlms_datetime_as_string(const uint8_t *value_buffer_ptr, uint8_t value_length);
std::string dlms_data_as_string(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length);

const LogString *dlms_data_type_to_string(DLMS_DATA_TYPE vt);
const LogString *dlms_error_to_string(int error);

}  // namespace esphome::dlms_cosem
