#include "dlms_cosem_helpers.h"
#include <cstring>
#include <cstdio>

namespace esphome::dlms_cosem {

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

std::string dlms_datetime_as_string(const uint8_t *value_buffer_ptr, uint8_t value_length) {
  if (value_buffer_ptr == nullptr || value_length < 12) {
    return std::string();
  }

  const uint8_t *buf = value_buffer_ptr;

  // Year (2 bytes, big-endian)
  uint16_t year = (buf[0] << 8) | buf[1];

  // Month (1 byte)
  uint8_t month = buf[2];

  // Day of month (1 byte)
  uint8_t day = buf[3];

  // Day of week (1 byte) - skip for formatting
  // uint8_t dow = buf[4];

  // Hour (1 byte)
  uint8_t hour = buf[5];

  // Minute (1 byte)
  uint8_t minute = buf[6];

  // Second (1 byte)
  uint8_t second = buf[7];

  // Hundredths of second (1 byte)
  uint8_t hundredths = buf[8];

  // Timezone deviation (2 bytes, signed, big-endian)
  uint16_t u_dev = (buf[9] << 8) | buf[10];
  int16_t deviation = (int16_t) u_dev;

  // Clock status (1 byte)
  // uint8_t clock_status = buf[11];

  // Format: YYYY-MM-DD HH:MM:SS[.hh][ +HH:MM] -> max ~29 chars, 40 is safe.
  char out[40];
  size_t pos = 0;
  auto adv = [&](int n) {
    if (n > 0 && pos + static_cast<size_t>(n) < sizeof(out))
      pos += static_cast<size_t>(n);
  };

  if (year != 0x0000 && year != 0xFFFF)
    adv(snprintf(out + pos, sizeof(out) - pos, "%04u", year));
  else
    adv(snprintf(out + pos, sizeof(out) - pos, "????"));

  if (month != 0xFF && month >= 1 && month <= 12)
    adv(snprintf(out + pos, sizeof(out) - pos, "-%02u", month));
  else
    adv(snprintf(out + pos, sizeof(out) - pos, "-??"));

  if (day != 0xFF && day >= 1 && day <= 31)
    adv(snprintf(out + pos, sizeof(out) - pos, "-%02u", day));
  else
    adv(snprintf(out + pos, sizeof(out) - pos, "-??"));

  if (hour != 0xFF && hour <= 23)
    adv(snprintf(out + pos, sizeof(out) - pos, " %02u", hour));
  else
    adv(snprintf(out + pos, sizeof(out) - pos, " ??"));

  if (minute != 0xFF && minute <= 59)
    adv(snprintf(out + pos, sizeof(out) - pos, ":%02u", minute));
  else
    adv(snprintf(out + pos, sizeof(out) - pos, ":??"));

  if (second != 0xFF && second <= 59)
    adv(snprintf(out + pos, sizeof(out) - pos, ":%02u", second));
  else
    adv(snprintf(out + pos, sizeof(out) - pos, ":??"));

  // Add hundredths if available
  if (hundredths != 0xFF && hundredths <= 99)
    adv(snprintf(out + pos, sizeof(out) - pos, ".%02u", hundredths));

  // Add timezone info if available
  if (deviation != (int16_t) 0x8000) {
    int abs_dev = deviation >= 0 ? deviation : -deviation;
    adv(snprintf(out + pos, sizeof(out) - pos, " %c%02d:%02d", deviation >= 0 ? '+' : '-', abs_dev / 60, abs_dev % 60));
  }

  return std::string(out, pos);
}

std::string dlms_data_as_string(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length) {
  if (value_buffer_ptr == nullptr && value_length == 0)
    return std::string();

  auto hex_of = [](const uint8_t *p, uint8_t len) -> std::string {
    char out[2 * 255 + 1];  // len is uint8_t -> at most 255 bytes -> 510 hex chars + NUL
    size_t pos = 0;
    for (uint8_t i = 0; i < len; i++) {
      int n = snprintf(out + pos, sizeof(out) - pos, "%02x", p[i]);
      if (n > 0)
        pos += static_cast<size_t>(n);
    }
    return std::string(out, pos);
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
      // Use minimal formatting (%g matches the old shortest-form output)
      char out[32];
      snprintf(out, sizeof(out), "%g", f);
      return std::string(out);
    }
    case DLMS_DATA_TYPE_DATETIME:
      return dlms_datetime_as_string(value_buffer_ptr, value_length);
    case DLMS_DATA_TYPE_DATE:
    case DLMS_DATA_TYPE_TIME:
      // For now, return hex. Higher-level layers may decode properly.
      return hex_of(value_buffer_ptr, value_length);

    case DLMS_DATA_TYPE_NONE:
    default:
      return std::string();
  }
}

const LogString *dlms_error_to_string(int error) {
  switch (error) {
    case DLMS_ERROR_CODE_OK:
      return LOG_STR("DLMS_ERROR_CODE_OK");
    case DLMS_ERROR_CODE_HARDWARE_FAULT:
      return LOG_STR("DLMS_ERROR_CODE_HARDWARE_FAULT");
    case DLMS_ERROR_CODE_TEMPORARY_FAILURE:
      return LOG_STR("DLMS_ERROR_CODE_TEMPORARY_FAILURE");
    case DLMS_ERROR_CODE_READ_WRITE_DENIED:
      return LOG_STR("DLMS_ERROR_CODE_READ_WRITE_DENIED");
    case DLMS_ERROR_CODE_UNDEFINED_OBJECT:
      return LOG_STR("DLMS_ERROR_CODE_UNDEFINED_OBJECT");
    case DLMS_ERROR_CODE_ACCESS_VIOLATED:
      return LOG_STR("DLMS_ERROR_CODE_ACCESS_VIOLATED");
    default:
      return LOG_STR("");
  }
}

const LogString *dlms_data_type_to_string(DLMS_DATA_TYPE vt) {
  switch (vt) {
    case DLMS_DATA_TYPE_NONE:
      return LOG_STR("NONE");
    case DLMS_DATA_TYPE_BOOLEAN:
      return LOG_STR("BOOLEAN");
    case DLMS_DATA_TYPE_BIT_STRING:
      return LOG_STR("BIT_STRING");
    case DLMS_DATA_TYPE_INT32:
      return LOG_STR("INT32");
    case DLMS_DATA_TYPE_UINT32:
      return LOG_STR("UINT32");
    case DLMS_DATA_TYPE_OCTET_STRING:
      return LOG_STR("OCTET_STRING");
    case DLMS_DATA_TYPE_STRING:
      return LOG_STR("STRING");
    case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
      return LOG_STR("BINARY_CODED_DESIMAL");
    case DLMS_DATA_TYPE_STRING_UTF8:
      return LOG_STR("STRING_UTF8");
    case DLMS_DATA_TYPE_INT8:
      return LOG_STR("INT8");
    case DLMS_DATA_TYPE_INT16:
      return LOG_STR("INT16");
    case DLMS_DATA_TYPE_UINT8:
      return LOG_STR("UINT8");
    case DLMS_DATA_TYPE_UINT16:
      return LOG_STR("UINT16");
    case DLMS_DATA_TYPE_INT64:
      return LOG_STR("INT64");
    case DLMS_DATA_TYPE_UINT64:
      return LOG_STR("UINT64");
    case DLMS_DATA_TYPE_ENUM:
      return LOG_STR("ENUM");
    case DLMS_DATA_TYPE_FLOAT32:
      return LOG_STR("FLOAT32");
    case DLMS_DATA_TYPE_FLOAT64:
      return LOG_STR("FLOAT64");
    case DLMS_DATA_TYPE_DATETIME:
      return LOG_STR("DATETIME");
    case DLMS_DATA_TYPE_DATE:
      return LOG_STR("DATE");
    case DLMS_DATA_TYPE_TIME:
      return LOG_STR("TIME");
    case DLMS_DATA_TYPE_ARRAY:
      return LOG_STR("ARRAY");
    case DLMS_DATA_TYPE_STRUCTURE:
      return LOG_STR("STRUCTURE");
    case DLMS_DATA_TYPE_COMPACT_ARRAY:
      return LOG_STR("COMPACT_ARRAY");
    case DLMS_DATA_TYPE_BYREF:
      return LOG_STR("BYREF");
    default:
      return LOG_STR("UNKNOWN");
  }
}

}  // namespace esphome::dlms_cosem