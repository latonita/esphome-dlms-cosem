#pragma once
#ifdef ENABLE_DLMS_COSEM_PUSH_MODE

#include <cstdint>
#include <functional>
#include <list>
#include <string>
#include <vector>

#include <client.h>
#include <converters.h>
#include <cosem.h>
#include <dlmssettings.h>

namespace esphome::dlms_cosem {

bool hlp_isValueDataType(DLMS_DATA_TYPE type);

using CosemObjectFoundCallback =
    std::function<void(uint16_t class_id, const uint8_t *obis, DLMS_DATA_TYPE value_type, const uint8_t *value_ptr,
                       uint8_t value_len, const int8_t *scaler, const uint8_t *unit)>;

constexpr uint8_t PUT_BYTE_BACK = 1;

enum class AxdrTokenType : uint8_t {
  EXPECT_TO_BE_FIRST,
  EXPECT_TYPE_EXACT,         // param_u8_a = required DLMS type tag
  EXPECT_TYPE_U_I_8,         // param_u8_a = required DLMS type tag; accept UINT8 or INT8
  EXPECT_CLASS_ID_UNTAGGED,  // capture class_id (big endian)
  EXPECT_OBIS6_TAGGED,       // capture 6-byte tagged OBIS
  EXPECT_OBIS6_UNTAGGED,     // capture 6-byte OBIS
  EXPECT_ATTR8_UNTAGGED,     // capture attribute id (accept INT8/UINT8 depending on flags)
  EXPECT_VALUE_GENERIC,      // capture value: first byte is type tag; supports fixed/variable lengths
  EXPECT_VALUE_DTM_AS_OCTET_STRING,
  EXPECT_STRUCTURE_N,       // expect a structure with element count = param_u8_a
  EXPECT_SCALER_TAGGED,     // capture scaler (INT8 or UINT8)
  EXPECT_UNIT_ENUM_TAGGED,  // capture unit enum (ENUM base + 1 byte value)
  GOING_DOWN,               // capture going down
  GOING_UP,                 // capture going up
};

struct AxdrPatternStep {
  AxdrTokenType type;
  uint8_t param_u8_a{0};
  uint8_t param_u8_b{0};
};

struct AxdrDescriptorPattern {
  const char *name;
  int priority{0};
  std::vector<AxdrPatternStep> steps;

  uint16_t default_class_id{0};
  uint8_t value_attr_id{2};
  uint8_t scaler_unit_attr_id{3};
};

struct AxdrCaptures {
  uint32_t elem_idx{0};
  uint16_t class_id{0};
  const uint8_t *obis{nullptr};
  DLMS_DATA_TYPE value_type{DLMS_DATA_TYPE_NONE};
  const uint8_t *value_ptr{nullptr};
  uint8_t value_len{0};

  bool has_scaler_unit{false};
  int8_t scaler{0};
  uint8_t unit_enum{DLMS_UNIT_NO_UNIT};
};

class AxdrPatternRegistry {
 public:
  void add_pattern(const AxdrDescriptorPattern &p);
  const std::vector<AxdrDescriptorPattern> &patterns() const { return patterns_; }
  void clear() { patterns_.clear(); }

 private:
  std::vector<AxdrDescriptorPattern> patterns_{};
};

class AxdrStreamParser {
  gxByteBuffer *buffer_;
  CosemObjectFoundCallback callback_;
  size_t objects_found_ = 0;
  bool show_log_ = false;

  AxdrPatternRegistry registry_{};
  uint8_t last_pattern_elements_consumed_{0};

  uint8_t peek_byte_();
  uint8_t read_byte_();
  uint16_t read_u16_();
  uint32_t read_u32_();

  bool test_if_date_time_12b_(const uint8_t *buf_in = nullptr);

  bool parse_element_(uint8_t type, uint8_t depth = 0);
  bool parse_sequence_(uint8_t type, uint8_t depth = 0);
  bool parse_data_(uint8_t type, uint8_t depth = 0);
  bool skip_data_(uint8_t type);
  bool skip_sequence_(uint8_t type);

  bool try_match_patterns_(uint8_t elem_idx);
  bool match_pattern_(uint8_t elem_idx, const AxdrDescriptorPattern &pat, uint8_t &elements_consumed_at_level);
  bool capture_generic_value_(AxdrCaptures &c, uint8_t expected_type = 0xFF, uint8_t expected_var_len = 0xFF,
                              uint8_t replacement_type = 0xFF);
  void emit_object_(const AxdrDescriptorPattern &pat, const AxdrCaptures &c);

 public:
  AxdrStreamParser(gxByteBuffer *buf, CosemObjectFoundCallback callback, bool show_log)
      : buffer_(buf), callback_(callback), show_log_(show_log) {}
  size_t parse();
  void register_pattern_dsl(const char *name, const std::string &dsl, int priority = 10);
  void clear_patterns() { registry_.clear(); }
};

}  // namespace esphome::dlms_cosem

#endif  // ENABLE_DLMS_COSEM_PUSH_MODE