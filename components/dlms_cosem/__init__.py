import re
from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_AUTH,
    CONF_BAUD_RATE,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
    CONF_FLOW_CONTROL_PIN,
    CONF_PASSWORD,
)

CODEOWNERS = ["@latonita","@shammysha"]

MULTI_CONF = True

DEPENDENCIES = ["uart"]

DEFAULTS_MAX_SENSOR_INDEX = 12
DEFAULTS_BAUD_RATE_HANDSHAKE = 9600
DEFAULTS_BAUD_RATE_SESSION = 9600
DEFAULTS_RECEIVE_TIMEOUT = "500ms"
DEFAULTS_DELAY_BETWEEN_REQUESTS = "50ms"
DEFAULTS_UPDATE_INTERVAL = "60s"

CONF_DLMS_COSEM_ID = "dlms_cosem_id"
CONF_OBIS_CODE = "obis_code"
CONF_OBIS_CLASS = "obis_class"
CONF_CLIENT_ADDRESS = "client_address"
CONF_SERVER_ADDRESS = "server_address"
CONF_LOGICAL_DEVICE = "logical_device"
CONF_PHYSICAL_DEVICE = "physical_device"
CONF_ADDRESS_LENGTH = "address_length"
CONF_DELAY_BETWEEN_REQUESTS = "delay_between_requests"
CONF_DONT_PUBLISH = "dont_publish"
CONF_CP1251 = "cp1251"
CONF_ATTRIBUTE = "attribute"

# COSEM interface class 71 (Limiter). Unlike Data/Register/Clock, the values worth reading are not
# in attribute 2 (that one is the monitored_value structure) but in the thresholds and the
# emergency-profile flag listed below.
OBIS_CLASS_LIMITER = 71
LIMITER_ATTRIBUTES = {
    3: "threshold_active",
    4: "threshold_normal",
    5: "threshold_emergency",
    6: "min_over_threshold_duration",
    7: "min_under_threshold_duration",
    10: "emergency_profile_active",
}
DEFAULT_ATTRIBUTE = 2
DEFAULT_LIMITER_ATTRIBUTE = 3

CONF_PUSH_MODE = "push_mode"
CONF_PUSH_SHOW_LOG = "push_show_log"
CONF_PUSH_CUSTOM_PATTERN = "push_custom_pattern"

CONF_REBOOT_AFTER_FAILURE = "reboot_after_failure"

CONF_BAUD_RATE_HANDSHAKE = "baud_rate_handshake"

dlms_cosem_ns = cg.esphome_ns.namespace("dlms_cosem")
DlmsCosem = dlms_cosem_ns.class_(
    "DlmsCosemComponent", cg.Component, uart.UARTDevice
)

# Referenced by name only so the parent UART can be recognized as our BLE NUS
# client (which has no hardware UART port) without importing that component.
ble_nus_client_ns = cg.esphome_ns.namespace("ble_nus_client")
BLENUSClientComponent = ble_nus_client_ns.class_(
    "BLENUSClientComponent", uart.UARTComponent
)

BAUD_RATES = [300, 600, 1200, 2400, 4800, 9600, 19200]
ADDRESS_LENGTH_ENUM = [1, 2, 4]

def obis_code(value):
    value = cv.string(value)
    # examples of valid OBIS codes that should pass:
    #   0.0.96.14.0.255
    #   1-0:32.7.0.255
    #   1-0:32.7.0*255

    # output will always be in format x.x.x.x.x.x

    # So we just accept dots, dashes, colons, and asterisks as separators
    match = re.match(r"^\d{1,3}[.\-:*]\d{1,3}[.\-:*]\d{1,3}[.\-:*]\d{1,3}[.\-:*]\d{1,3}[.\-:*]\d{1,3}$", value)
    if match is None:
        raise cv.Invalid(f"{value} is not a valid OBIS code. Use format A.B.C.D.E.F or A-B:C.D.E*F")
    
    # Normalize to dot-separated format
    normalized = re.sub(r'[.\-:*]', '.', value)
    return normalized


def validate_attribute(config):
    """Pick the default COSEM attribute for the object class and reject unreadable ones."""
    obis_class = config[CONF_OBIS_CLASS]
    if CONF_ATTRIBUTE not in config:
        config[CONF_ATTRIBUTE] = (
            DEFAULT_LIMITER_ATTRIBUTE
            if obis_class == OBIS_CLASS_LIMITER
            else DEFAULT_ATTRIBUTE
        )
    attribute = config[CONF_ATTRIBUTE]
    if obis_class == OBIS_CLASS_LIMITER and attribute not in LIMITER_ATTRIBUTES:
        readable = ", ".join(f"{k} ({v})" for k, v in LIMITER_ATTRIBUTES.items())
        raise cv.Invalid(
            f"Attribute {attribute} of the Limiter (obis_class {OBIS_CLASS_LIMITER}) cannot be "
            f"read as a sensor. Use one of: {readable}"
        )
    return config


def validate_meter_address(value):
    if len(value) > 15:
        raise cv.Invalid("Meter address length must be no longer than 15 characters")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DlmsCosem),
            cv.Optional(CONF_CLIENT_ADDRESS, default=16): cv.positive_int,
            cv.Optional(CONF_SERVER_ADDRESS, default=1): cv.Any(
                cv.positive_int,
                cv.Schema({
                    cv.Optional(CONF_LOGICAL_DEVICE, default=1): cv.positive_int,
                    cv.Required(CONF_PHYSICAL_DEVICE): cv.positive_int,
                    cv.Optional(CONF_ADDRESS_LENGTH, default=2): cv.one_of(1, 2, 4),
                })
            ),
            cv.Optional(CONF_AUTH, default=False): cv.boolean,
            cv.Optional(CONF_PASSWORD, default=""): cv.string,
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_BAUD_RATE_HANDSHAKE, default=DEFAULTS_BAUD_RATE_HANDSHAKE
            ): cv.one_of(*BAUD_RATES),
            cv.Optional(CONF_BAUD_RATE, default=DEFAULTS_BAUD_RATE_SESSION): cv.one_of(
                *BAUD_RATES
            ),
            cv.Optional(
                CONF_RECEIVE_TIMEOUT, default=DEFAULTS_RECEIVE_TIMEOUT
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_DELAY_BETWEEN_REQUESTS, default=DEFAULTS_DELAY_BETWEEN_REQUESTS
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_UPDATE_INTERVAL, default=DEFAULTS_UPDATE_INTERVAL
            ): cv.update_interval,
            cv.Optional(CONF_REBOOT_AFTER_FAILURE, default=0): cv.int_range(
                min=0, max=100
            ),
            cv.Optional(CONF_CP1251, default=True): cv.boolean,
            cv.Optional(CONF_PUSH_MODE, default=False): cv.boolean,
            cv.Optional(CONF_PUSH_SHOW_LOG, default=False): cv.boolean,
            cv.Optional(CONF_PUSH_CUSTOM_PATTERN, default=""): cv.string,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Detect whether the UART parent is our BLE NUS client (no hardware UART port).
    full_id, _ = await cg.get_variable_with_full_id(config[uart.CONF_UART_ID])
    is_ble_nus = isinstance(
        full_id.type, cg.MockObjClass
    ) and full_id.type.inherits_from(BLENUSClientComponent)
    cg.add(var.set_uart_is_ble_nus(is_ble_nus))

    if flow_control_pin := config.get(CONF_FLOW_CONTROL_PIN):
        pin = await cg.gpio_pin_expression(flow_control_pin)
        cg.add(var.set_flow_control_pin(pin))

    if isinstance(config[CONF_SERVER_ADDRESS], int):
        cg.add(var.set_server_address(config[CONF_SERVER_ADDRESS]))
    else:
        cg.add(var.set_server_address(config[CONF_SERVER_ADDRESS][CONF_LOGICAL_DEVICE], 
                                      config[CONF_SERVER_ADDRESS][CONF_PHYSICAL_DEVICE], 
                                      config[CONF_SERVER_ADDRESS][CONF_ADDRESS_LENGTH]))    
     
    cg.add(var.set_client_address(config[CONF_CLIENT_ADDRESS]))
    cg.add(var.set_auth_required(config[CONF_AUTH]))
    cg.add(var.set_password(config[CONF_PASSWORD]))
    cg.add(var.set_baud_rates(config[CONF_BAUD_RATE_HANDSHAKE], config[CONF_BAUD_RATE]))
    cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT]))
    cg.add(var.set_delay_between_requests_ms(config[CONF_DELAY_BETWEEN_REQUESTS]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_reboot_after_failure(config[CONF_REBOOT_AFTER_FAILURE]))
    cg.add(var.set_cp1251_conversion_required(config[CONF_CP1251]))

    if config[CONF_PUSH_MODE] == True:
        cg.add_build_flag("-DENABLE_DLMS_COSEM_PUSH_MODE")
        cg.add(var.set_push_mode(config[CONF_PUSH_MODE]))
        cg.add(var.set_push_show_log(config[CONF_PUSH_SHOW_LOG]))
        cg.add(var.set_push_custom_pattern_dsl(config[CONF_PUSH_CUSTOM_PATTERN]))
    
    #cg.add_build_flag("-Wno-error=implicit-function-declaration")
    cg.add_library("GuruxDLMS_c", None, "https://github.com/latonita/GuruxDLMS_c")
    # Its a hard-copy of this one, which is a 2-y.o. fork of official gurux repo + platformio json lib file
    # cg.add_library("GuruxDLMS", None, "https://github.com/viric/GuruxDLMS.c#platformio")
