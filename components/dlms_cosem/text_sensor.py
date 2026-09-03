import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import (
    DlmsCosem,
    dlms_cosem_ns,
    obis_code,
    CONF_DLMS_COSEM_ID,
    CONF_OBIS_CODE,
    CONF_DONT_PUBLISH,
    CONF_OBIS_CLASS,
    CONF_CP1251,
    CONF_ATTRIBUTE,
    validate_attribute,
)

AUTO_LOAD = ["dlms_cosem"]

DlmsCosemTextSensor = dlms_cosem_ns.class_(
    "DlmsCosemTextSensor", text_sensor.TextSensor
)


CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(
        DlmsCosemTextSensor,
    ).extend(
        {
            cv.GenerateID(CONF_DLMS_COSEM_ID): cv.use_id(DlmsCosem),
            cv.Required(CONF_OBIS_CODE): obis_code,
            cv.Optional(CONF_DONT_PUBLISH, default=False): cv.boolean,
            cv.Optional(CONF_OBIS_CLASS, default=1): cv.int_,
            cv.Optional(CONF_CP1251): cv.boolean,
            cv.Optional(CONF_ATTRIBUTE): cv.int_range(min=2, max=11),
        }
    ),
    cv.has_exactly_one_key(CONF_OBIS_CODE),
    validate_attribute,
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_DLMS_COSEM_ID])
    var = await text_sensor.new_text_sensor(config)
    cg.add(var.set_obis_code(config[CONF_OBIS_CODE]))
    cg.add(var.set_dont_publish(config.get(CONF_DONT_PUBLISH)))
    cg.add(var.set_obis_class(config[CONF_OBIS_CLASS]))
    cg.add(var.set_attribute(config[CONF_ATTRIBUTE]))

    if conf := config.get(CONF_CP1251):
        cg.add(var.set_cp1251_conversion_required(config[CONF_CP1251]))
        
    cg.add(component.register_sensor(var))
