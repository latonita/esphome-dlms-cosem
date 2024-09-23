import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import (
    DlmsCosem,
    CONF_DLMS_COSEM_ID,
    dlms_cosem_ns,
    obis_code,
    CONF_OBIS_CODE,
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
        }
    ),
    cv.has_exactly_one_key(CONF_OBIS_CODE),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_DLMS_COSEM_ID])
    var = await text_sensor.new_text_sensor(config)
    cg.add(var.set_obis_code(config[CONF_OBIS_CODE]))
    cg.add(component.register_sensor(var))