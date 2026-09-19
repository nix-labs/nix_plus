import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import uart, sensor, light, binary_sensor, text_sensor, switch, time as time_, text, button
from esphome.const import (
    CONF_ID,
    CONF_LIGHT,
    CONF_TEMPERATURE,
    CONF_TIME_ID,
    UNIT_PERCENT,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
    ICON_BRIGHTNESS_5,
)

CONF_NUM_DIGITS = "num_digits"

CODEOWNERS = ["@nixlabs"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor", "switch", "light", "text", "button"]

nix_plus_ns = cg.esphome_ns.namespace("nix_plus")
NixPlus = nix_plus_ns.class_("NixPlus", cg.Component, uart.UARTDevice)
NixPlusLight = nix_plus_ns.class_("NixPlusLight", light.LightOutput)
NixPlusDisplayLight = nix_plus_ns.class_("NixPlusDisplayLight", light.LightOutput)

CONF_NIX_PLUS_ID = "nix_plus_id"
CONF_AMBIENT_LIGHT = "ambient_light"
CONF_DAY_MODE = "day_mode"
CONF_NIGHT_MODE = "night_mode"
CONF_DISPLAY = "display"
CONF_MODEL_NAME = "model_name"
CONF_IP_ADDRESS = "ip_address"
CONF_AMBIENT_MODE_SWITCH = "ambient_mode_switch"
CONF_SNTP_SWITCH = "sntp_switch"
CONF_TIME_ZONE_TEXT = "time_zone_text"
CONF_DETECT_TIMEZONE_BUTTON = "detect_timezone_button"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NixPlus),
            cv.Optional(CONF_TIME_ID): cv.use_id(time_.RealTimeClock),
            cv.Optional(CONF_NUM_DIGITS, default=0): cv.one_of(0, 4, 6, int=True),
            cv.Optional(CONF_LIGHT): light.RGB_LIGHT_SCHEMA.extend(
                {
                    cv.GenerateID(light.CONF_OUTPUT_ID): cv.declare_id(NixPlusLight),
                }
            ),
            cv.Optional(CONF_DISPLAY): light.BRIGHTNESS_ONLY_LIGHT_SCHEMA.extend(
                {
                    cv.GenerateID(light.CONF_OUTPUT_ID): cv.declare_id(NixPlusDisplayLight),
                }
            ),
            cv.Optional(CONF_AMBIENT_LIGHT): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon=ICON_BRIGHTNESS_5,
                accuracy_decimals=1,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
            ),
            cv.Optional(CONF_DAY_MODE): binary_sensor.binary_sensor_schema(
                icon="mdi:weather-sunny",
            ),
            cv.Optional(CONF_NIGHT_MODE): binary_sensor.binary_sensor_schema(
                icon="mdi:weather-night",
            ),
            cv.Optional(CONF_MODEL_NAME): text_sensor.text_sensor_schema(
                icon="mdi:information",
            ),
            cv.Optional(CONF_IP_ADDRESS): text_sensor.text_sensor_schema(
                icon="mdi:ip-network",
            ),
            cv.Optional(CONF_AMBIENT_MODE_SWITCH): cv.use_id(switch.Switch),
            cv.Optional(CONF_SNTP_SWITCH): cv.use_id(switch.Switch),
            cv.Optional(CONF_TIME_ZONE_TEXT): cv.use_id(text.Text),
            cv.Optional(CONF_DETECT_TIMEZONE_BUTTON): cv.use_id(button.Button),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_num_digits(config[CONF_NUM_DIGITS]))

    if CONF_TIME_ID in config:
        time_comp = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time(time_comp))

    if CONF_AMBIENT_LIGHT in config:
        sens = await sensor.new_sensor(config[CONF_AMBIENT_LIGHT])
        cg.add(var.set_ambient_light_sensor(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_DAY_MODE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_DAY_MODE])
        cg.add(var.set_day_mode_sensor(sens))

    if CONF_NIGHT_MODE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_NIGHT_MODE])
        cg.add(var.set_night_mode_sensor(sens))

    if CONF_MODEL_NAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_MODEL_NAME])
        cg.add(var.set_model_name_sensor(sens))

    if CONF_IP_ADDRESS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_IP_ADDRESS])
        cg.add(var.set_ip_address_sensor(sens))

    if CONF_LIGHT in config:
        conf = config[CONF_LIGHT]
        light_out = cg.new_Pvariable(conf[light.CONF_OUTPUT_ID])
        cg.add(light_out.set_parent(var))
        await light.register_light(light_out, conf)
        backlight_state = await cg.get_variable(conf[CONF_ID])
        cg.add(var.set_backlight_light_state(backlight_state))

    if CONF_DISPLAY in config:
        conf = config[CONF_DISPLAY]
        light_out = cg.new_Pvariable(conf[light.CONF_OUTPUT_ID])
        cg.add(light_out.set_parent(var))
        await light.register_light(light_out, conf)
        disp_state = await cg.get_variable(conf[CONF_ID])
        cg.add(var.set_display_light_state(disp_state))

    if CONF_AMBIENT_MODE_SWITCH in config:
        sw = await cg.get_variable(config[CONF_AMBIENT_MODE_SWITCH])
        cg.add(var.set_ambient_mode_switch(sw))

    if CONF_SNTP_SWITCH in config:
        sw = await cg.get_variable(config[CONF_SNTP_SWITCH])
        cg.add(var.set_sntp_switch(sw))

    if CONF_TIME_ZONE_TEXT in config:
        txt = await cg.get_variable(config[CONF_TIME_ZONE_TEXT])
        cg.add(var.set_time_zone_text(txt))

    if CONF_DETECT_TIMEZONE_BUTTON in config:
        btn = await cg.get_variable(config[CONF_DETECT_TIMEZONE_BUTTON])
        cg.add(var.set_detect_timezone_button(btn))
