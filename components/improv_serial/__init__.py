import esphome.codegen as cg
from esphome.components import improv_base, uart
import esphome.config_validation as cv
from esphome.const import CONF_ID

AUTO_LOAD = ["improv_base"]
CODEOWNERS = ["@nixlabs"]
DEPENDENCIES = ["wifi", "uart"]

improv_serial_ns = cg.esphome_ns.namespace("improv_serial")

ImprovSerialComponent = improv_serial_ns.class_(
    "ImprovSerialComponent", cg.Component, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ImprovSerialComponent),
        }
    )
    .extend(improv_base.IMPROV_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await improv_base.setup_improv_core(var, config, "improv_serial")
    cg.add_define("USE_IMPROV_SERIAL")
