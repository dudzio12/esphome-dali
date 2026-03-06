from typing import OrderedDict
from esphome import pins
from esphome.const import CONF_ID, CONF_RX_PIN, CONF_TX_PIN, CONF_DISCOVERY
from esphome.core import CORE

import esphome.codegen as cg
import esphome.config_validation as cv

AUTO_LOAD = ["light", "output", "text_sensor"]

CONF_DALI_BUS = 'dali_bus'
CONF_INITIALIZE_ADDRESSES = 'initialize_addresses'
CONF_DEFAULT_FADE_TIME = 'default_fade_time'
CONF_DEFAULT_FADE_RATE = 'default_fade_rate'
CONF_DEFAULT_BRIGHTNESS_CURVE = 'default_brightness_curve'
CONF_MAX_DEVICES = 'max_devices'
CONF_RESTORE_STATE_ON_TOGGLE = 'restore_state_on_toggle'

# Text sensors per device (device_type, light_source, version, min_level,
# max_level, power_on_level, fade_time, fade_rate, dimming_curve, status,
# + color_temp_range for TC-capable devices)
TEXT_SENSORS_PER_DEVICE = 11

dali_ns = cg.esphome_ns.namespace('dali')
dali_lib_ns = cg.global_ns
DaliBusComponent = dali_ns.class_('DaliBusComponent', cg.Component)

# Import validators and enums from light.py
from .light import validate_fade_time, validate_fade_rate, DALI_BRIGHTNESS_CURVES, DaliLedDimmingCurve

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DaliBusComponent),
    cv.Required(CONF_RX_PIN): pins.internal_gpio_input_pin_schema,
    cv.Required(CONF_TX_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_DISCOVERY): cv.All(cv.requires_component("light"), cv.boolean),
    cv.Optional(CONF_INITIALIZE_ADDRESSES): cv.boolean,
    cv.Optional(CONF_DEFAULT_FADE_TIME): validate_fade_time,
    cv.Optional(CONF_DEFAULT_FADE_RATE): validate_fade_rate,
    cv.Optional(CONF_DEFAULT_BRIGHTNESS_CURVE): cv.enum(DALI_BRIGHTNESS_CURVES),
    cv.Optional(CONF_MAX_DEVICES, default=8): cv.int_range(min=1, max=64),
    cv.Optional(CONF_RESTORE_STATE_ON_TOGGLE, default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config: OrderedDict):
    var = cg.new_Pvariable(config[CONF_ID])
    bus = await cg.register_component(var, config)

    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin))
    
    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))

    if config.get(CONF_DISCOVERY, False):
        cg.add(var.do_device_discovery())

        # Reserve slots in ESPHome's StaticVector for dynamically created entities.
        # ESPHome uses compile-time fixed-size arrays — we must pre-allocate enough
        # slots for all devices that might be discovered at runtime.
        max_devices = config[CONF_MAX_DEVICES]
        for _ in range(max_devices):
            CORE.register_platform_component("light", bus)
        for _ in range(max_devices * TEXT_SENSORS_PER_DEVICE):
            CORE.register_platform_component("text_sensor", bus)

    if config.get(CONF_INITIALIZE_ADDRESSES, False):
        cg.add(var.do_initialize_addresses())

    if CONF_DEFAULT_FADE_TIME in config:
        cg.add(var.set_default_fade_time(config[CONF_DEFAULT_FADE_TIME]))
    if CONF_DEFAULT_FADE_RATE in config:
        cg.add(var.set_default_fade_rate(config[CONF_DEFAULT_FADE_RATE]))
    if CONF_DEFAULT_BRIGHTNESS_CURVE in config:
        cg.add(var.set_default_brightness_curve(config[CONF_DEFAULT_BRIGHTNESS_CURVE]))
    if config.get(CONF_RESTORE_STATE_ON_TOGGLE, False):
        cg.add(var.set_restore_state_on_toggle(True))
