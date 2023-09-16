# Support for i2c based current sensors
#
# Copyright (C) 2023  Karel Funda <karel.funda@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

from extras.adc_temperature import LinearInterpolate

# REPORT_TIME = .8  # time in sec
REPORT_TIME = .5  # time in sec

BME280_CHIP_ADDR = 0x76
INA260_CHIP_ADDR = 0x40

CURRENT_CONSTANT = 1.25
VOLTAGE_CONSTANT = 0.00125
POWER_CONSTANT = 10


INA260_REGS = {
    'CONFIG': 0x00,        # CONFIGURATION REGISTER (R/W)
    'CURRENT': 0x01,       # CURRENT REGISTER (R)
    'BUSVOLTAGE': 0x02,    # BUS VOLTAGE REGISTER (R)
    'POWER': 0x03,         # POWER REGISTER (R)
    'MASK_ENABLE': 0x06,   # MASK ENABLE REGISTER (R/W)
    'ALERT_LIMIT': 0x07,   # ALERT LIMIT REGISTER (R/W)
    'MFG_UID': 0xFE,       # MANUFACTURER UNIQUE ID REGISTER (R)
    'DIE_UID': 0xFF,       # DIE UNIQUE ID REGISTER (R)
}

INA_CHIP_ID_REG = 0xFE
INA_CONFIG_CONTINUOS = 0x111

INA219_REGS = {
    'CONFIG': 0x00,        # CONFIGURATION REGISTER (R/W)
    'VOLTAGE': 0x01,       # SHUNT VOLTAGE REGISTER (R)
    'BUSVOLTAGE': 0x02,    # BUS VOLTAGE REGISTER (R)
    'POWER': 0x03,         # POWER REGISTER (R)
    'CURRENT': 0x04,       # CURRENT REGISTER (R)
    'CALIBRATION': 0x05,   # CALIBRATION REGISTER (R/W)
}

INA_CHIPS = {
    0x54: 'INA260', 0x60: 'INA219'
}

######################################################################
# Default sensors
######################################################################
# temp, resistance
# INA260 = [
#     (22., 11.3), (66., 13.5), (102., 14.8), (170., 16.8),
#     (180., 18.5), (208., 19.2), (210., 18.8), (230., 19.8),
#     (242., 21.0)
# ]
# temp, resistance
INA260 = [
    (24.5, 7.88),
    (30.1, 8.01),
    (62.5, 8.98),
    (109., 10.38),
    (122.5, 10.84),
    (184.5, 13.014),  # 12.01V
    (206.4, 13.51),
    (218.5, 13.89),
    (240.8, 14.80),  # 15.1V
    (300.0, 16.57),  # 18.14V
    (332.0, 17.87)  # 20.01V
]


STATUS_MEASURING = 1 << 3
STATUS_IM_UPDATE = 1
MODE = 1
RUN_GAS = 1 << 4
NB_CONV_0 = 0
EAS_NEW_DATA = 1 << 7
GAS_DONE = 1 << 6
MEASURE_DONE = 1 << 5
RESET_CHIP_VALUE = 0xB6

BME_CHIPS = {
    0x58: 'BMP280', 0x60: 'BME280', 0x61: 'BME680'
}
BME_CHIP_ID_REG = 0xD0


def get_twos_complement(val, bit_size):
    if val & (1 << (bit_size - 1)):
        val -= (1 << bit_size)
    return val


def get_unsigned_short(bits):
    return bits[1] << 8 | bits[0]


def get_signed_short(bits):
    val = get_unsigned_short(bits)
    return get_twos_complement(val, 16)


def get_signed_byte(bits):
    return get_twos_complement(bits, 8)


class ProtoPrintTemperature:
    def __init__(self, config, params):
        self.params = params

        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=INA260_CHIP_ADDR, default_speed=100000)
        self.mcu = self.i2c.get_mcu()

        self.bus_voltage = 0.
        self.current = 0.
        self.power = 0.
        self.resistance = 0.
        self.measured_time = 0
        self.temp = 0
        self.tempStack = TemperatureStack(150)

        self.min_temp = self.max_temp = self.range_switching_error = 0.
        self.max_sample_time = None

        self.dig = self.sample_timer = None
        self.chip_type = 'INA260'
        self.chip_registers = INA260_REGS

        self.printer.add_object("ina260 " + self.name, self)

        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        self._init_ina260()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME

    def _init_ina260(self):
        chip_id = self.read_id()
        if chip_id not in INA_CHIPS.keys():
            logging.info("ina260: Unknown Chip ID received %#x" % chip_id)
        else:
            self.chip_type = INA_CHIPS[chip_id]
            logging.info("ina: Found Chip %s at %#x" % (
                self.chip_type, self.i2c.i2c_address))

        # self.max_sample_time = 10 / 1000
        self.max_sample_time = 5 / 1000  # magic 5ms

        if self.chip_type == 'INA260':
            self.sample_timer = self.reactor.register_timer(
                self._sample_ina260)
            self.chip_registers = INA260_REGS

    def _sample_ina260(self, eventtime):
        try:
            # wait until results are ready
            self.reactor.pause(
                self.reactor.monotonic() + self.max_sample_time)

            _raw_voltage = self.read_register('BUSVOLTAGE', 2)
            _raw_current = self.read_register('CURRENT', 2)
            _raw_power = self.read_register('POWER', 2)

        except Exception:
            logging.exception("INA260: Error reading data")
            self.bus_voltage = .0
            self.current = .0
            self.power = .0
            return self.reactor.NEVER

        self.bus_voltage = abs(int.from_bytes(_raw_voltage, byteorder='big',
                                              signed=False) * VOLTAGE_CONSTANT)

        self.current = abs((int.from_bytes(_raw_current, byteorder='big',
                                           signed=True) * CURRENT_CONSTANT))
        # / 1000.

        self.power = (int.from_bytes(_raw_power, byteorder='big',
                                     signed=False) * POWER_CONSTANT)
        # / 1000.

        if self.current > 0.1 and self.bus_voltage > 0.009:
            self.resistance = (self.bus_voltage * 1000.) / self.current
            self.temp = self._calc_temp(self.resistance)
        else:
            self.resistance = 0.
            self.temp = 0.
            self.current = 0.

        self.measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(
            self.measured_time), self.temp)
        return self.measured_time + REPORT_TIME

    def read_id(self):
        # read chip id register
        regs = [INA_CHIP_ID_REG]
        logging.info("ina260: Readin Chip ID reg %#x" % INA_CHIP_ID_REG)

        params = self.i2c.i2c_read(regs, 1)
        logging.info("ina260: Readin Chip ID reponse: %#x" %
                     bytearray(params['response'])[0])

        return bytearray(params['response'])[0]

    def read_register(self, reg_name, read_len):
        # read a single register

        regs = [self.chip_registers[reg_name]]
        params = self.i2c.i2c_read(regs, read_len)

        return bytearray(params['response'])

    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = self.chip_registers[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)

    def get_status(self, eventtime):

        data = {
            'temperature': self.temp,
            'busvoltage': round(self.bus_voltage, 4),
            'current': round(self.current, 4),
            'power': round(self.power, 2),
            'calcpower': round(self.current * self.bus_voltage, 2),
            'resistance': round(self.resistance, 2),
            'measured_time': round(self.measured_time, 2)
        }
        return data

    def get_temp(self, eventtime):
        return self.temp, 0.

    def _calc_temp(self, resistance):
        if (self.current > 0):
            try:
                # logging.info("ina260 log: %s" % str(self.params))

                li = LinearInterpolate(self.params)
                temperature = li.reverse_interpolate(resistance)
                # logging.info("ina260 log res: %0.2f" % self.resistance)
                # logging.info("ina260 log temp: %0.2f" % temperature)
                # logging.info("ina260 log monotonic: %0.9f" %
                #              self.reactor.monotonic())

                self.tempStack.add_measurement(
                    self.reactor.monotonic(), temperature)

                # return temperature
                return self.tempStack.get_temp_average(10)

            except ValueError as e:
                logging.exception("ina260: %s in %s" % (
                    str(e), self.name))

                # raise ValueError("duplicate value")
                # raise config.error()

        return 0.

        # return round(random.uniform(20, 30), 2)

######################################################################
# helper class for averaging temperature measurements
######################################################################


class TemperatureStack:
    def __init__(self, size):
        self.stack = []
        self.size = size

    def add_measurement(self, time, temp):
        if len(self.stack) == self.size:
            self.stack.pop(0)  # Odstraň nejstarší prvek z zásobníku
        self.stack.append((time, temp))

    def get_stack(self):
        return self.stack

    def get_temp_average(self, last_n=None):

        count = len(self.stack)

        if last_n is None:
            last_n = count

        if last_n > count:
            last_n = count

        if last_n == 0:
            return None

        # logging.info("ina260 last_n: %d" % last_n)

        total_temperature = 0.
        stack_n = self.stack[-last_n:]

        for time, temperature in stack_n:
            total_temperature += temperature

        # logging.info("ina260 last_n: %s" % str(stack_n))

        # return 1.1
        return total_temperature / len(stack_n)


# loads temperature resistance params
def _load_config_data(config):
    params = []

    if config.get("temperature1", None) is None:
        params = INA260
    else:
        for i in range(1, 1000):
            t = config.getfloat("temperature%d" % (i,), None)
            if t is None:
                break
            r = config.getfloat("resistance%d" % (i,))
            params.append((t, r))

    return params


def load_config_prefix(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    params = _load_config_data(config)
    sensor = ProtoPrintTemperature(config, params)

    # func = (lambda config, params=params:
    #         sensor)

    pheaters.add_sensor_factory(
        sensor.name, sensor)


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    params = _load_config_data(config)

    func = (lambda config, params=params:
            ProtoPrintTemperature(config, params))

    pheaters.add_sensor_factory(
        "INA260", func)

# def load_config_prefix(config):
#     return INA260(config)
