# Support for i2c based current sensors
#
# Copyright (C) 2023  Karel Funda <karel.funda@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

import random

REPORT_TIME = .8
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


BME280_REGS = {
    'RESET': 0xE0,
    'CTRL_HUM': 0xF2,
    'STATUS': 0xF3,
    'CTRL_MEAS': 0xF4,
    'CONFIG': 0xF5,
    'PRESSURE_MSB': 0xF7,
    'PRESSURE_LSB': 0xF8,
    'PRESSURE_XLSB': 0xF9,
    'TEMP_MSB': 0xFA,
    'TEMP_LSB': 0xFB,
    'TEMP_XLSB': 0xFC,
    'HUM_MSB': 0xFD,
    'HUM_LSB': 0xFE,
    'CAL_1': 0x88,
    'CAL_2': 0xE1
}

BME680_REGS = {
    'RESET': 0xE0, 'CTRL_HUM': 0x72, 'CTRL_GAS_1': 0x71, 'CTRL_GAS_0': 0x70,
    'GAS_WAIT_0': 0x64, 'RES_HEAT_0': 0x5A, 'IDAC_HEAT_0': 0x50,
    'STATUS': 0x73, 'EAS_STATUS_0': 0x1D, 'CTRL_MEAS': 0x74, 'CONFIG': 0x75,
    'GAS_R_LSB': 0x2B, 'GAS_R_MSB': 0x2A,
    'PRESSURE_MSB': 0x1F, 'PRESSURE_LSB': 0x20, 'PRESSURE_XLSB': 0x21,
    'TEMP_MSB': 0x22, 'TEMP_LSB': 0x23, 'TEMP_XLSB': 0x24,
    'HUM_MSB': 0x25, 'HUM_LSB': 0x26, 'CAL_1': 0x88, 'CAL_2': 0xE1,
    'RES_HEAT_VAL': 0x00, 'RES_HEAT_RANGE': 0x02, 'RANGE_SWITCHING_ERROR': 0x04
}

BME680_GAS_CONSTANTS = {
    0: (1., 8000000.),
    1: (1., 4000000.),
    2: (1., 2000000.),
    3: (1., 1000000.),
    4: (1., 499500.4995),
    5: (0.99, 248262.1648),
    6: (1., 125000.),
    7: (0.992, 63004.03226),
    8: (1., 31281.28128),
    9: (1., 15625.),
    10: (0.998, 7812.5),
    11: (0.995, 3906.25),
    12: (1., 1953.125),
    13: (0.99, 976.5625),
    14: (1., 488.28125),
    15: (1., 244.140625)
}

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


class INA2601:

    def __init__(self, config):
        logging.info("INA260 init")
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()

        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=INA260_CHIP_ADDR, default_speed=100000)
        # self.mcu = self.i2c.get_mcu()

        self.iir_filter = config.getint('bme280_iir_filter', 1)
        self.os_temp = config.getint('bme280_oversample_temp', 2)
        self.os_hum = config.getint('bme280_oversample_hum', 2)
        self.os_pres = config.getint('bme280_oversample_pressure', 2)

        self.bus_voltage = 0.
        self.current = 0.
        self.power = 0.
        self.min_temp = 0.
        self.max_temp = 0.

        # self.min_temp = self.max_temp = self.range_switching_error = 0.
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
        self._init_ina2xx()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME

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

    def read_id(self):
        # read chip id register
        regs = [INA260_REGS["MFG_UID"]]
        params = self.i2c.i2c_read(regs, 1)
        return bytearray(params['response'])[0]

    def get_status(self, eventtime):
        self.bus_voltage = round(random.uniform(0, 24), 2)
        self.current = round(random.uniform(0, 2), 2)
        self.power = self.bus_voltage * self.current

        data = {
            'temperature': round(self.bus_voltage, 2),
            'busvoltage': round(self.bus_voltage, 2),
            'current': round(self.current, 2),
            'power': round(self.power, 2)
        }
        return data

    def _init_ina2xx(self):
        pass


# def load_config_prefix(config):
#     return INA260(config)


class INA260:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=INA260_CHIP_ADDR, default_speed=100000)
        self.mcu = self.i2c.get_mcu()
        self.iir_filter = config.getint('bme280_iir_filter', 1)
        self.os_temp = config.getint('bme280_oversample_temp', 2)
        self.os_hum = config.getint('bme280_oversample_hum', 2)
        self.os_pres = config.getint('bme280_oversample_pressure', 2)
        self.gas_heat_temp = config.getint('bme280_gas_target_temp', 320)
        self.gas_heat_duration = config.getint('bme280_gas_heat_duration', 150)
        logging.info("BMxx80: Oversampling: Temp %dx Humid %dx Pressure %dx" % (
            pow(2, self.os_temp - 1), pow(2, self.os_hum - 1),
            pow(2, self.os_pres - 1)))
        logging.info("BMxx80: IIR: %dx" % (pow(2, self.iir_filter) - 1))

        self.bus_voltage = 0.
        self.current = 0.
        self.power = 0.

        self.temp = self.pressure = self.humidity = self.gas = self.t_fine = 0.
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
        def read_calibration_data_bmp280(calib_data_1):
            dig = {}
            dig['T1'] = get_unsigned_short(calib_data_1[0:2])
            dig['T2'] = get_signed_short(calib_data_1[2:4])
            dig['T3'] = get_signed_short(calib_data_1[4:6])

            dig['P1'] = get_unsigned_short(calib_data_1[6:8])
            dig['P2'] = get_signed_short(calib_data_1[8:10])
            dig['P3'] = get_signed_short(calib_data_1[10:12])
            dig['P4'] = get_signed_short(calib_data_1[12:14])
            dig['P5'] = get_signed_short(calib_data_1[14:16])
            dig['P6'] = get_signed_short(calib_data_1[16:18])
            dig['P7'] = get_signed_short(calib_data_1[18:20])
            dig['P8'] = get_signed_short(calib_data_1[20:22])
            dig['P9'] = get_signed_short(calib_data_1[22:24])
            return dig

        def read_calibration_data_bme280(calib_data_1, calib_data_2):
            dig = read_calibration_data_bmp280(calib_data_1)
            dig['H1'] = calib_data_1[25] & 0xFF
            dig['H2'] = get_signed_short(calib_data_2[0:2])
            dig['H3'] = calib_data_2[2] & 0xFF
            dig['H4'] = get_twos_complement(
                (calib_data_2[3] << 4) | (calib_data_2[4] & 0x0F), 12)
            dig['H5'] = get_twos_complement(
                (calib_data_2[5] << 4) | ((calib_data_2[4] & 0xF0) >> 4), 12)
            dig['H6'] = get_twos_complement(calib_data_2[6], 8)
            return dig

        def read_calibration_data_bme680(calib_data_1, calib_data_2):
            dig = {}
            dig['T1'] = get_unsigned_short(calib_data_2[8:10])
            dig['T2'] = get_signed_short(calib_data_1[2:4])
            dig['T3'] = get_signed_byte(calib_data_1[4])

            dig['P1'] = get_unsigned_short(calib_data_1[6:8])
            dig['P2'] = get_signed_short(calib_data_1[8:10])
            dig['P3'] = calib_data_1[10]
            dig['P4'] = get_signed_short(calib_data_1[12:14])
            dig['P5'] = get_signed_short(calib_data_1[14:16])
            dig['P6'] = get_signed_byte(calib_data_1[17])
            dig['P7'] = get_signed_byte(calib_data_1[16])
            dig['P8'] = get_signed_short(calib_data_1[20:22])
            dig['P9'] = get_signed_short(calib_data_1[22:24])
            dig['P10'] = calib_data_1[24]

            dig['H1'] = get_twos_complement(
                (calib_data_2[2] << 4) | (calib_data_2[1] & 0x0F), 12)
            dig['H2'] = get_twos_complement(
                (calib_data_2[0] << 4) | ((calib_data_2[1] & 0xF0) >> 4), 12)
            dig['H3'] = get_signed_byte(calib_data_2[3])
            dig['H4'] = get_signed_byte(calib_data_2[4])
            dig['H5'] = get_signed_byte(calib_data_2[5])
            dig['H6'] = calib_data_2[6]
            dig['H7'] = get_signed_byte(calib_data_2[7])

            dig['G1'] = get_signed_byte(calib_data_2[12])
            dig['G2'] = get_signed_short(calib_data_2[10:12])
            dig['G3'] = get_signed_byte(calib_data_2[13])
            return dig

        chip_id = self.read_id()
        if chip_id not in INA_CHIPS.keys():
            logging.info("ina260: Unknown Chip ID received %#x" % chip_id)
        else:
            self.chip_type = INA_CHIPS[chip_id]
            logging.info("ina: Found Chip %s at %#x" % (
                self.chip_type, self.i2c.i2c_address))

        # Reset chip
        # self.write_register('RESET', [RESET_CHIP_VALUE])
        # self.reactor.pause(self.reactor.monotonic() + .5)

        # Make sure non-volatile memory has been copied to registers
        # status = self.read_register('STATUS', 1)[0]
        # while status & STATUS_IM_UPDATE:
        #     self.reactor.pause(self.reactor.monotonic() + .01)
        #     status = self.read_register('STATUS', 1)[0]

        # self.max_sample_time = 10 / 1000
        self.max_sample_time = 10 / 1000  # magic

        if self.chip_type == 'INA260':
            self.sample_timer = self.reactor.register_timer(
                self._sample_ina260)
            self.chip_registers = INA260_REGS

        # if self.chip_type in ('BME680', 'BME280'):
        #     self.write_register('CONFIG', (self.iir_filter & 0x07) << 2)

        # Read out and calculate the trimming parameters
        # cal_1 = self.read_register('CAL_1', 26)
        # cal_2 = self.read_register('CAL_2', 16)
        # if self.chip_type == 'BME280':
        #     self.dig = read_calibration_data_bme280(cal_1, cal_2)
        # elif self.chip_type == 'BMP280':
        #     self.dig = read_calibration_data_bmp280(cal_1)
        # elif self.chip_type == 'BME680':
        #     self.dig = read_calibration_data_bme680(cal_1, cal_2)

    def _sample_ina260(self, eventtime):
        # Enter forced mode
        # if self.chip_type == 'BME280':
        #     self.write_register('CTRL_HUM', self.os_hum)
        # meas = self.os_temp << 5 | self.os_pres << 2 | MODE
        # self.write_register('CTRL_MEAS', meas)

        try:
            # wait until results are ready
            self.reactor.pause(
                self.reactor.monotonic() + self.max_sample_time)

            _raw_voltage = self.read_register('BUSVOLTAGE', 2)
            _raw_current = self.read_register('CURRENT', 2)
            _raw_power = self.read_register('POWER', 2)
            # _raw_config = self.read_register('MASK_ENABLE', 2)

            # config_register_int = int.from_bytes(
            #     _raw_config, byteorder='big', signed=False)
            # config_register_binary_string = format(config_register_int, "016b")

            # logging.info("ina260: register response: %s" %
            #              config_register_binary_string)

        except Exception:
            logging.exception("INA260: Error reading data")
            self.bus_voltage = .0
            self.current = .0
            self.power = .0
            return self.reactor.NEVER

        self.bus_voltage = int.from_bytes(
            _raw_voltage, byteorder='big', signed=False) * VOLTAGE_CONSTANT
        self.current = int.from_bytes(
            _raw_current, byteorder='big', signed=True) * CURRENT_CONSTANT
        self.power = int.from_bytes(
            _raw_power, byteorder='big', signed=False) * POWER_CONSTANT

        # pressure_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        # self.pressure = self._compensate_pressure_bme280(pressure_raw) / 100.
        # if self.chip_type == 'BME280':
        #     humid_raw = (data[6] << 8) | data[7]
        #     self.humidity = self._compensate_humidity_bme280(humid_raw)
        # if self.temp < self.min_temp or self.temp > self.max_temp:
        #     self.printer.invoke_shutdown(
        #         "BME280 temperature %0.1f outside range of %0.1f:%.01f"
        #         % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + REPORT_TIME

    def _sample_bme680(self, eventtime):
        self.write_register('CTRL_HUM', self.os_hum & 0x07)
        meas = self.os_temp << 5 | self.os_pres << 2
        self.write_register('CTRL_MEAS', [meas])

        gas_wait_0 = self._calculate_gas_heater_duration(
            self.gas_heat_duration)
        self.write_register('GAS_WAIT_0', [gas_wait_0])
        res_heat_0 = self._calculate_gas_heater_resistance(self.gas_heat_temp)
        self.write_register('RES_HEAT_0', [res_heat_0])
        gas_config = RUN_GAS | NB_CONV_0
        self.write_register('CTRL_GAS_1', [gas_config])

        def data_ready(stat):
            new_data = (stat & EAS_NEW_DATA)
            gas_done = not (stat & GAS_DONE)
            meas_done = not (stat & MEASURE_DONE)
            return new_data and gas_done and meas_done

        # Enter forced mode
        meas = meas | MODE
        self.write_register('CTRL_MEAS', meas)
        try:
            # wait until results are ready
            status = self.read_register('EAS_STATUS_0', 1)[0]
            while not data_ready(status):
                self.reactor.pause(
                    self.reactor.monotonic() + self.max_sample_time)
                status = self.read_register('EAS_STATUS_0', 1)[0]

            data = self.read_register('PRESSURE_MSB', 8)
            gas_data = self.read_register('GAS_R_MSB', 2)
        except Exception:
            logging.exception("BME680: Error reading data")
            self.temp = self.pressure = self.humidity = self.gas = .0
            return self.reactor.NEVER

        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        if temp_raw != 0x80000:
            self.temp = self._compensate_temp(temp_raw)
        pressure_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        if pressure_raw != 0x80000:
            self.pressure = self._compensate_pressure_bme680(
                pressure_raw) / 100.
        humid_raw = (data[6] << 8) | data[7]
        self.humidity = self._compensate_humidity_bme680(humid_raw)

        gas_valid = ((gas_data[1] & 0x20) == 0x20)
        if gas_valid:
            gas_heater_stable = ((gas_data[1] & 0x10) == 0x10)
            if not gas_heater_stable:
                logging.warning("BME680: Gas heater didn't reach target")
            gas_raw = (gas_data[0] << 2) | ((gas_data[1] & 0xC0) >> 6)
            gas_range = (gas_data[1] & 0x0F)
            self.gas = self._compensate_gas(gas_raw, gas_range)

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "BME680 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))
        measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + REPORT_TIME * 4

    def _compensate_temp(self, raw_temp):
        dig = self.dig
        var1 = ((raw_temp / 16384. - (dig['T1'] / 1024.)) * dig['T2'])
        var2 = (
            ((raw_temp / 131072.) - (dig['T1'] / 8192.)) *
            ((raw_temp / 131072.) - (dig['T1'] / 8192.)) * dig['T3'])
        self.t_fine = var1 + var2
        return self.t_fine / 5120.0

    def _compensate_pressure_bme280(self, raw_pressure):
        dig = self.dig
        t_fine = self.t_fine
        var1 = t_fine / 2. - 64000.
        var2 = var1 * var1 * dig['P6'] / 32768.
        var2 = var2 + var1 * dig['P5'] * 2.
        var2 = var2 / 4. + (dig['P4'] * 65536.)
        var1 = (dig['P3'] * var1 * var1 / 524288. + dig['P2'] * var1) / 524288.
        var1 = (1. + var1 / 32768.) * dig['P1']
        if var1 == 0:
            return 0.
        else:
            pressure = 1048576.0 - raw_pressure
            pressure = ((pressure - var2 / 4096.) * 6250.) / var1
            var1 = dig['P9'] * pressure * pressure / 2147483648.
            var2 = pressure * dig['P8'] / 32768.
            return pressure + (var1 + var2 + dig['P7']) / 16.

    def _compensate_pressure_bme680(self, raw_pressure):
        dig = self.dig
        t_fine = self.t_fine
        var1 = t_fine / 2. - 64000.
        var2 = var1 * var1 * dig['P6'] / 131072.
        var2 = var2 + var1 * dig['P5'] * 2.
        var2 = var2 / 4. + (dig['P4'] * 65536.)
        var1 = (dig['P3'] * var1 * var1 / 16384. + dig['P2'] * var1) / 524288.
        var1 = (1. + var1 / 32768.) * dig['P1']
        if var1 == 0:
            return 0.
        else:
            pressure = 1048576.0 - raw_pressure
            pressure = ((pressure - var2 / 4096.) * 6250.) / var1
            var1 = dig['P9'] * pressure * pressure / 2147483648.
            var2 = pressure * dig['P8'] / 32768.
            var3 = (pressure / 256.) * (pressure / 256.) * (pressure / 256.) * (
                dig['P10'] / 131072.)
            return pressure + (var1 + var2 + var3 + (dig['P7'] * 128.)) / 16.

    def _compensate_humidity_bme280(self, raw_humidity):
        dig = self.dig
        t_fine = self.t_fine
        humidity = t_fine - 76800.
        h1 = (
            raw_humidity - (
                dig['H4'] * 64. + dig['H5'] / 16384. * humidity))
        h2 = (dig['H2'] / 65536. * (1. + dig['H6'] / 67108864. * humidity *
                                    (1. + dig['H3'] / 67108864. * humidity)))
        humidity = h1 * h2
        humidity = humidity * (1. - dig['H1'] * humidity / 524288.)
        return min(100., max(0., humidity))

    def _compensate_humidity_bme680(self, raw_humidity):
        dig = self.dig
        temp_comp = self.temp

        var1 = raw_humidity - (
            (dig['H1'] * 16.) + ((dig['H3'] / 2.) * temp_comp))
        var2 = var1 * ((dig['H2'] / 262144.) *
                       (1. + ((dig['H4'] / 16384.) * temp_comp) +
                        ((dig['H5'] / 1048576.) * temp_comp * temp_comp)))
        var3 = dig['H6'] / 16384.
        var4 = dig['H7'] / 2097152.
        humidity = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2)
        return min(100., max(0., humidity))

    def _compensate_gas(self, gas_raw, gas_range):
        gas_switching_error = self.read_register('RANGE_SWITCHING_ERROR', 1)[0]
        var1 = (1340. + 5. * gas_switching_error) * \
            BME680_GAS_CONSTANTS[gas_range][0]
        gas = var1 * BME680_GAS_CONSTANTS[gas_range][1] / (
            gas_raw - 512. + var1)
        return gas

    def _calculate_gas_heater_resistance(self, target_temp):
        amb_temp = self.temp
        heater_data = self.read_register('RES_HEAT_VAL', 3)
        res_heat_val = get_signed_byte(heater_data[0])
        res_heat_range = (heater_data[2] & 0x30) >> 4
        dig = self.dig
        var1 = (dig['G1'] / 16.) + 49.
        var2 = ((dig['G2'] / 32768.) * 0.0005) + 0.00235
        var3 = dig['G3'] / 1024.
        var4 = var1 * (1. + (var2 * target_temp))
        var5 = var4 + (var3 * amb_temp)
        res_heat = (3.4 * ((var5 * (4. / (4. + res_heat_range))
                            * (1. / (1. + (res_heat_val * 0.002)))) - 25))
        return int(res_heat)

    def _calculate_gas_heater_duration(self, duration_ms):
        if duration_ms >= 4032:
            duration_reg = 0xff
        else:
            factor = 0
            while duration_ms > 0x3F:
                duration_ms //= 4
                factor += 1
            duration_reg = duration_ms + (factor * 64)

        return duration_reg

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
        # logging.info("ina260: Readin register %#x and len: %d" %
        #              (self.chip_registers[reg_name], read_len))

        regs = [self.chip_registers[reg_name]]
        params = self.i2c.i2c_read(regs, read_len)

        # logging.info("ina260: Readin reponse test: %s" %
        #              str(params))

        return bytearray(params['response'])

    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = self.chip_registers[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)

    # def get_status(self, eventtime):
    #     data = {
    #         'temperature': round(self.temp, 2),
    #         'pressure': self.pressure
    #     }
    #     if self.chip_type in ('BME280', 'BME680'):
    #         data['humidity'] = self.humidity
    #     if self.chip_type == 'BME680':
    #         data['gas'] = self.gas
    #     return data

    def get_status(self, eventtime):
        # self.bus_voltage = round(random.uniform(0, 24), 2)
        # self.current = round(random.uniform(0, 2), 2)
        # self.power = self.bus_voltage * self.current

        data = {
            'temperature': round(random.uniform(20, 300), 2),
            'busvoltage': round(self.bus_voltage, 2),
            'current': round(self.current, 2),
            'power': round(self.power, 2)
        }
        return data


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    # pheaters.add_sensor_factory("BME280", BME280)
    pheaters.add_sensor_factory("INA260", INA260)
