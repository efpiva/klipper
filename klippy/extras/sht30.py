# SHT30(F)/Si7013/Si7020/Si7021/SHT21 i2c based temperature sensors support
#
# Copyright (C) 2020  Lucio Tarantino <lucio.tarantino@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import time
from . import bus

SHT30_I2C_ADDR= 0x44

SHT30_COMMANDS = {
    'MEASURE'           :0x2C,
    'RESET'             :[0x30,0xA2]
}

SHT30_RESOLUTION = 0.50

#crc8 polynomial for 16bit value, CRC8 -> x^8 + x^5 + x^4 + 1
SHT30_CRC8_POLYNOMINAL= 0x131

class SHT30:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=SHT30_I2C_ADDR, default_speed=100000)
        self.hold_master_mode = config.getboolean('sht30_hold_master',False)
        self.report_time = config.getint('sht30_report_time',30,minval=5)
        self.deviceId = config.get('sensor_type')
        self.temp = self.min_temp = self.max_temp = self.humidity = 0.
        self.sample_timer = self.reactor.register_timer(self._sample_sht30)
        self.printer.add_object("sht30 " + self.name, self) # + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        self._init_sht30()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _init_sht30(self):
        # Device Soft Reset
        self.i2c.i2c_write(SHT30_COMMANDS['RESET'])
        # Wait 15ms after reset
        self.reactor.pause(self.reactor.monotonic() + .15)

    def _sample_sht30(self, eventtime):
        try:
            params = self.i2c.i2c_write([SHT30_COMMANDS['MEASURE'],0x06])

            time.sleep(0.5)

            params = self.i2c.i2c_read([0x00],6)

            response = bytearray(params['response'])
            rtemp  = response[0] << 8
            rtemp |= response[1]

            if self._chekCRC8(response[:2]) != response[2]:
                logging.warn("sht30: Checksum error on Temperature reading!")
                self.temp = ((175 * float(rtemp))/65535.0) - 45
            else:
                self.temp = ((175 * float(rtemp))/65535.0) - 45
                logging.debug("sht30: Temperature %.2f " % self.temp)


            rhumid = response[3] << 8
            rhumid|= response[4]
            if self._chekCRC8(response[3:5]) != response[5]:
                logging.warn("sht30: Checksum error on Humidity reading!")
            else:
                #clear status bits,
                # humidity always returns xxxxxx10 in the LSB field
                # rhumid   ^= 0x02;
                self.humidity = (100 * float(rhumid))/65535.0
                if (self.humidity < 0):
                    #due to RH accuracy, measured value might be
                    # slightly less than 0 or more 100
                    self.humidity = 0
                elif (self.humidity > 100):
                    self.humidity = 100
                # Calculates temperature compensated Humidity, %RH
                logging.debug("sht30: Humidity %.2f " % self.humidity)
        except Exception as e:
            logging.exception("sht30: Error reading data:" + str(e))
            self.temp = self.humidity = .0
            return self.reactor.NEVER

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "SHT30 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        self._callback(print_time, self.temp)
        return measured_time + self.report_time

    def _chekCRC8(self,data):
        crc = 0xFF
        for d in data:
            crc ^= d

            for bit in range(0,8):
                crc <<= 1

                if (crc & 0x100):
                    crc ^= SHT30_CRC8_POLYNOMINAL;
        return crc

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
            'humidity': self.humidity,
        }


def load_config(config):
    # Register sensor
    pheater = config.get_printer().lookup_object("heaters")
    pheater.add_sensor_factory('SHT30', SHT30)
