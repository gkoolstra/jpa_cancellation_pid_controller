import numpy as np
import serial
import time
import visdom
from tabulate import tabulate
from tqdm import tqdm, trange

class ArduinoPID:
    def __init__(self, port='COM4', baud_rate=9600, timeout=1.0):
        self.dev = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout)
        self.term_char = '\r\n'
        self.query_sleep = 0.25
        self.dac_volt_max = 3.3
        self.dac_bit_max = 4096
        self.quant_char_len = 4

        self.plotter = visdom.Visdom()
        time.sleep(2.0)
        print("PID Initialized!")

    def serial_write(self, string):
        self.dev.write(str.encode(string + self.term_char))

    def serial_query(self, query):
        self.serial_write(query)
        time.sleep(self.query_sleep)
        return self.dev.readline().decode().strip()

    def _constrain(self, value, min_val, max_val):
        if min_val <= value <= max_val:
            ret = value
        elif value < min_val:
            ret = min_val
        elif value > max_val:
            ret = max_val
        return ret

    def _bit_to_volt(self, bit):
        volt_value = bit * self.dac_volt_max / self.dac_bit_max
        return self._constrain(volt_value, 0, self.dac_volt_max)

    def _volt_to_bit(self, volt):
        bit_value = int(volt / np.float(self.dac_volt_max) * self.dac_bit_max)
        return self._constrain(bit_value, 0, self.dac_bit_max)

    def set_ramping(self, mode):
        currently_ramping = self.get_ramping_status()
        if (mode and not currently_ramping) or (currently_ramping and not mode):
            self.serial_write('R')

    def set_manual_output(self, value, output=True):
        if output == True:  # We want to turn it on and set a value to the output DAC
            bit_value = self._volt_to_bit(value)
            self.serial_write(f'M{bit_value:04d}')
        elif self.get_manual_status():  # If we want to turn it off, but it's currently on
            self.serial_write('M9999')  # Turn it off
        else:  # We want to turn it off, but it's already off
            pass

    def _set_Kp(self, value):
        if 0 < value < 0.01:
            raise ValueError("Value must be between 0.01 and 9999")
        else:
            self.serial_write(f'P{value:1.2f}'[:self.quant_char_len + 1])

    def _set_Ki(self, value):
        if 0 < value < 0.01:
            raise ValueError("Value must be between 0.01 and 9999")
        else:
            self.serial_write(f'I{value:1.2f}'[:self.quant_char_len + 1])

    def _set_Kd(self, value):
        if 0 < value < 0.01:
            raise ValueError("Value must be between 0.01 and 9999")
        else:
            self.serial_write(f'D{value:1.2f}'[:self.quant_char_len + 1])

    def get_Kp(self):
        return np.float(self.serial_query('G0003'))

    def get_Ki(self):
        return np.float(self.serial_query('G0004'))

    def get_Kd(self):
        return np.float(self.serial_query('G0005'))

    def set_tuning_parameters(self, P=None, I=None, D=None):
        if P is not None: self._set_Kp(P); time.sleep(1.0)
        if I is not None: self._set_Ki(I); time.sleep(1.0)
        if D is not None: self._set_Kd(D); time.sleep(1.0)

    def get_display_mode(self):
        return self.serial_query('G0006')

    def set_display_mode(self, mode='B'):
        self.serial_write()

    def get_input(self):
        return self._bit_to_volt(np.float(self.serial_query('G0001')))

    def get_setpoint(self):
        return self._bit_to_volt(np.float(self.serial_query('G0000')))

    def set_setpoint(self, value):
        bit_value = self._volt_to_bit(value)
        return self.serial_write(f'S{bit_value:1.2f}'[:self.quant_char_len + 1])

    def get_output(self):
        return self._bit_to_volt(np.float(self.serial_query('G0002')))

    def get_cts_dac_value(self):
        return self._bit_to_volt(int(self.serial_query('G0007')))

    def get_ramping_status(self):
        return int(self.serial_query('G0008'))

    def get_manual_status(self):
        return int(self.serial_query('G0009'))

    def test_sequence(self):
        get_function_list = ['get_display_mode',
                             'get_manual_status',
                             'get_ramping_status',
                             'get_cts_dac_value',
                             'get_input',
                             'get_output',
                             'get_setpoint',
                             'get_Kp',
                             'get_Ki',
                             'get_Kd']

        table = list()
        for k in trange(len(get_function_list)):
            table.append([get_function_list[k], getattr(self, get_function_list[k])()])
            time.sleep(0.25)

        headers = ["Function call", "Result"]
        print(tabulate(table, headers=headers, tablefmt='fancy_grid', floatfmt=".4f"))
        pass

    def monitor_controller(self, n_samples=25):
        self.plotter.close('controller')
        setpoint = self.get_setpoint()
        time.sleep(0.25)
        t0 = time.time()
        t = t0

        inputs = list()
        for k in range(n_samples):
            if self.plotter.win_exists('controller'):
                kwargs = {'update': 'append'}
            else:
                kwargs = {}

            inp = self.get_input()

            plot = self.plotter.line(np.array([inp]),
                                     X=np.array([t - t0]),
                                     win='controller',
                                     opts=dict(title="Controller monitor", xlabel='Time (s)', ylabel='Input (V)'),
                                     name='input',
                                     **kwargs)

            plot = self.plotter.line(np.array([setpoint]),
                                     X=np.array([t - t0]),
                                     win='controller',
                                     opts=dict(title="Controller monitor", xlabel='Time (s)', ylabel='Input (V)'),
                                     name='setpoint',
                                     **kwargs)

            inputs.append(inp)
            t = time.time()
            time.sleep(0.2)

        return np.array(inputs)

# if __name__ == "__main__":
#     Arnie = ArduinoPID()
#     Arnie.test_sequence()