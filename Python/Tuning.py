"""
This script allows you to write a list of tuning parameter, and carry out 
experiments with them, logging the results and plotting them so you can compare
different tunings.
Results are cached in a file, so only the tunings that changed cause the 
experiments to be run on the Arduino if you re-run the script.

Be sure to set `Config::serial_control = true`, 
`Config::test_reference = false` and `Config::print_controller_signals = false` 
in the Arduino code.
"""

import struct
from numpy import genfromtxt
import matplotlib.pyplot as plt
from sys import path
from serial import Serial
from time import sleep
from os.path import isfile
from os import chdir, makedirs
from SLIP import read_slip, write_slip

# List of tuning parameters to compare, each element of the list is a tuple 
# containing (Kp, Ki, Kd, fc)
tunings = [
    (7.0, 320, 0.05, 60),  # p-on-m 0.75
    (8.55, 440, 0.043, 70),  # p-on-m 0.75
    (8.55, 440, 0.050, 70),  # p-on-m 0.75
    (8.55, 440, 0.053, 70),  # p-on-m 0.75
    (8.55, 440, 0.055, 70),  # p-on-m 0.75
]

speed_div = 8.
fader_idx = 0
sma = False
imgname = 'img.svg'

def get_tuning_name(tuning: tuple((float, float, float, float))):
    name = ''
    for i in range(0, 4):
        setting = ('p', 'i', 'd', 'c')[i]
        set_k = setting + str(tuning[i])
        name += set_k
    name += 's' + str(speed_div)
    if sma: name += '-sma'
    if len(tuning) > 4: name += tuning[4]
    return name

def get_readable_name(tuning: tuple((float, float, float, float))):
    name = '$K_p = {:.2f}$,     $K_i = {:.2f}$,     $K_d = {:.4f}$,     $f_c = {:.2f}$'.format(*tuning)
    if sma: name += ' (SMA)'
    return name

def set_tuning(ser: Serial, tuning: tuple((float, float, float, float))):
    for i in range(0, 4):
        setting = ('p', 'i', 'd', 'c')[i]
        set_k = (setting + str(fader_idx)).encode()
        set_k += struct.pack('<f', tuning[i])
        write_slip(ser, set_k)
        sleep(0.01)

def start(ser: Serial):
    msg = b's0' + struct.pack('<f', speed_div)
    write_slip(ser, msg)

chdir(path[0])
makedirs('data', exist_ok=True)
fig, axs = plt.subplots(len(tunings), 1, sharex='all', sharey='all', figsize=(12, 8), squeeze=False)
with Serial('/dev/ttyUSB0', 1_000_000, timeout=0.5) as ser:
    for i, tuning in enumerate(tunings):
        print(get_readable_name(tuning))
        filename = 'data/data' + get_tuning_name(tuning) + '.tsv'
        print(filename)
        if not isfile(filename):
            with open(filename, 'w') as f:
                ser.reset_input_buffer()
                if i != 0:
                    set_tuning(ser, (2, 10, 0.028, 40))
                    read_slip(ser)
                sleep(2.1) # allow the bootloader to finish
                set_tuning(ser, tuning)
                read_slip(ser)
                ser.reset_input_buffer()
                start(ser)
                while True:
                    data = read_slip(ser)
                    if data is None:
                        break
                    f.write('\t'.join(map(str, data)) + '\n')
                print()

        data = genfromtxt(filename, delimiter='\t')
        axs[i][0].plot(data, linewidth=1)
        axs[i][0].axhline(0, linewidth=0.5, color='k')
        axs[i][0].set_title(get_readable_name(tuning))

plt.xlim(left=246 * speed_div)
plt.xlim(right=len(data) - 200 * speed_div)
plt.tight_layout()
plt.savefig(imgname)
plt.show()
