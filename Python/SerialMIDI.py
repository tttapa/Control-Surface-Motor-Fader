"""
Simple script to test MIDI over Serial. Simply reads the position and touch
status, and always responds with the current position, so the setpoint is the
same as the position set by the user.

To use:
 - In Motor-Controller/main.cpp, set the `WITH_MIDI` macro to `1`.
 - Set `Config::serial_control` to `false`.
 - Upload Motor-Controller to an Arduino Uno/Nano.
 - Use `pip` to install the `pySerial` package, e.g. using 
   `python3 -m pip install pySerial`.
 - Change the `SER_PORT` constant below to the port of your Arduino.
 - Run this script, e.g. using
   `python3 Python/SerialMIDI.py`.
 - Move the fader and see the messages be printed and the setpoint updated.
"""

SER_PORT = "/dev/ttyUSB0"

from serial import Serial


class MIDIParser:
    """Very basic, only supports note on/off and pitch bend"""

    NAMES = {0x80: "NOTE_OFF", 0x90: "NOTE_ON", 0xE0: "PITCH_BEND"}

    def __init__(self):
        self.type, self.channel, self.data1, self.data2 = 0, 0, 0, 0
        self.third_byte = False

    def __call__(self, b: int) -> bool:
        if b & 0x80:
            self.type = b & 0xF0
            self.channel = (b & 0x0F) + 1
            self.third_byte = False
        elif self.third_byte:
            self.data2 = b
            self.third_byte = False
            return self.type in self.NAMES
        else:
            self.data1 = b
            self.third_byte = True
        return False


def handle_midi(ser: Serial, parser: MIDIParser):
    name = parser.NAMES[parser.type]
    if name == "PITCH_BEND":
        value = parser.data1 | (parser.data2 << 7)
        print(f"{name}: {parser.channel}, " f"{value}")
        # Echo back to change the setpoint
        msg = [parser.type | (parser.channel - 1), parser.data1, parser.data2]
        ser.write(bytes(msg))
    else:
        print(f"{name}: {parser.channel}, " f"{parser.data1}, {parser.data2}")


def read_midi(ser: Serial, parser: MIDIParser):
    data: bytes = ser.read()
    if not data:
        return
    for b in data:
        if parser(b):
            handle_midi(ser, parser)


with Serial(SER_PORT, 1_000_000, timeout=0.5) as ser:
    ser.reset_input_buffer()
    parser = MIDIParser()
    while True:
        read_midi(ser, parser)
