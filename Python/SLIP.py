import struct
from serial import Serial

END = b'\300'
ESC = b'\333'
ESC_END = b'\334'
ESC_ESC = b'\335'

# https://datatracker.ietf.org/doc/html/rfc1055
def write_slip(ser: Serial, data: bytes):
    """Encode the data as a SLIP packet and send it over the serial port"""
    print(data)
    slipdata = bytearray(END)
    for d in data:
        if   bytes([d]) == END: slipdata.extend(ESC + ESC_END)
        elif bytes([d]) == ESC: slipdata.extend(ESC + ESC_ESC)
        else: slipdata.append(d)
    slipdata.extend(END)
    ser.write(slipdata)

# https://datatracker.ietf.org/doc/html/rfc1055
def read_slip(ser: Serial):
    """Try reading a SLIP packet of 3 16-bit integers from the serial port"""
    buf = bytearray()
    escape = False
    while True:
        data = ser.read()
        if not data: return None
        elif data == END: 
            try: return struct.unpack("<hhh", buf)
            except: 
                if len(buf) > 1: print(buf)
            buf = bytearray()
            escape = False
        elif data == ESC: escape = True
        else:
            if escape: 
                if data == ESC_END: data = END 
                elif data == ESC_ESC: data = ESC
                escape = False
            buf.extend(data)