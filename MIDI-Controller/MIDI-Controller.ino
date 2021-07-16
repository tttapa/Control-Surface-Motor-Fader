/*
 * This sketch connects to the motor controller over I²C, and handles the MIDI
 * communication. Pitch bend messages it receives over MIDI will be sent to the
 * motor controller as setpoint changes for the faders, and if the fader is 
 * touched or moved, this sketch sends MIDI messages to indicate these events.
 */

#include <Control_Surface.h>
#include <Wire.h>

const uint8_t mcu_idx = 0;    // MCU track number
const uint8_t fader_idx = 0;  // Index of the fader connected to the slave
const uint8_t slave_addr = 8; // I²C address of slave fader controller
USBDebugMIDI_Interface midi;  // MIDI interface to DAW
PBValue pb {MCU::VOLUME_1 + mcu_idx};   // Receive pitch bend with setpoint
PitchBendSender<10> sender;             // Send pitch bend with actual position
Hysteresis<4, uint16_t, uint16_t> hyst; // Filter the actual position
bool prevTouched = false;               // Whether the knob is being touched
uint16_t prevSetpoint = 0;              // Previous setpoint set by DAW
constexpr bool debug_print = false;     // Print the data from the slave

void setup() {
    Wire.begin();                          // Join the I²C bus
    Wire.setClock(400000);                 // Set higher I²C speed
    if (debug_print) Serial.begin(115200); // Initialize the Serial port
    Control_Surface.begin();               // Initialize everything else
}

void readFromSlave() {
    // Request the position from the slave over I²C
    uint8_t maxlen = 1 + (1 + fader_idx) * 2;
    Wire.requestFrom(slave_addr, maxlen);
    uint8_t buf[maxlen];
    uint8_t i = 0;
    while (Wire.available() && i < maxlen)
        buf[i++] = Wire.read();
    if (debug_print) Serial << AH::HexDump(buf, i) << endl;
    // If we received the “touch” byte
    if (i >= 1) {
        bool touched = buf[0] & (1 << fader_idx);
        const MIDIAddress addr = MCU::FADER_TOUCH_1 + mcu_idx;
        if (touched != prevTouched) {
            touched ? Control_Surface.sendNoteOn(addr, 127)
                    : Control_Surface.sendNoteOff(addr, 127);
            prevTouched = touched;
        }
    }
    // If we received the position and if the fader is being touched
    if (i >= 1 + (1 + fader_idx) * 2 && prevTouched) {
        uint16_t faderpos;
        memcpy(&faderpos, &buf[1 + fader_idx * 2], 2);
        // Send the fader position over MIDI if it changed
        if (hyst.update(faderpos))
            sender.send(hyst.getValue(), MCU::VOLUME_1 + mcu_idx);
    }
}

void sendToSlave() {
    uint16_t setpoint = pb.getValue() >> 4; // 14-bit → 10-bit
    // If the setpoint changed
    if (setpoint != prevSetpoint) {
        // Send it to the slave over I²C
        uint16_t idx = fader_idx;
        uint16_t data = setpoint;
        data |= idx << 12;
        Wire.beginTransmission(slave_addr);
        Wire.write(reinterpret_cast<const uint8_t *>(&data), 2);
        Wire.endTransmission();
        prevSetpoint = setpoint;
    }
}

void loop() {
    Control_Surface.loop();
    static Timer<millis> timer = 5;
    // Don't update at full speed, just every couple of milliseconds
    if (timer) {
        readFromSlave();
        sendToSlave();
    }
}
