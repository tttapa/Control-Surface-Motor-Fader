// Configuration and initialization of the analog-to-digital converter:
#include "ADC.hpp"
// Capacitive touch sensing:
#include "Touch.hpp"
// PID controller:
#include "Controller.hpp"
// Configuration of PWM and Timer2/0 for driving the motor:
#include "Motor.hpp"
// Reference signal for testing the performance of the controller:
#include "Reference.hpp"
// Helpers for low-level AVR Timer2/0 and ADC registers:
#include "Registers.hpp"
// Parsing incoming messages over Serial using SLIP packets:
#include "SerialSLIP.hpp"

#include <Arduino.h>         // setup, loop, analogRead
#include <Arduino_Helpers.h> // EMA.hpp
#include <Wire.h>            // I²C slave

#include "SMA.hpp"            // SMA filter
#include <AH/Filters/EMA.hpp> // EMA filter

// ------------------------------ Description ------------------------------- //

// This sketch drives up to four motorized faders using a PID controller. The
// motor is disabled when the user touches the knob of the fader.
//
// Everything is driven by Timer2, which runs (by default) at a rate of
// 31.250 kHz. This high rate is used to eliminate audible tones from the PWM
// drive for the motor. Timer0 is used for the PWM outputs of faders 3 and 4.
// Every 30 periods of Timer2 (960 µs), each analog input is sampled, and
// this causes the PID control loop to run in the main loop function.
// Capacitive sensing is implemented by measuring the RC time on the touch pin
// in the Timer2 interrupt handler. The “touched” status is sticky for >20 ms
// to prevent interference from the 50 Hz mains.
//
// There are options to (1) follow a test reference (with ramps and jumps), (2)
// to receive a target position over I²C, or (3) to run experiments based on
// commands received over the serial port. The latter is used by a Python script
// that performs experiments with different tuning parameters for the
// controllers.

// -------------------------------- Hardware -------------------------------- //

// Fader 0:
//  - A0:  wiper of the potentiometer          (ADC0)
//  - D8:  touch pin of the knob               (PB0)
//  - D2:  input 1A of L293D dual H-bridge 1   (PD2)
//  - D3:  input 2A of L293D dual H-bridge 1   (OC2B)
//
// Fader 1:
//  - A1:  wiper of the potentiometer          (ADC1)
//  - D9:  touch pin of the knob               (PB1)
//  - D7:  input 3A of L293D dual H-bridge 1   (PD7)
//  - D11: input 4A of L293D dual H-bridge 1   (OC2A)
//
// Fader 2:
//  - A2:  wiper of the potentiometer          (ADC2)
//  - D10: touch pin of the knob               (PB2)
//  - D4:  input 1A of L293D dual H-bridge 2   (PD4)
//  - D5:  input 2A of L293D dual H-bridge 2   (OC0B)
//
// Fader 3:
//  - A3:  wiper of the potentiometer          (ADC3)
//  - D12: touch pin of the knob               (PB4)
//  - D13: input 3A of L293D dual H-bridge 2   (PB5)
//  - D6:  input 4A of L293D dual H-bridge 2   (OC0A)
//
// If fader 1 is unused:
//  - D13: LED or scope as overrun indicator   (PB5)
//
// For communication:
//  - D0:  UART TX                             (TXD)
//  - D1:  UART RX                             (RXD)
//  - A4:  I²C data                            (SDA)
//  - A5:  I²C clock                           (SCL)
//
// Connect the outer connections of the potentiometers to ground and Vcc, it's
// recommended to add a 100 nF capacitor between each wiper and ground.
// Connect the 1,2EN and 3,4EN enable pins of the L293D chips to Vcc.
// Connect a 500kΩ pull-up resistor between each touch pin and Vcc.
// On an Arduino Nano, you can set an option to use pins A6/A7 instead of A2/A3.
// Note that D13 is often pulsed by the bootloader, which might cause the fader
// to move when resetting the Arduino. You can either disable this behavior in
// the bootloader, or use a different pin (e.g. A2 or A3 on an Arduino Nano).
// The overrun indicator is only enabled if the number of faders is less than 4,
// because it conflicts with the motor driver pin of Fader 1. You can choose a
// different pin instead.

// ----------------------------- Configuration ------------------------------ //

// Enable MIDI input/output.
#define WITH_MIDI 0
// Print to the Serial monitor instead of sending actual MIDI messages.
#define MIDI_DEBUG 0

struct Config {
    // Print the control loop and interrupt frequencies to Serial at startup:
    static constexpr bool print_frequencies = true;
    // Print the setpoint, actual position and control signal to Serial.
    // Note that this slows down the control loop significantly, it probably
    // won't work if you are using more than one fader without increasing
    // `interrupt_divisor`:
    static constexpr bool print_controller_signals = false;
    static constexpr uint8_t controller_to_print = 0;
    // Follow the test reference trajectory (true) or receive the target
    // position over I²C or Serial (false):
    static constexpr bool test_reference = false;
    // Increase this divisor to slow down the test reference:
    static constexpr uint8_t test_reference_speed_div = 4;
    // Allow control for tuning and starting experiments over Serial:
    static constexpr bool serial_control = true;
    // I²C slave address (zero to disable I²C):
    static constexpr uint8_t i2c_address = 8;
    // The baud rate to use for the Serial interface (e.g. for MIDI_DEBUG,
    // print_controller_signals, serial_control, etc.)
    static constexpr uint32_t serial_baud_rate = 1000000;
    // The baud rate to use for MIDI over Serial.
    // Use 31'250 for MIDI over 5-pin DIN, HIDUINO/USBMidiKliK.
    // Hairless MIDI uses 115'200 by default.
    // The included python/SerialMIDI.py script uses 1'000'000.
    static constexpr uint32_t midi_baud_rate = serial_baud_rate;

    // Number of faders, must be between 1 and 4:
    static constexpr size_t num_faders = 1;
    // Actually drive the motors. If set to false, runs all code as normal, but
    // doesn't turn on the motors.
    static constexpr bool enable_controller = true;
    // Use analog pins (A0, A1, A6, A7) instead of (A0, A1, A2, A3), useful for
    // saving digital pins on an Arduino Nano:
    static constexpr bool use_A6_A7 = false;
    // Use pin A2 instead of D13 as the motor driver pin for the fourth fader.
    // Allows D13 to be used as overrun indicator, and avoids issues with the
    // bootloader blinking the LED.
    // Can only be used if `use_A6_A7` is set to true.
    static constexpr bool fader_3_A2 = false;
    // Change the setpoint to the current position when touching the knob.
    // Useful if your DAW does not send any feedback when manually moving the
    // fader.
    static constexpr bool touch_to_current_position = false;

    // Capacitive touch sensing RC time threshold.
    // Increase this time constant if the capacitive touch sense is too
    // sensitive or decrease it if it's not sensitive enough:
    static constexpr float touch_rc_time_threshold = 150e-6; // seconds
    // Bit masks of the touch pins (must be on port B):
    static constexpr uint8_t touch_masks[] = {1 << PB0, 1 << PB1, 1 << PB2,
                                              1 << PB4};

    // Use phase-correct PWM (true) or fast PWM (false), this determines the
    // timer interrupt frequency, prefer phase-correct PWM with prescaler 1 on
    // 16 MHz boards, and fast PWM with prescaler 1 on 8 MHz boards, both result
    // in a PWM and interrupt frequency of 31.250 kHz
    // (fast PWM is twice as fast):
    static constexpr bool phase_correct_pwm = true;
    // The fader position will be sampled once per `interrupt_divisor` timer
    // interrupts, this determines the sampling frequency of the control loop.
    // Some examples include 20 → 320 µs, 30 → 480 µs, 60 → 960 µs,
    // 90 → 1,440 µs, 124 → 2,016 µs, 188 → 3,008 µs, 250 → 4,000 µs.
    // 60 is the default, because it works with four faders. If you only use
    // a single fader, you can go as low as 20 because you only need a quarter
    // of the computations and ADC time:
    static constexpr uint8_t interrupt_divisor = 60 / (1 + phase_correct_pwm);
    // The prescaler for the timer, affects PWM and control loop frequencies:
    static constexpr unsigned prescaler_fac = 1;
    // The prescaler for the ADC, affects speed of analog readings:
    static constexpr uint8_t adc_prescaler_fac = 64;

    // Turn off the motor after this many seconds of inactivity:
    static constexpr float timeout = 2;

    // EMA filter factor for fader position filters:
    static constexpr uint8_t adc_ema_K = 2;
    // SMA filter length for setpoint filters, improves tracking of ramps if the
    // setpoint changes in steps (e.g. when the DAW only updates the reference
    // every 20 ms). Powers of two are significantly faster (e.g. 32 works well):
    static constexpr uint8_t setpoint_sma_length = 0;

    // ------------------------ Computed Quantities ------------------------- //

    // Sampling time of control loop:
    constexpr static float Ts = 1. * prescaler_fac * interrupt_divisor * 256 *
                                (1 + phase_correct_pwm) / F_CPU;
    // Frequency at which the interrupt fires:
    constexpr static float interrupt_freq =
        1. * F_CPU / prescaler_fac / 256 / (1 + phase_correct_pwm);
    // Clock speed of the ADC:
    constexpr static float adc_clock_freq = 1. * F_CPU / adc_prescaler_fac;
    // Pulse pin D13 if the control loop took too long:
    constexpr static bool enable_overrun_indicator =
        num_faders < 4 || fader_3_A2;

    static_assert(0 < num_faders && num_faders <= 4,
                  "At most four faders supported");
    static_assert(use_A6_A7 || !fader_3_A2,
                  "Cannot use A2 for motor driver "
                  "and analog input at the same time");
    static_assert(!WITH_MIDI || !serial_control,
                  "Cannot use MIDI and Serial control at the same time");
    static_assert(!WITH_MIDI || !print_controller_signals,
                  "Cannot use MIDI while printing controller signals");
};
constexpr uint8_t Config::touch_masks[];
constexpr float Ts = Config::Ts;

// ----------------- ADC, Capacitive Touch State and Motors ----------------- //

ADCManager<Config> adc;
TouchSense<Config> touch;
Motors<Config> motors;

// ------------------------ Setpoints and References ------------------------ //

// Setpoints (target positions) for all faders:
Reference<Config> references[Config::num_faders];

// ------------------------------ Controllers ------------------------------- //

// The main PID controllers. Need tuning for your specific setup:

PID controllers[] {
    // This is an example of a controller with very little overshoot
    {
        6,     // Kp: proportional gain
        2,     // Ki: integral gain
        0.035, // Kd: derivative gain
        Ts,    // Ts: sampling time
        60, // fc: cutoff frequency of derivative filter (Hz), zero to disable
    },
    // This one has more overshoot, but less ramp tracking error
    {
        4,     // Kp: proportional gain
        11,    // Ki: integral gain
        0.028, // Kd: derivative gain
        Ts,    // Ts: sampling time
        40, // fc: cutoff frequency of derivative filter (Hz), zero to disable
    },
    // This is a very aggressive controller
    {
        8.55,  // Kp: proportional gain
        440,   // Ki: integral gain
        0.043, // Kd: derivative gain
        Ts,    // Ts: sampling time
        70, // fc: cutoff frequency of derivative filter (Hz), zero to disable
    },
    // Fourth controller
    {
        6,     // Kp: proportional gain
        2,     // Ki: integral gain
        0.035, // Kd: derivative gain
        Ts,    // Ts: sampling time
        60, // fc: cutoff frequency of derivative filter (Hz), zero to disable
    },
};

// ---------------------------------- MIDI ---------------------------------- //

#if WITH_MIDI
#include <Control_Surface.h>

#if MIDI_DEBUG
USBDebugMIDI_Interface midi {Config::serial_baud_rate};
#else
HardwareSerialMIDI_Interface midi {Serial, Config::midi_baud_rate};
#endif

template <uint8_t Idx>
void sendMIDIMessages(bool touched) {
    // Don't send if the UART buffer is (almost) full
    if (Serial.availableForWrite() < 6) return;
    // Touch
    static bool prevTouched = false; // Whether the knob is being touched
    if (touched != prevTouched) {
        MIDIAddress addr(MCU::FADER_TOUCH_1, Channel(Idx)); // Send note on/off on correct channel
        touched ? midi.sendNoteOn(addr, 127) : midi.sendNoteOff(addr, 127);
        prevTouched = touched;
    }
    // Position
    static Hysteresis<6 - Config::adc_ema_K, uint16_t, uint16_t> hyst;
    if (prevTouched && hyst.update(adc.readFiltered(Idx))) {
        auto value = AH::increaseBitDepth<14, 10, uint16_t>(hyst.getValue());
        midi.sendPitchBend(MCU::VOLUME_1 + Idx, value);
    }
}

void updateMIDISetpoint(ChannelMessage msg) {
    auto type = msg.getMessageType();
    auto channel = msg.getChannel().getRaw();
    if (type == MIDIMessageType::PITCH_BEND && channel < Config::num_faders)
        references[channel].setMasterSetpoint(msg.getData14bit() >> 4);
}

void initMIDI() { midi.begin(); }

void updateMIDI() {
    while (1) {
        auto evt = midi.read();
        if (evt == MIDIReadEvent::NO_MESSAGE)
            break;
        else if (evt == MIDIReadEvent::CHANNEL_MESSAGE)
            updateMIDISetpoint(midi.getChannelMessage());
    }
}

#endif

// ---------------- Printing all signals for serial plotter ----------------- //

template <uint8_t Idx>
void printControllerSignals(int16_t setpoint, int16_t adcval, int16_t control) {
    // Send (binary) controller signals over Serial to plot in Python
    if (Config::serial_control && references[Idx].experimentInProgress()) {
        const int16_t data[3] {setpoint, adcval, control};
        SLIPSender(Serial).writePacket(reinterpret_cast<const uint8_t *>(data),
                                       sizeof(data));
    }
    // Print signals as text
    else if (Config::print_controller_signals &&
             Idx == Config::controller_to_print) {
        Serial.print(setpoint);
        Serial.print('\t');
        Serial.print(adcval);
        Serial.print('\t');
        Serial.print((control + 256) * 2);
        Serial.println();
    }
}

// ----------------------------- Control logic ------------------------------ //

template <uint8_t Idx>
void updateController(uint16_t setpoint, int16_t adcval, bool touched) {
    auto &controller = controllers[Idx];

    // Prevent the motor from being turned off after being touched
    if (touched) controller.resetActivityCounter();

    // Set the target position
    if (Config::setpoint_sma_length > 0) {
        static SMA<Config::setpoint_sma_length, uint16_t, uint32_t> sma;
        uint16_t filtsetpoint = sma(setpoint);
        controller.setSetpoint(filtsetpoint);
    } else {
        controller.setSetpoint(setpoint);
    }

    // Update the PID controller to get the control action
    int16_t control = controller.update(adcval);

    // Apply the control action to the motor
    if (Config::enable_controller) {
        if (touched) // Turn off motor if knob is touched
            motors.setSpeed<Idx>(0);
        else
            motors.setSpeed<Idx>(control);
    }

    // Change the setpoint as we move
    if (Config::touch_to_current_position && touched)
        references[Idx].setMasterSetpoint(adcval);

#if WITH_MIDI
    sendMIDIMessages<Idx>(touched);
#else
    printControllerSignals<Idx>(controller.getSetpoint(), adcval, control);
#endif
}

template <uint8_t Idx>
void readAndUpdateController() {
    // Read the ADC value for the given fader:
    int16_t adcval = adc.read(Idx);
    // If the ADC value was updated by the ADC interrupt, run the control loop:
    if (adcval >= 0) {
        // Check if the fader knob is touched
        bool touched = touch.read(Idx);
        // Read the target position
        uint16_t setpoint = references[Idx].getNextSetpoint();
        // Run the control loop
        updateController<Idx>(setpoint, adcval, touched);
        // Write -1 so the controller doesn't run again until the next value is
        // available:
        adc.write(Idx, -1);
        if (Config::enable_overrun_indicator)
            cbi(PORTB, 5); // Clear overrun indicator
    }
}

// ------------------------------ Setup & Loop ------------------------------ //

void onRequest();
void onReceive(int);
void updateSerialIn();

void setup() {
    // Initialize some globals
    for (uint8_t i = 0; i < Config::num_faders; ++i) {
        // all fader positions for the control loop start of as -1 (no reading)
        adc.write(i, -1);
        // reset the filter to the current fader position to prevent transients
        adc.writeFiltered(i, analogRead(adc.channel_index_to_mux_address(i)));
        // after how many seconds of not touching the fader and not changing
        // the reference do we turn off the motor?
        controllers[i].setActivityTimeout(Config::timeout);
    }

    // Configure the hardware
    if (Config::enable_overrun_indicator) sbi(DDRB, 5); // Pin 13 output

#if WITH_MIDI
    initMIDI();
#else
    if (Config::print_frequencies || Config::print_controller_signals ||
        Config::serial_control)
        Serial.begin(Config::serial_baud_rate);
#endif

    adc.begin();
    touch.begin();
    motors.begin();

    // Print information to the serial monitor or legends to the serial plotter
    if (Config::print_frequencies) {
        Serial.println();
        Serial.print(F("Interrupt frequency (Hz): "));
        Serial.println(Config::interrupt_freq);
        Serial.print(F("Controller sampling time (µs): "));
        Serial.println(Config::Ts * 1e6);
        Serial.print(F("ADC clock rate (Hz): "));
        Serial.println(Config::adc_clock_freq);
        Serial.print(F("ADC sampling rate (Sps): "));
        Serial.println(adc.adc_rate);
    }
    if (Config::print_controller_signals) {
        Serial.println();
        Serial.println(F("Reference\tActual\tControl\t-"));
        Serial.println(F("0\t0\t0\t0\r\n0\t0\t0\t1024"));
    }

    // Initalize I²C slave and attach callbacks
    if (Config::i2c_address) {
        Wire.begin(Config::i2c_address);
        Wire.onRequest(onRequest);
        Wire.onReceive(onReceive);
    }

    // Enable Timer2 overflow interrupt, this starts reading the touch sensitive
    // knobs and sampling the ADC, which causes the controllers to run in the
    // main loop
    sbi(TIMSK2, TOIE2);
}

void loop() {
    if (Config::num_faders > 0) readAndUpdateController<0>();
    if (Config::num_faders > 1) readAndUpdateController<1>();
    if (Config::num_faders > 2) readAndUpdateController<2>();
    if (Config::num_faders > 3) readAndUpdateController<3>();
#if WITH_MIDI
    updateMIDI();
#else
    if (Config::serial_control) updateSerialIn();
#endif
}

// ------------------------------- Interrupts ------------------------------- //

// Fires at a constant rate of `interrupt_freq`.
ISR(TIMER2_OVF_vect) {
    // We don't have to take all actions at each interupt, so keep a counter to
    // know when to take what actions.
    static uint8_t counter = 0;

    adc.update(counter);
    touch.update(counter);

    ++counter;
    if (counter == Config::interrupt_divisor) counter = 0;
}

// Fires when the ADC measurement is complete. Stores the reading, both before
// and after filtering (for the controller and for user input respectively).
ISR(ADC_vect) { adc.complete(); }

// ---------------------------------- Wire ---------------------------------- //

// Send the touch status and filtered fader positions to the master.
void onRequest() {
    uint8_t touched = 0;
    for (uint8_t i = 0; i < Config::num_faders; ++i)
        touched |= touch.touched[i] << i;
    Wire.write(touched);
    for (uint8_t i = 0; i < Config::num_faders; ++i) {
        uint16_t filt_read = adc.readFiltered14ISR(i);
        Wire.write(reinterpret_cast<const uint8_t *>(&filt_read), 2);
    }
}

// Change the setpoint of the given fader based on the value in the message
// received from the master.
void onReceive(int count) {
    if (count < 2) return;
    if (Wire.available() < 2) return;
    uint16_t data = Wire.read();
    data |= uint16_t(Wire.read()) << 8;
    uint8_t idx = data >> 12;
    data &= 0x03FF;
    if (idx < Config::num_faders) references[idx].setMasterSetpoint(data);
}

// ---------------------------------- Serial -------------------------------- //

// Read SLIP messages from the serial port that allow dynamically updating the
// tuning of the controllers. This is used by the Python tuning script.
//
// Message format: <command> <fader> <value>
// Commands:
//   - p: proportional gain Kp
//   - i: integral gain Ki
//   - d: derivative gain Kd
//   - c: derivative filter cutoff frequency f_c (Hz)
//   - m: maximum absolute control output
//   - s: start an experiment, using getNextExperimentSetpoint
// Fader index: up to four faders are addressed using the characters '0' - '3'.
// Values: values are sent as 32-bit little Endian floating point numbers.
//
// For example the message 'c0\x00\x00\x20\x42' sets the derivative filter
// cutoff frequency of the first fader to 40.

void updateSerialIn() {
    static SLIPParser parser;
    static char cmd = '\0';
    static uint8_t fader_idx = 0;
    static uint8_t buf[4];
    static_assert(sizeof(buf) == sizeof(float), "");
    // This function is called if a new byte of the message arrives:
    auto on_char_receive = [&](char new_byte, size_t index_in_packet) {
        if (index_in_packet == 0)
            cmd = new_byte;
        else if (index_in_packet == 1)
            fader_idx = new_byte - '0';
        else if (index_in_packet < 6)
            buf[index_in_packet - 2] = new_byte;
    };
    // Convert the 4-byte buffer to a float:
    auto as_f32 = [&] {
        float f;
        memcpy(&f, buf, sizeof(float));
        return f;
    };
    // Read and parse incoming packets from Serial:
    while (Serial.available() > 0) {
        uint8_t c = Serial.read();
        auto msg_size = parser.parse(c, on_char_receive);
        // If a complete message of 6 bytes was received, and if it addresses
        // a valid fader:
        if (msg_size == 6 && fader_idx < Config::num_faders) {
            // Execute the command:
            switch (cmd) {
                case 'p': controllers[fader_idx].setKp(as_f32()); break;
                case 'i': controllers[fader_idx].setKi(as_f32()); break;
                case 'd': controllers[fader_idx].setKd(as_f32()); break;
                case 'c': controllers[fader_idx].setEMACutoff(as_f32()); break;
                case 'm': controllers[fader_idx].setMaxOutput(as_f32()); break;
                case 's':
                    references[fader_idx].startExperiment(as_f32());
                    controllers[fader_idx].resetIntegral();
                    break;
                default: break;
            }
        }
    }
}
