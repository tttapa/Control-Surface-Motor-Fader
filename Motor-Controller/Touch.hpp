#pragma once
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

template <class Config>
struct TouchSense {

    /// The actual threshold as a number of interrupts instead of seconds:
    static constexpr uint8_t touch_sense_thres =
        Config::interrupt_freq * Config::touch_rc_time_threshold;
    /// Ignore mains noise by making the “touched” status stick for longer than
    /// the mains period:
    static constexpr float period_50Hz = 1. / 50;
    /// Keep the “touched” status active for this many periods (see below):
    static constexpr uint8_t touch_sense_stickiness =
        Config::interrupt_freq * period_50Hz * 4 / Config::interrupt_counter;
    /// Check that the threshold is smaller than the control loop period:
    static_assert(touch_sense_thres < Config::interrupt_counter,
                  "Touch sense threshold too high");

    /// The combined bit mask for all touch GPIO pins.
    static constexpr uint8_t gpio_mask =
        (Config::num_faders > 0 ? Config::touch_masks[0] : 0) |
        (Config::num_faders > 1 ? Config::touch_masks[1] : 0) |
        (Config::num_faders > 2 ? Config::touch_masks[2] : 0) |
        (Config::num_faders > 3 ? Config::touch_masks[3] : 0);

    /// Initialize the GPIO pins for capacitive sensing.
    /// Called from main program, with interrupts enabled.
    void begin();

    /// Check which touch sensing knobs are being touched.
    /// @param  counter
    ///         Counter that keeps track of how many times the timer interrupt
    ///         fired, between 0 and Config::interrupt_counter - 1.
    /// Called inside an ISR.
    void update(uint8_t counter);

    /// Get the touch status for the given index.
    /// Called from main program, with interrupts enabled.
    bool read(uint8_t idx);

    // Timers to take into account the stickiness.
    uint8_t touch_timers[Config::num_faders] {};
    // Whether the knobs are being touched.
    volatile bool touched[Config::num_faders];
};

template <class Config>
void TouchSense<Config>::begin() {
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        PORTB &= ~gpio_mask; // low
        DDRB |= gpio_mask;   // output mode
    }
}

// 0. The pin mode is “output”, the value is “low”.
// 1. Set the pin mode to “input”, touch_timer = 0.
// 2. The pin will start charging through the external pull-up resistor.
// 3. After a fixed amount of time, check whether the pin became “high”:
//    if this is the case, the RC-time of the knob/pull-up resistor circuit
//    was smaller than the given threshold. Since R is fixed, this can be used
//    to infer C, the capacitance of the knob: if the capacitance is lower than
//    the threshold (i.e. RC-time is lower), this means the knob was not touched.
// 5. Set the pin mode to “output”, to start discharging the pin to 0V again.
// 6. Some time later, the pin has discharged, so switch to “input” mode and
//    start charging again for the next RC-time measurement.
//
// The “touched” status is sticky: it will remain set for at least
// touch_sense_stickiness ticks. If the pin never resulted in another “touched”
// measurement during that period, the “touched” status for that pin is cleared.

template <class Config>
void TouchSense<Config>::update(uint8_t counter) {
    if (counter == 0) {
        DDRB &= ~gpio_mask; // input mode, start charging
    } else if (counter == touch_sense_thres) {
        uint8_t touched_bits = PINB;
        DDRB |= gpio_mask; // output mode, start discharging
        for (uint8_t i = 0; i < Config::num_faders; ++i) {
            bool touch_i = (touched_bits & Config::touch_masks[i]) == 0;
            if (touch_i) {
                touch_timers[i] = touch_sense_stickiness;
                touched[i] = true;
            } else if (touch_timers[i] > 0) {
                --touch_timers[i];
                if (touch_timers[i] == 0) touched[i] = false;
            }
        }
    }
}

template <class Config>
bool TouchSense<Config>::read(uint8_t idx) {
    bool t;
    ATOMIC_BLOCK(ATOMIC_FORCEON) { t = touched[idx]; }
    return t;
}