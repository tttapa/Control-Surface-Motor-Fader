#pragma once
#include "Registers.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

/// Configure Timer0 in either phase correct or fast PWM mode with the given
/// prescaler, enable output compare B.
inline void setupMotorTimer0(bool phase_correct, Timer0Prescaler prescaler) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setTimer0WGMode(phase_correct ? Timer0WGMode::PWM
                                      : Timer0WGMode::FastPWM);
        setTimer0Prescaler(prescaler);
        sbi(TCCR0A, COM0B1); // Table 14-6, 14-7 Compare Output Mode
        sbi(TCCR0A, COM0A1); // Table 14-6, 14-7 Compare Output Mode
    }
}

/// Configure Timer2 in either phase correct or fast PWM mode with the given
/// prescaler, enable output compare B.
inline void setupMotorTimer2(bool phase_correct, Timer2Prescaler prescaler) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setTimer2WGMode(phase_correct ? Timer2WGMode::PWM
                                      : Timer2WGMode::FastPWM);
        setTimer2Prescaler(prescaler);
        sbi(TCCR2A, COM2B1); // Table 14-6, 14-7 Compare Output Mode
        sbi(TCCR2A, COM2A1); // Table 14-6, 14-7 Compare Output Mode
    }
}

/// Configure the timers for the PWM outputs.
template <class Config>
inline void setupMotorTimers() {
    constexpr auto prescaler0 = factorToTimer0Prescaler(Config::prescaler_fac);
    static_assert(prescaler0 != Timer0Prescaler::Invalid, "Invalid prescaler");
    constexpr auto prescaler2 = factorToTimer2Prescaler(Config::prescaler_fac);
    static_assert(prescaler2 != Timer2Prescaler::Invalid, "Invalid prescaler");

    if (Config::num_faders > 0)
        setupMotorTimer2(Config::phase_correct_pwm, prescaler2);
    if (Config::num_faders > 2)
        setupMotorTimer0(Config::phase_correct_pwm, prescaler0);
}

/// Class for driving up to 4 DC motors using PWM.
template <class Config>
struct Motors {
    void begin();
    template <uint8_t Idx>
    void setSpeed(int16_t speed);

    template <uint8_t Idx>
    void setupGPIO();
    template <uint8_t Idx>
    void forward(uint8_t speed);
    template <uint8_t Idx>
    void backward(uint8_t speed);
};

template <class Config>
inline void Motors<Config>::begin() {
    setupMotorTimers<Config>();

    if (Config::num_faders > 0) {
        sbi(DDRD, 2);
        sbi(DDRD, 3);
    }
    if (Config::num_faders > 1) {
        if (Config::fader_1_A2)
            sbi(DDRC, 2);
        else
            sbi(DDRB, 5);
        sbi(DDRB, 3);
    }
    if (Config::num_faders > 2) {
        sbi(DDRD, 4);
        sbi(DDRD, 5);
    }
    if (Config::num_faders > 3) {
        sbi(DDRD, 7);
        sbi(DDRD, 6);
    }
}

// Fast PWM (Table 14-6):
//   Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode).
// Phase Correct PWM (Table 14-7):
//   Clear OC0B on compare match when up-counting. Set OC0B on compare match
//   when down-counting.
template <class Config>
template <uint8_t Idx>
inline void Motors<Config>::forward(uint8_t speed) {
    if (Idx >= Config::num_faders)
        return;
    else if (Idx == 0)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            cbi(TCCR2A, COM2B0);
            cbi(PORTD, 2);
            OCR2B = speed;
        }
    else if (Idx == 1)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            cbi(TCCR2A, COM2A0);
            if (Config::fader_1_A2)
                cbi(PORTC, 2);
            else
                cbi(PORTB, 5);
            OCR2A = speed;
        }
    else if (Idx == 2)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            cbi(TCCR0A, COM0B0);
            cbi(PORTD, 4);
            OCR0B = speed;
        }
    else if (Idx == 3)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            cbi(TCCR0A, COM0A0);
            cbi(PORTD, 7);
            OCR0A = speed;
        }
}

// Fast PWM (Table 14-6):
//   Set OC0B on Compare Match, clear OC0B at BOTTOM (inverting mode).
// Phase Correct PWM (Table 14-7):
//   Set OC0B on compare match when up-counting. Clear OC0B on compare match
//   when down-counting.
template <class Config>
template <uint8_t Idx>
inline void Motors<Config>::backward(uint8_t speed) {
    if (Idx >= Config::num_faders)
        return;
    else if (Idx == 0)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            sbi(TCCR2A, COM2B0);
            sbi(PORTD, 2);
            OCR2B = speed;
        }
    else if (Idx == 1)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            sbi(TCCR2A, COM2A0);
            if (Config::fader_1_A2)
                sbi(PORTC, 2);
            else
                sbi(PORTB, 5);
            OCR2A = speed;
        }
    else if (Idx == 2)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            sbi(TCCR0A, COM0B0);
            sbi(PORTD, 4);
            OCR0B = speed;
        }
    else if (Idx == 3)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            sbi(TCCR0A, COM0A0);
            sbi(PORTD, 7);
            OCR0A = speed;
        }
}

template <class Config>
template <uint8_t Idx>
inline void Motors<Config>::setSpeed(int16_t speed) {
    if (speed >= 0)
        forward<Idx>(speed);
    else
        backward<Idx>(-speed);
}