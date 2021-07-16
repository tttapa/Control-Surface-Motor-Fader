#pragma once

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h> // F_CPU

// --------------------------------- Utils ---------------------------------- //

#ifndef ARDUINO // Ensures that my IDE sees the correct frequency
#undef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef sbi
/// Set bit in register.
template <class R>
inline void sbi(R &reg, uint8_t bit) {
    reg |= (1u << bit);
}
#define sbi sbi
#endif
#ifndef cbi
/// Clear bit in register.
template <class R>
inline void cbi(R &reg, uint8_t bit) {
    reg &= ~(1u << bit);
}
#define cbi cbi
#endif
/// Write bit in register.
template <class R>
inline void wbi(R &reg, uint8_t bit, bool value) {
    value ? sbi(reg, bit) : cbi(reg, bit);
}

// --------------------------------- Timer0 --------------------------------- //

/// Timer 0 clock select (Table 14-9).
enum class Timer0Prescaler : uint8_t {
    None = 0b000,
    S1 = 0b001,
    S8 = 0b010,
    S64 = 0b011,
    S256 = 0b100,
    S1024 = 0b101,
    ExtFall = 0b110,
    ExtRise = 0b111,
    Invalid = 0xFF,
};

/// Timer 0 waveform generation mode (Table 14-8).
enum class Timer0WGMode : uint8_t {
    Normal = 0b000,
    PWM = 0b001,
    CTC = 0b010,
    FastPWM = 0b011,
    PWM_OCRA = 0b101,
    FastPWM_OCRA = 0b111,
};

// Convert the prescaler factor to the correct bit pattern to write to the
// TCCR0B register (Table 14-9).
constexpr inline Timer0Prescaler factorToTimer0Prescaler(uint16_t factor) {
    return factor == 1      ? Timer0Prescaler::S1
           : factor == 8    ? Timer0Prescaler::S8
           : factor == 64   ? Timer0Prescaler::S64
           : factor == 256  ? Timer0Prescaler::S256
           : factor == 1024 ? Timer0Prescaler::S1024
                            : Timer0Prescaler::Invalid;
}

/// Set the clock source/prescaler of Timer0 (Table 14-9).
inline void setTimer0Prescaler(Timer0Prescaler ps) {
    if (ps == Timer0Prescaler::Invalid)
        return;
    wbi(TCCR0B, CS02, static_cast<uint8_t>(ps) & (1u << 2));
    wbi(TCCR0B, CS01, static_cast<uint8_t>(ps) & (1u << 1));
    wbi(TCCR0B, CS00, static_cast<uint8_t>(ps) & (1u << 0));
}

/// Set the wavefrom generation mode of Timer0 (Table 14-8).
inline void setTimer0WGMode(Timer0WGMode mode) {
    wbi(TCCR0B, WGM02, static_cast<uint8_t>(mode) & (1u << 2));
    wbi(TCCR0A, WGM01, static_cast<uint8_t>(mode) & (1u << 1));
    wbi(TCCR0A, WGM00, static_cast<uint8_t>(mode) & (1u << 0));
}

// --------------------------------- Timer2 --------------------------------- //

/// Timer 0 clock select (Table 17-9).
enum class Timer2Prescaler : uint8_t {
    None = 0b000,
    S1 = 0b001,
    S8 = 0b010,
    S32 = 0b011,
    S64 = 0b100,
    S128 = 0b101,
    S256 = 0b110,
    S1024 = 0b111,
    Invalid = 0xFF,
};

/// Timer 0 waveform generation mode (Table 17-8).
enum class Timer2WGMode : uint8_t {
    Normal = 0b000,
    PWM = 0b001,
    CTC = 0b010,
    FastPWM = 0b011,
    PWM_OCRA = 0b101,
    FastPWM_OCRA = 0b111,
};

// Convert the prescaler factor to the correct bit pattern to write to the
// TCCR0B register (Table 17-9).
constexpr inline Timer2Prescaler factorToTimer2Prescaler(uint16_t factor) {
    return factor == 1      ? Timer2Prescaler::S1
           : factor == 8    ? Timer2Prescaler::S8
           : factor == 32   ? Timer2Prescaler::S32
           : factor == 64   ? Timer2Prescaler::S64
           : factor == 128  ? Timer2Prescaler::S128
           : factor == 256  ? Timer2Prescaler::S256
           : factor == 1024 ? Timer2Prescaler::S1024
                            : Timer2Prescaler::Invalid;
}

/// Set the clock source/prescaler of Timer2 (Table 17-9).
inline void setTimer2Prescaler(Timer2Prescaler ps) {
    if (ps == Timer2Prescaler::Invalid)
        return;
    wbi(TCCR2B, CS22, static_cast<uint8_t>(ps) & (1u << 2));
    wbi(TCCR2B, CS21, static_cast<uint8_t>(ps) & (1u << 1));
    wbi(TCCR2B, CS20, static_cast<uint8_t>(ps) & (1u << 0));
}

/// Set the wavefrom generation mode of Timer2 (Table 17-8).
inline void setTimer2WGMode(Timer2WGMode mode) {
    wbi(TCCR2B, WGM22, static_cast<uint8_t>(mode) & (1u << 2));
    wbi(TCCR2A, WGM21, static_cast<uint8_t>(mode) & (1u << 1));
    wbi(TCCR2A, WGM20, static_cast<uint8_t>(mode) & (1u << 0));
}

// ---------------------------------- ADC ----------------------------------- //

/// ADC prescaler select (Table 23-5).
enum class ADCPrescaler : uint8_t {
    S2 = 0b000,
    S2_2 = 0b001,
    S4 = 0b010,
    S8 = 0b011,
    S16 = 0b100,
    S32 = 0b101,
    S64 = 0b110,
    S128 = 0b111,
    Invalid = 0xFF,
};

// Convert the prescaler factor to the correct bit pattern to write to the
// ADCSRA register (Table 23-5).
constexpr inline ADCPrescaler factorToADCPrescaler(uint8_t factor) {
    return factor == 2     ? ADCPrescaler::S2_2
           : factor == 4   ? ADCPrescaler::S4
           : factor == 8   ? ADCPrescaler::S8
           : factor == 16  ? ADCPrescaler::S16
           : factor == 32  ? ADCPrescaler::S32
           : factor == 64  ? ADCPrescaler::S64
           : factor == 128 ? ADCPrescaler::S128
                           : ADCPrescaler::Invalid;
}

/// Set the prescaler of the ADC (Table 23-5).
inline void setADCPrescaler(ADCPrescaler ps) {
    if (ps == ADCPrescaler::Invalid)
        return;
    wbi(ADCSRA, ADPS2, static_cast<uint8_t>(ps) & (1u << 2));
    wbi(ADCSRA, ADPS1, static_cast<uint8_t>(ps) & (1u << 1));
    wbi(ADCSRA, ADPS0, static_cast<uint8_t>(ps) & (1u << 0));
}
