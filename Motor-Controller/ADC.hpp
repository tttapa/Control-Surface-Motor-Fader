#pragma once
#include "Registers.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

#include <Arduino_Helpers.h> // EMA.hpp

#include <AH/Filters/EMA.hpp> // EMA filter

template <class Config>
struct ADCManager {
    /// Evenly distribute the analog readings in the control loop period.
    constexpr static uint8_t adc_start_count =
        Config::interrupt_counter / Config::num_faders;
    /// The rate at which we're sampling using the ADC.
    constexpr static float adc_rate = Config::interrupt_freq / adc_start_count;
    // Check that this doesn't take more time than the 13 ADC clock cycles it
    // takes to actually do the conversion. Use 14 instead of 13 just to be safe.
    static_assert(adc_rate <= Config::adc_clock_freq / 14, "ADC too slow");

    /// Enable the ADC with Vcc reference, with the given prescaler, auto
    /// trigger disabled, ADC interrupt enabled.
    /// Called from main program, with interrupts enabled.
    void begin();

    /// Start an ADC conversion on the given channel.
    /// Called inside an ISR.
    void startConversion(uint8_t channel);

    /// Start an ADC conversion at the right intervals.
    /// @param  counter
    ///         Counter that keeps track of how many times the timer interrupt
    ///         fired, between 0 and Config::interrupt_counter - 1.
    /// Called inside an ISR.
    void update(uint8_t counter);

    /// Read the latest ADC result.
    /// Called inside an ISR.
    void complete();

    /// Get the latest ADC reading for the given index.
    /// Called from main program, with interrupts enabled.
    int16_t read(uint8_t idx);
    /// Get the latest filtered ADC reading for the given index.
    /// Called from main program, with interrupts enabled.
    int16_t readFiltered(uint8_t idx);

    /// Write the ADC reading for the given index.
    /// Called from main program, with interrupts enabled.
    void write(uint8_t idx, int16_t val);
    /// Write the filtered ADC reading for the given index.
    /// Called only before ADC interrupts are enabled.
    void writeFiltered(uint8_t idx, int16_t val);

    /// Convert the channel index between 0 and Config::num_faders - 1 to the
    /// actual ADC multiplexer address.
    constexpr static inline uint8_t
    channel_index_to_mux_address(uint8_t adc_mux_idx) {
        return Config::use_A6_A7
                   ? (adc_mux_idx < 2 ? adc_mux_idx : adc_mux_idx + 4)
                   : adc_mux_idx;
    }

    /// Index of the ADC channel currently being read.
    uint8_t channel_index = Config::num_faders;
    /// Latest ADC reading of each fader (updated in ADC ISR). Used for the
    /// control loop.
    volatile int16_t readings[Config::num_faders];
    /// Filters for ADC readings.
    EMA<Config::adc_ema_K, uint16_t> filters[Config::num_faders];
    /// Filtered ADC readings. Used to output over MIDI.
    volatile uint16_t filtered_readings[Config::num_faders];
};

template <class Config>
inline void ADCManager<Config>::begin() {
    constexpr auto prescaler = factorToADCPrescaler(Config::adc_prescaler_fac);
    static_assert(prescaler != ADCPrescaler::Invalid, "Invalid prescaler");

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        cbi(ADCSRA, ADEN); // Disable ADC

        cbi(ADMUX, REFS1); // Vcc reference
        sbi(ADMUX, REFS0); // Vcc reference

        cbi(ADMUX, ADLAR); // 8 least significant bits in ADCL

        setADCPrescaler(prescaler);

        cbi(ADCSRA, ADATE); // Auto trigger disable
        sbi(ADCSRA, ADIE);  // ADC Interrupt Enable
        sbi(ADCSRA, ADEN);  // Enable ADC
    }
}

template <class Config>
inline void ADCManager<Config>::update(uint8_t counter) {
    if (Config::num_faders > 0 && counter == 0 * adc_start_count)
        startConversion(0);
    else if (Config::num_faders > 1 && counter == 1 * adc_start_count)
        startConversion(1);
    else if (Config::num_faders > 2 && counter == 2 * adc_start_count)
        startConversion(2);
    else if (Config::num_faders > 3 && counter == 3 * adc_start_count)
        startConversion(3);
}

template <class Config>
inline void ADCManager<Config>::startConversion(uint8_t channel) {
    channel_index = channel;
    ADMUX &= 0xF0;
    ADMUX |= channel_index_to_mux_address(channel);
    sbi(ADCSRA, ADSC); // ADC Start Conversion
}

template <class Config>
inline void ADCManager<Config>::complete() {
    if (Config::enable_overrun_indicator && readings[channel_index] >= 0)
        sbi(PORTB, 5);    // Set overrun indicator
    uint16_t value = ADC; // Store ADC reading
    readings[channel_index] = value;
    // Filter the reading
    auto &filter = filters[channel_index];
    filtered_readings[channel_index] = filter(value << (6 - Config::adc_ema_K));
}

template <class Config>
inline int16_t ADCManager<Config>::read(uint8_t idx) {
    int16_t v;
    ATOMIC_BLOCK(ATOMIC_FORCEON) { v = readings[idx]; }
    return v;
}

template <class Config>
inline void ADCManager<Config>::write(uint8_t idx, int16_t val) {
    ATOMIC_BLOCK(ATOMIC_FORCEON) { readings[idx] = val; }
}

template <class Config>
inline int16_t ADCManager<Config>::readFiltered(uint8_t idx) {
    int16_t v;
    ATOMIC_BLOCK(ATOMIC_FORCEON) { v = filtered_readings[idx]; }
    return v;
}

template <class Config>
inline void ADCManager<Config>::writeFiltered(uint8_t idx, int16_t val) {
    filters[idx].reset(val);
    filtered_readings[idx] = val;
}
