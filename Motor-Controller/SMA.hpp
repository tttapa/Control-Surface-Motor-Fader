#pragma once

#include <Arduino_Helpers.h>

#include <AH/Math/Divide.hpp>
#include <AH/STL/algorithm> // std::fill
#include <AH/STL/cstdint>

/** 
 * @brief   Simple Moving Average filter.
 * 
 * Returns the average of the N most recent input values.
 * 
 * @f[
 * y[n] = \frac{1}{N} \sum_{i=0}^{N-1}x[n-i]
 * @f]
 * 
 * @see     https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Simple%20Moving%20Average/Simple-Moving-Average.html
 * 
 * @tparam  N
 *          The number of samples to average.
 * @tparam  input_t
 *          The type of the input (and output) of the filter.
 * @tparam  sum_t
 *          The type to use for the accumulator, must be large enough to fit
 *          N times the maximum input value.
 */
template <uint8_t N, class input_t = uint16_t, class sum_t = uint32_t>
class SMA {
  public:
    /// Default constructor (initial state is initialized to all zeros).
    SMA() = default;

    /// Constructor (initial state is initialized to given value).
    ///
    /// @param  initialValue
    ///         Determines the initial state of the filter:
    ///         @f$ x[-N] =\ \ldots\ = x[-2] = x[-1] = \text{initialValue} @f$
    ///
    SMA(input_t initialValue) : sum(N * (sum_t)initialValue) {
        std::fill(std::begin(previousInputs), std::end(previousInputs),
                  initialValue);
    }

    /// Update the internal state with the new input @f$ x[n] @f$ and return the
    /// new output @f$ y[n] @f$.
    ///
    /// @param  input
    ///         The new input @f$ x[n] @f$.
    /// @return The new output @f$ y[n] @f$.
    input_t operator()(input_t input) {
        sum -= previousInputs[index];
        sum += input;
        previousInputs[index] = input;
        if (++index == N) index = 0;
        return AH::round_div<N>(sum);
    }

  private:
    uint8_t index = 0;
    input_t previousInputs[N] {};
    sum_t sum = 0;
};
