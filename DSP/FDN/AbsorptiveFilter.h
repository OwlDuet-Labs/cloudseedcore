/*
AbsorptiveFilter: Frequency-Dependent Decay Filter (Jot's Method)

Implements 1st-order IIR lowpass per delay line to create natural high-frequency
absorption matching real acoustic spaces.

Academic Reference: Jot (1991) Section 3.5.3

ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-absorptive>
*/

#pragma once

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-absorptive>
/**
 * AbsorptiveFilter: Frequency-Dependent Decay
 *
 * Transfer Function:
 *   h(z) = k × (1 - β) / (1 - β × z^(-1))
 *
 * Where:
 *   - k: Energy preservation factor (k = sqrt(1 - β²))
 *   - β: Pole location (determines cutoff frequency)
 *
 * Frequency Response:
 *   - DC (ω=0): Unity gain (no LF attenuation)
 *   - HF (ω=π): Attenuation (natural rolloff)
 *   - Cutoff: User-specified frequency where absorption begins
 *   - Slope: -6 dB/octave above cutoff
 *
 * Signal Flow:
 *   DelayLine Output → Modulation → EQ → **Absorptive Filter** → Hadamard Matrix
 */
class AbsorptiveFilter
{
public:
    AbsorptiveFilter();
    ~AbsorptiveFilter() = default;

    // Lifecycle
    void prepare(double sampleRate);
    void reset();

    // Parameter setters
    void setParameters(float cutoffHz, float amount01);

    // Processing
    float process(float input);

    // Query
    float getCutoffHz() const { return cutoffHz_; }
    float getAmount() const { return amount_; }

private:
    // Filter state
    float z1_;  // Previous output sample y[n-1]

    // Filter coefficients
    float k_;    // Energy preservation factor
    float beta_; // Pole location

    // Parameters
    float cutoffHz_;
    float amount_;

    double sampleRate_;
};

} // namespace FDN
} // namespace Cloudseed
