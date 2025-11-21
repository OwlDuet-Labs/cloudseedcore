/*
CorrectionFilter: Automatic Frequency Response Compensation

Inverts absorptive filter's frequency response to ensure flat output spectrum
regardless of RT60 and absorption settings.

Academic Reference: Jot (1991) Section 3.5.4

ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-correction>
*/

#pragma once

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-correction>
/**
 * CorrectionFilter: Tonal Balance Compensation
 *
 * Purpose:
 *   Compensate for frequency response changes from absorptive filters,
 *   ensuring flat output spectrum regardless of RT60 settings.
 *
 * Transfer Function:
 *   |t(e^(jω))| ∝ 1 / √(T_r(ω))
 *
 *   Where T_r(ω) is frequency-dependent reverb time from absorptive filter.
 *
 * Implementation:
 *   High-shelf biquad filter with inverse characteristics of absorptive filter.
 *   Boost amount automatically computed from absorption parameters.
 *
 * Signal Flow:
 *   DelayLine Out → Mod → EQ → Absorptive → **Correction Filter** → Hadamard
 *
 * Automatic Update:
 *   Coefficients recomputed whenever absorption parameters change.
 */
class CorrectionFilter
{
public:
    CorrectionFilter();
    ~CorrectionFilter() = default;

    // Lifecycle
    void prepare(double sampleRate);
    void reset();

    // Parameter update (called when absorption settings change)
    void updateFromAbsorptive(float absCutoff, float absAmount);

    // Processing
    float process(float input);

    // Query
    float getBoostDB() const { return boostDB_; }

private:
    // Biquad state
    float z1_, z2_;  // Delay line state variables

    // Biquad coefficients
    float b0_, b1_, b2_;  // Numerator
    float a1_, a2_;       // Denominator (a0 = 1.0 normalized)

    // Computed parameters
    float boostDB_;  // HF boost amount

    double sampleRate_;
};

} // namespace FDN
} // namespace Cloudseed
