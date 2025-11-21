/*
CorrectionFilter Implementation

ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-correction>
*/

#include "CorrectionFilter.h"

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-correction>

CorrectionFilter::CorrectionFilter()
    : z1_(0.0f)
    , z2_(0.0f)
    , b0_(1.0f)
    , b1_(0.0f)
    , b2_(0.0f)
    , a1_(0.0f)
    , a2_(0.0f)
    , boostDB_(0.0f)
    , sampleRate_(48000.0)
{
}

void CorrectionFilter::prepare(double sampleRate)
{
    sampleRate_ = sampleRate;
    reset();
}

void CorrectionFilter::reset()
{
    z1_ = 0.0f;
    z2_ = 0.0f;
}

void CorrectionFilter::updateFromAbsorptive(float absCutoff, float absAmount)
{
    // Clamp inputs
    absCutoff = std::clamp(absCutoff, 1000.0f, 20000.0f);
    absAmount = std::clamp(absAmount, 0.0f, 1.0f);

    // Complete bypass when absorption = 0 (no correction needed)
    if (absAmount < 0.00001f) {
        // Unity gain passthrough
        b0_ = 1.0f;
        b1_ = 0.0f;
        b2_ = 0.0f;
        a1_ = 0.0f;
        a2_ = 0.0f;
        boostDB_ = 0.0f;
        return;
    }

    // Compute inverse of absorptive filter's HF attenuation
    // Absorptive filter: h(z) = k × (1-β) / (1 - β×z^(-1))
    // Measure HF loss at Nyquist frequency

    float omega = static_cast<float>(M_PI);  // Nyquist
    float beta = std::exp(-2.0f * static_cast<float>(M_PI) * absCutoff / static_cast<float>(sampleRate_));

    // Scale beta by amount (same as absorptive filter)
    beta = 1.0f - absAmount * (1.0f - beta);

    // HF attenuation from absorptive filter at Nyquist
    // |H(π)| = k × (1-β) / (1+β)
    float k = std::sqrt(1.0f - beta * beta);
    float absHFGain = k * (1.0f - beta) / (1.0f + beta);

    // Correction gain (inverse, scaled by amount)
    // Boost HF to compensate for absorptive filter loss
    boostDB_ = -20.0f * std::log10(absHFGain) * absAmount;
    boostDB_ = std::clamp(boostDB_, 0.0f, 12.0f);  // Limit boost to +12 dB

    // Convert to linear gain
    float gainLinear = std::pow(10.0f, boostDB_ / 20.0f);

    // Design high-shelf biquad filter at same cutoff frequency as absorptive filter
    // Coefficients for high-shelf filter (cookbook formula)
    float K = std::tan(static_cast<float>(M_PI) * absCutoff / static_cast<float>(sampleRate_));
    float V0 = gainLinear;
    float root2 = std::sqrt(2.0f);

    if (V0 >= 1.0f) {
        // Boost case
        float norm = 1.0f / (1.0f + root2 * K + K * K);
        b0_ = (V0 + root2 * std::sqrt(V0) * K + K * K) * norm;
        b1_ = 2.0f * (K * K - V0) * norm;
        b2_ = (V0 - root2 * std::sqrt(V0) * K + K * K) * norm;
        a1_ = 2.0f * (K * K - 1.0f) * norm;
        a2_ = (1.0f - root2 * K + K * K) * norm;
    } else {
        // Unity gain (shouldn't happen, but handle gracefully)
        b0_ = 1.0f;
        b1_ = 0.0f;
        b2_ = 0.0f;
        a1_ = 0.0f;
        a2_ = 0.0f;
    }
}

float CorrectionFilter::process(float input)
{
    // TEMPORARY: Complete bypass - correction filter depends on broken absorptive filter
    // TODO: Re-enable after absorptive filter math is fixed
    return input;

    /* DISABLED
    // Complete bypass when boost = 0
    if (boostDB_ < 0.01f) {
        return input;
    }

    // Biquad processing (Direct Form I)
    // y[n] = b0×x[n] + b1×x[n-1] + b2×x[n-2] - a1×y[n-1] - a2×y[n-2]
    float output = b0_ * input + b1_ * z1_ + b2_ * z2_ - a1_ * z1_ - a2_ * z2_;

    // Update state
    z2_ = z1_;
    z1_ = input;

    return output;
    */
}

} // namespace FDN
} // namespace Cloudseed
