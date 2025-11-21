/*
AbsorptiveFilter Implementation

ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-absorptive>
*/

#include "AbsorptiveFilter.h"

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-absorptive>

AbsorptiveFilter::AbsorptiveFilter()
    : z1_(0.0f)
    , k_(1.0f)
    , beta_(0.0f)
    , cutoffHz_(8000.0f)
    , amount_(0.0f)  // DEFAULT TO BYPASS (was 0.5f - caused decay issues)
    , sampleRate_(48000.0)
{
}

void AbsorptiveFilter::prepare(double sampleRate)
{
    sampleRate_ = sampleRate;
    reset();
    setParameters(cutoffHz_, amount_);  // Recompute coefficients for new sample rate
}

void AbsorptiveFilter::reset()
{
    z1_ = 0.0f;
}

void AbsorptiveFilter::setParameters(float cutoffHz, float amount01)
{
    // Store parameters
    cutoffHz_ = std::clamp(cutoffHz, 1000.0f, 20000.0f);
    amount_ = std::clamp(amount01, 0.0f, 1.0f);

    // Compute pole location from cutoff frequency
    // β = exp(-2π × f_cutoff / sample_rate)
    float betaBase = std::exp(-2.0f * static_cast<float>(M_PI) * cutoffHz_ / static_cast<float>(sampleRate_));

    // Scale by amount parameter
    // amount = 0: no absorption (β → 1.0, bypass filter)
    // amount = 1: max absorption (β at computed value)
    beta_ = 1.0f - amount_ * (1.0f - betaBase);

    // Compute k for energy preservation
    // k ensures unity DC gain and energy balance
    k_ = std::sqrt(1.0f - beta_ * beta_);

    // Clamp to safe range
    beta_ = std::clamp(beta_, 0.0f, 0.999f);  // Prevent instability
    k_ = std::clamp(k_, 0.0f, 1.0f);
}

float AbsorptiveFilter::process(float input)
{
    // TEMPORARY: Complete bypass - filters are causing RT60 and instability issues
    // TODO: Fix energy-preserving coefficient math per ADC-009 contract
    return input;

    /* DISABLED - BROKEN IMPLEMENTATION
    // Complete bypass when amount = 0 (CRITICAL: avoid artifacts)
    if (amount_ < 0.00001f) {
        return input;
    }

    // 1st-order IIR: y[n] = k × (1-β) × x[n] + β × y[n-1]
    float output = k_ * (1.0f - beta_) * input + beta_ * z1_;

    // Update state
    z1_ = output;

    return output;
    */
}

} // namespace FDN
} // namespace Cloudseed
