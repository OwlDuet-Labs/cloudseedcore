/*
FDN Core Processing Engine
Template-based N×N Feedback Delay Network with Hadamard matrix, modulation, per-line EQ,
and v1.2 quality improvements (absorptive filters, correction filters).

SUPPORTS: 8×8 (64 echoes) and 16×16 (256 echoes) with runtime switching.

ADC-IMPLEMENTS: <reverb-v1-fdn-topology-datamodel-01>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-impl-01>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-injection>
ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-absorptive>
ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-correction>
ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
*/

#pragma once

// Define CloudSeedCore constants if not already defined
#ifndef BUFFER_SIZE
#define BUFFER_SIZE 512
#endif

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include "../ModulatedDelay.h"
#include "../Biquad.h"
#include "../Utils.h"
#include "DelayConfig.h"
#include "DelayConfig16.h"
#include "HadamardTransform.h"
#include "AbsorptiveFilter.h"
#include "CorrectionFilter.h"

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
/**
 * FDNCore: Template-based N×N Feedback Delay Network (v1.2)
 *
 * Template Parameter:
 * - N: FDN size (8 for 8×8, 16 for 16×16)
 *
 * Architecture:
 * - N delay lines with modulated read positions
 * - Per-line EQ (low shelf, high shelf, lowpass)
 * - v1.2: Per-line absorptive filters (frequency-dependent decay)
 * - v1.2: Per-line correction filters (flat frequency response)
 * - Hadamard N×N orthogonal feedback matrix
 * - Divergent stereo injection (L→lines 0..N/2-1, R→lines N/2..N-1)
 * - Unified decay control via feedback gain
 *
 * Signal Flow (per sample) - v1.2 updated:
 * 1. Read from all delay lines → apply modulation → apply EQ
 * 2. v1.2: Apply absorptive filter → apply correction filter
 * 3. Apply Hadamard matrix to delay outputs
 * 4. Scale by decay gain
 * 5. Add input injection (L/R split) and write back to delays
 * 6. Extract outputs (sum lines 0..N/2-1 for L, N/2..N-1 for R)
 *
 * Key Properties:
 * - Guaranteed stability (orthogonal matrix → eigenvalues on unit circle)
 * - Natural stereo coupling (matrix feedback, no explicit crossfeed)
 * - Coprime delay lengths (prevents comb filtering)
 * - Real-time safe (no allocations in process())
 */
template<int N>
class FDNCore
{
public:
    FDNCore();
    ~FDNCore() = default;

    // Lifecycle methods
    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    // Processing
    void process(const float* inputL, const float* inputR,
                float* outputL, float* outputR, int numSamples);

    // Parameter setters (atomic, thread-safe from message thread)
    void setDecayTime(float rt60Seconds);
    void setFDNSize(float sizeMs);
    void setModulationAmount(float amount);  // 0-1
    void setModulationRate(float hz);
    void setEQEnabled(bool lowShelf, bool highShelf, bool lowpass);
    void setEQFrequencies(float lowFreq, float highFreq, float cutoff);
    void setEQGains(float lowGain, float highGain);

    // v1.2: Quality improvement parameters
    void setAbsorptionCutoff(float cutoffHz);
    void setAbsorptionAmount(float amount01);

private:
    // Delay lines (N total, using ModulatedDelay directly)
    std::array<ModulatedDelay, N> delayLines_;

    // Modulation (N LFO phases, one per line)
    std::array<float, N> lfoPhases_;

    // EQ (3 filters per line: low shelf, high shelf, lowpass)
    std::array<Biquad, N> lowShelfFilters_;
    std::array<Biquad, N> highShelfFilters_;
    std::array<Biquad, N> lowpassFilters_;

    // v1.2: Quality filters (absorptive + correction per line)
    std::array<AbsorptiveFilter, N> absorptiveFilters_;
    std::array<CorrectionFilter, N> correctionFilters_;

    // Processing buffers (reused each block to avoid allocations)
    std::array<float, N> delayOutputs_;
    std::array<float, N> feedbackSignals_;
    std::array<float, N> feedbackInput_;  // Input prepared for next delay iteration

    // Parameters (atomic for thread-safe updates from message thread)
    std::atomic<float> decayGain_;
    std::atomic<float> sizeMultiplier_;
    std::atomic<float> modulationDepth_;
    std::atomic<float> modulationFreq_;

    std::atomic<bool> eqLowShelfEnabled_;
    std::atomic<bool> eqHighShelfEnabled_;
    std::atomic<bool> eqLowpassEnabled_;

    std::atomic<float> eqLowFreq_;
    std::atomic<float> eqHighFreq_;
    std::atomic<float> eqCutoff_;
    std::atomic<float> eqLowGain_;
    std::atomic<float> eqHighGain_;

    // v1.2: Absorption parameters
    std::atomic<float> absorptionCutoff_;
    std::atomic<float> absorptionAmount_;

    // State
    double currentSampleRate_;
    int scaledDelayLengths_[N];
    bool eqNeedsUpdate_;
    bool delayLengthsNeedUpdate_;

    // Internal methods
    void updateDelayLengths();
    void updateEQFilters();
    void updateAbsorptiveFilters();
    void updateCorrectionFilters();
    float computeDecayGain(float rt60Seconds);
    float processEQ(int lineIndex, float input);
    float advanceLFO(int lineIndex);

    // Config selector - specialized for each N
    const int* getDelayLengths();
    bool getChannelAssignment(int i);
};

// ============================================================================
// TEMPLATE IMPLEMENTATION (must be in header)
// ============================================================================

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-datamodel-01>
template<int N>
FDNCore<N>::FDNCore()
    : currentSampleRate_(48000.0)
    , decayGain_(0.8f)
    , sizeMultiplier_(1.0f)
    , modulationDepth_(0.0f)
    , modulationFreq_(1.0f)
    , eqLowShelfEnabled_(false)
    , eqHighShelfEnabled_(false)
    , eqLowpassEnabled_(false)
    , eqLowFreq_(150.0f)
    , eqHighFreq_(6000.0f)
    , eqCutoff_(12000.0f)
    , eqLowGain_(0.0f)
    , eqHighGain_(0.0f)
    , absorptionCutoff_(5000.0f)
    , absorptionAmount_(0.5f)
    , eqNeedsUpdate_(false)
    , delayLengthsNeedUpdate_(false)
{
    // Initialize LFO phases with unique offsets per line (avoid correlation)
    for (int i = 0; i < N; i++) {
        lfoPhases_[i] = (i * 2.0f * 3.14159265f) / N;
    }

    // Initialize delay lengths at 48kHz
    updateDelayLengths();
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
template<int N>
const int* FDNCore<N>::getDelayLengths()
{
    if constexpr (N == 8) {
        return FDNConfig::delayLengths48k;
    } else if constexpr (N == 16) {
        return FDNConfig16::delayLengths48k;
    } else {
        static_assert(N == 8 || N == 16, "FDNCore only supports N=8 or N=16");
        return nullptr;
    }
}

template<int N>
bool FDNCore<N>::getChannelAssignment(int i)
{
    if constexpr (N == 8) {
        return FDNConfig::isLeftChannel[i];
    } else if constexpr (N == 16) {
        return FDNConfig16::isLeftChannel[i];
    } else {
        static_assert(N == 8 || N == 16, "FDNCore only supports N=8 or N=16");
        return false;
    }
}

template<int N>
void FDNCore<N>::prepare(double sampleRate, int maxBlockSize)
{
    currentSampleRate_ = sampleRate;

    // Compute delay lengths for current sample rate with coprimality preservation
    updateDelayLengths();

    // Initialize delay lines (ModulatedDelay uses SampleDelay field directly)
    for (int i = 0; i < N; i++) {
        delayLines_[i].SampleDelay = scaledDelayLengths_[i];
        delayLines_[i].ModAmount = 0.0f;  // Will be set in process()
        delayLines_[i].ModRate = 0.0f;
    }

    // Initialize EQ filters
    updateEQFilters();

    // v1.2: Initialize absorptive and correction filters
    for (int i = 0; i < N; i++) {
        absorptiveFilters_[i].prepare(sampleRate);
        correctionFilters_[i].prepare(sampleRate);
    }
    updateAbsorptiveFilters();
    updateCorrectionFilters();

    // Reset state
    reset();
}

template<int N>
void FDNCore<N>::reset()
{
    // Clear all delay line buffers
    for (int i = 0; i < N; i++) {
        delayLines_[i].ClearBuffers();
    }

    // Reset LFO phases
    for (int i = 0; i < N; i++) {
        lfoPhases_[i] = (i * 2.0f * 3.14159265f) / N;
    }

    // v1.2: Reset absorptive and correction filter state
    for (int i = 0; i < N; i++) {
        absorptiveFilters_[i].reset();
        correctionFilters_[i].reset();
    }

    // Clear processing buffers
    delayOutputs_.fill(0.0f);
    feedbackSignals_.fill(0.0f);
    feedbackInput_.fill(0.0f);
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-impl-01>
// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-injection>
// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-modulation>
template<int N>
void FDNCore<N>::process(const float* inputL, const float* inputR,
                     float* outputL, float* outputR, int numSamples)
{
    // Prime multipliers for frequency offset (per-line modulation decorrelation)
    static constexpr float primeMultipliers8[8] = {
        1.000f, 1.013f, 1.031f, 1.061f, 1.091f, 1.131f, 1.193f, 1.311f
    };

    static constexpr float primeMultipliers16[16] = {
        1.000f, 1.013f, 1.031f, 1.061f, 1.091f, 1.131f, 1.193f, 1.311f,
        1.433f, 1.511f, 1.613f, 1.733f, 1.811f, 1.933f, 2.011f, 2.113f
    };

    const float* primeMultipliers = (N == 8) ? primeMultipliers8 : primeMultipliers16;

    // Load current parameters (atomic reads, lock-free)
    const float decay = decayGain_.load(std::memory_order_relaxed);
    const float modDepth = modulationDepth_.load(std::memory_order_relaxed);
    const float modFreq = modulationFreq_.load(std::memory_order_relaxed);

    // Complete bypass when depth=0
    const bool modulationEnabled = (modDepth > 0.00001f);

    // Update EQ if parameters changed
    if (eqNeedsUpdate_) {
        updateEQFilters();
        eqNeedsUpdate_ = false;
    }

    // Update delay lengths if size changed
    if (delayLengthsNeedUpdate_) {
        updateDelayLengths();
        delayLengthsNeedUpdate_ = false;
    }

    // Energy normalization factor for output sum
    // 1/sqrt(N/2) maintains constant output level
    const float outputNormalization = 1.0f / std::sqrt(static_cast<float>(N / 2));

    // Temporary buffers for delay processing
    float delayInput[BUFFER_SIZE];
    float delayOutput[BUFFER_SIZE];

    // Process each sample
    for (int sample = 0; sample < numSamples; sample++) {
        // ================================================
        // STEP 1: Read from delay lines + Apply Modulation + EQ + v1.2 Filters
        // ================================================
        for (int i = 0; i < N; i++) {
            // Apply prime-based frequency offsets for each line
            if (modulationEnabled) {
                float lineFreq = modFreq * primeMultipliers[i];
                delayLines_[i].ModAmount = modDepth;
                delayLines_[i].ModRate = lineFreq;
            } else {
                // Complete bypass when depth=0
                delayLines_[i].ModAmount = 0.0f;
                delayLines_[i].ModRate = 0.0f;
            }

            // Process delay line
            delayLines_[i].Process(&feedbackInput_[i], &delayOutput[0], 1);

            // Apply per-line EQ to delayed output
            float eqOut = processEQ(i, delayOutput[0]);

            // v1.2: Apply absorptive and correction filters
            float absOut = absorptiveFilters_[i].process(eqOut);
            float corrOut = correctionFilters_[i].process(absOut);

            delayOutputs_[i] = corrOut;
        }

        // ================================================
        // STEP 2: Apply Hadamard feedback matrix
        // ================================================
        if constexpr (N == 8) {
            HadamardTransform::apply8(delayOutputs_.data(), feedbackSignals_.data());
        } else if constexpr (N == 16) {
            HadamardTransform::apply16(delayOutputs_.data(), feedbackSignals_.data());
        }

        // ================================================
        // STEP 3: DIVERGENT INPUT INJECTION - Prepare feedback for next iteration
        // ================================================
        const float inputSampleL = inputL[sample];
        const float inputSampleR = inputR[sample];

        // LEFT CHANNEL: Inject into lines 0..N/2-1
        for (int i = 0; i < N / 2; i++) {
            feedbackInput_[i] = inputSampleL + feedbackSignals_[i] * decay;
        }

        // RIGHT CHANNEL: Inject into lines N/2..N-1
        for (int i = N / 2; i < N; i++) {
            feedbackInput_[i] = inputSampleR + feedbackSignals_[i] * decay;
        }

        // ================================================
        // STEP 4: Output extraction (sum groups with normalization)
        // ================================================
        float outL = 0.0f;
        float outR = 0.0f;

        // Left output: Sum lines 0..N/2-1
        for (int i = 0; i < N / 2; i++) {
            outL += delayOutputs_[i];
        }

        // Right output: Sum lines N/2..N-1
        for (int i = N / 2; i < N; i++) {
            outR += delayOutputs_[i];
        }

        // Apply energy normalization
        outputL[sample] = outL * outputNormalization;
        outputR[sample] = outR * outputNormalization;

        // ================================================
        // STEP 5: Advance LFO phases
        // ================================================
        for (int i = 0; i < N; i++) {
            float lfoIncrement = 2.0f * 3.14159265f * modulationFreq_.load(std::memory_order_relaxed) / currentSampleRate_;
            lfoPhases_[i] += lfoIncrement;
            if (lfoPhases_[i] > 2.0f * 3.14159265f) {
                lfoPhases_[i] -= 2.0f * 3.14159265f;
            }
        }
    }
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-03>
template<int N>
float FDNCore<N>::computeDecayGain(float rt60Seconds)
{
    // Clamp RT60 to safe range
    if (rt60Seconds < 0.05f) rt60Seconds = 0.05f;
    if (rt60Seconds > 60.0f) rt60Seconds = 60.0f;

    // Compute average delay length
    float avgDelayLengthSamples = 0.0f;
    for (int i = 0; i < N; i++) {
        avgDelayLengthSamples += scaledDelayLengths_[i];
    }
    avgDelayLengthSamples /= N;

    // Compute feedback gain for desired RT60
    float exponent = -3.0f * avgDelayLengthSamples / (currentSampleRate_ * rt60Seconds);
    float gain = std::pow(10.0f, exponent);

    // Clamp to safe range [0.0, 0.999]
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 0.999f) gain = 0.999f;

    return gain;
}

template<int N>
void FDNCore<N>::setDecayTime(float rt60Seconds)
{
    float gain = computeDecayGain(rt60Seconds);
    decayGain_.store(gain, std::memory_order_relaxed);
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-04>
// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-size-scaling>
template<int N>
void FDNCore<N>::setFDNSize(float sizeMs)
{
    constexpr float defaultSizeMs = 236.0f;
    constexpr float minSizeMs = 20.0f;
    constexpr float maxSizeMs = 1000.0f;

    // Clamp to valid range
    float clampedSize = sizeMs;
    if (clampedSize < minSizeMs) clampedSize = minSizeMs;
    if (clampedSize > maxSizeMs) clampedSize = maxSizeMs;

    // Compute multiplier relative to default
    float multiplier = clampedSize / defaultSizeMs;

    // Clamp multiplier to safe range (0.5x to 2.0x)
    if (multiplier < 0.5f) multiplier = 0.5f;
    if (multiplier > 2.0f) multiplier = 2.0f;

    sizeMultiplier_.store(multiplier, std::memory_order_relaxed);
    delayLengthsNeedUpdate_ = true;

    // Recompute delay lengths with new size multiplier
    updateDelayLengths();
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-modulation>
template<int N>
void FDNCore<N>::setModulationAmount(float amount)
{
    constexpr float maxDepthSamples = 0.5f;

    // Clamp to [0, 1] range
    if (amount < 0.0f) amount = 0.0f;
    if (amount > 1.0f) amount = 1.0f;

    float depthSamples = amount * maxDepthSamples;
    modulationDepth_.store(depthSamples, std::memory_order_relaxed);
}

template<int N>
void FDNCore<N>::setModulationRate(float hz)
{
    // Clamp to reasonable range [0.01, 2 Hz]
    if (hz < 0.01f) hz = 0.01f;
    if (hz > 2.0f) hz = 2.0f;

    modulationFreq_.store(hz, std::memory_order_relaxed);
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-05>
template<int N>
void FDNCore<N>::setEQEnabled(bool lowShelf, bool highShelf, bool lowpass)
{
    eqLowShelfEnabled_.store(lowShelf, std::memory_order_relaxed);
    eqHighShelfEnabled_.store(highShelf, std::memory_order_relaxed);
    eqLowpassEnabled_.store(lowpass, std::memory_order_relaxed);
    eqNeedsUpdate_ = true;
}

template<int N>
void FDNCore<N>::setEQFrequencies(float lowFreq, float highFreq, float cutoff)
{
    eqLowFreq_.store(lowFreq, std::memory_order_relaxed);
    eqHighFreq_.store(highFreq, std::memory_order_relaxed);
    eqCutoff_.store(cutoff, std::memory_order_relaxed);
    eqNeedsUpdate_ = true;
}

template<int N>
void FDNCore<N>::setEQGains(float lowGain, float highGain)
{
    eqLowGain_.store(lowGain, std::memory_order_relaxed);
    eqHighGain_.store(highGain, std::memory_order_relaxed);
    eqNeedsUpdate_ = true;
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-05>
template<int N>
void FDNCore<N>::updateEQFilters()
{
    const float lowFreq = eqLowFreq_.load(std::memory_order_relaxed);
    const float highFreq = eqHighFreq_.load(std::memory_order_relaxed);
    const float cutoffFreq = eqCutoff_.load(std::memory_order_relaxed);
    const float lowGain = eqLowGain_.load(std::memory_order_relaxed);
    const float highGain = eqHighGain_.load(std::memory_order_relaxed);

    for (int i = 0; i < N; i++) {
        // Low shelf filter configuration
        lowShelfFilters_[i].Type = Cloudseed::Biquad::FilterType::LowShelf;
        lowShelfFilters_[i].Frequency = lowFreq;
        lowShelfFilters_[i].SetQ(0.707f);
        lowShelfFilters_[i].SetGainDb(lowGain);
        lowShelfFilters_[i].SetSamplerate(currentSampleRate_);
        lowShelfFilters_[i].Update();

        // High shelf filter configuration
        highShelfFilters_[i].Type = Cloudseed::Biquad::FilterType::HighShelf;
        highShelfFilters_[i].Frequency = highFreq;
        highShelfFilters_[i].SetQ(0.707f);
        highShelfFilters_[i].SetGainDb(highGain);
        highShelfFilters_[i].SetSamplerate(currentSampleRate_);
        highShelfFilters_[i].Update();

        // Lowpass filter configuration
        lowpassFilters_[i].Type = Cloudseed::Biquad::FilterType::LowPass;
        lowpassFilters_[i].Frequency = cutoffFreq;
        lowpassFilters_[i].SetQ(0.707f);
        lowpassFilters_[i].SetSamplerate(currentSampleRate_);
        lowpassFilters_[i].Update();
    }
}

template<int N>
float FDNCore<N>::processEQ(int lineIndex, float input)
{
    float output = input;

    // Apply filters in series
    if (eqLowShelfEnabled_.load(std::memory_order_relaxed)) {
        output = lowShelfFilters_[lineIndex].Process(output);
    }

    if (eqHighShelfEnabled_.load(std::memory_order_relaxed)) {
        output = highShelfFilters_[lineIndex].Process(output);
    }

    if (eqLowpassEnabled_.load(std::memory_order_relaxed)) {
        output = lowpassFilters_[lineIndex].Process(output);
    }

    return output;
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-02>
// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-size-scaling>
// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
template<int N>
void FDNCore<N>::updateDelayLengths()
{
    // Get current size multiplier
    float sizeMult = sizeMultiplier_.load(std::memory_order_relaxed);

    // Get base delay lengths for this N
    const int* baseLengths = getDelayLengths();

    // Compute scaled delay lengths
    for (int i = 0; i < N; i++) {
        double scaleFactor = (currentSampleRate_ / 48000.0) * sizeMult;
        int scaledLength = static_cast<int>(baseLengths[i] * scaleFactor + 0.5);

        // Ensure minimum delay
        if (scaledLength < 32) {
            scaledLength = 32;
        }

        scaledDelayLengths_[i] = scaledLength;
    }

    // Update delay line buffer sizes
    for (int i = 0; i < N; i++) {
        delayLines_[i].SampleDelay = scaledDelayLengths_[i];
    }
}

// ============================================================================
// v1.2: Absorptive Filter Parameter Management
// ============================================================================

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-absorptive>
template<int N>
void FDNCore<N>::setAbsorptionCutoff(float cutoffHz)
{
    // Clamp to valid range (1kHz - 20kHz)
    cutoffHz = std::clamp(cutoffHz, 1000.0f, 20000.0f);

    absorptionCutoff_.store(cutoffHz, std::memory_order_relaxed);

    // Update all absorptive and correction filters
    updateAbsorptiveFilters();
    updateCorrectionFilters();
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-absorptive>
template<int N>
void FDNCore<N>::setAbsorptionAmount(float amount01)
{
    // Clamp to valid range (0-1)
    amount01 = std::clamp(amount01, 0.0f, 1.0f);

    absorptionAmount_.store(amount01, std::memory_order_relaxed);

    // Update all absorptive and correction filters
    updateAbsorptiveFilters();
    updateCorrectionFilters();
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-absorptive>
template<int N>
void FDNCore<N>::updateAbsorptiveFilters()
{
    float cutoff = absorptionCutoff_.load(std::memory_order_relaxed);
    float amount = absorptionAmount_.load(std::memory_order_relaxed);

    // Update all absorptive filters with current parameters
    for (int i = 0; i < N; i++) {
        absorptiveFilters_[i].setParameters(cutoff, amount);
    }
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-correction>
template<int N>
void FDNCore<N>::updateCorrectionFilters()
{
    float cutoff = absorptionCutoff_.load(std::memory_order_relaxed);
    float amount = absorptionAmount_.load(std::memory_order_relaxed);

    // Update all correction filters to match absorptive filter settings
    for (int i = 0; i < N; i++) {
        correctionFilters_[i].updateFromAbsorptive(cutoff, amount);
    }
}

// ============================================================================
// FDNCoreManager: Dual-Instance Wrapper with Runtime Switching
// ============================================================================

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
/**
 * FDNCoreManager: Runtime-Switchable 8×8 ↔ 16×16 FDN
 *
 * Architecture:
 * - Holds two pre-allocated FDN instances (8×8 and 16×16)
 * - Provides smooth crossfade during size switching (50ms)
 * - Thread-safe size changes via atomic variables
 * - All parameter changes propagate to both instances
 *
 * Crossfade Strategy:
 * - When size changes, start crossfade from old → new
 * - During crossfade: output = (1-t)*old + t*new (linear)
 * - Duration: 50ms (2400 samples @ 48kHz)
 * - No clicks or pops during transition
 *
 * Memory:
 * - Both instances pre-allocated at prepare() time
 * - No runtime allocations during switching
 * - Inactive instance continues processing (maintains state)
 */
class FDNCoreManager
{
public:
    FDNCoreManager();
    ~FDNCoreManager() = default;

    // Lifecycle methods
    void prepare(double sampleRate, int maxBlockSize);
    void reset();

    // Processing (routes to active instance with crossfade)
    void process(const float* inputL, const float* inputR,
                float* outputL, float* outputR, int numSamples);

    // FDN size selection (8 or 16)
    void setFDNSize(int size);  // 8 or 16
    int getFDNSize() const;

    // Parameter setters (forward to both instances)
    void setDecayTime(float rt60Seconds);
    void setFDNSizeMs(float sizeMs);  // Delay length scaling
    void setModulationAmount(float amount);
    void setModulationRate(float hz);
    void setEQEnabled(bool lowShelf, bool highShelf, bool lowpass);
    void setEQFrequencies(float lowFreq, float highFreq, float cutoff);
    void setEQGains(float lowGain, float highGain);
    void setAbsorptionCutoff(float cutoffHz);
    void setAbsorptionAmount(float amount01);

private:
    // Dual FDN instances
    FDNCore<8> fdn8_;
    FDNCore<16> fdn16_;

    // State management
    std::atomic<int> activeSize_;      // 8 or 16
    std::atomic<int> targetSize_;      // Target during crossfade
    std::atomic<int> crossfadeSamples_; // Remaining crossfade samples

    double sampleRate_;
    static constexpr int CROSSFADE_DURATION_MS = 50;

    // Crossfade buffers
    float tempOutL8_[BUFFER_SIZE];
    float tempOutR8_[BUFFER_SIZE];
    float tempOutL16_[BUFFER_SIZE];
    float tempOutR16_[BUFFER_SIZE];
};

// ============================================================================
// FDNCoreManager Implementation
// ============================================================================

inline FDNCoreManager::FDNCoreManager()
    : activeSize_(8)
    , targetSize_(8)
    , crossfadeSamples_(0)
    , sampleRate_(48000.0)
{
}

inline void FDNCoreManager::prepare(double sampleRate, int maxBlockSize)
{
    sampleRate_ = sampleRate;

    // Pre-allocate both instances
    fdn8_.prepare(sampleRate, maxBlockSize);
    fdn16_.prepare(sampleRate, maxBlockSize);

    // Start with 8×8 active
    activeSize_.store(8, std::memory_order_release);
    targetSize_.store(8, std::memory_order_release);
    crossfadeSamples_.store(0, std::memory_order_release);
}

inline void FDNCoreManager::reset()
{
    fdn8_.reset();
    fdn16_.reset();
    crossfadeSamples_.store(0, std::memory_order_release);
}

inline void FDNCoreManager::setFDNSize(int size)
{
    // Validate size
    if (size != 8 && size != 16) {
        size = 8;  // Default to 8×8
    }

    int currentActive = activeSize_.load(std::memory_order_acquire);

    // Only start crossfade if size actually changed
    if (size != currentActive) {
        targetSize_.store(size, std::memory_order_release);

        // Start crossfade (50ms)
        int crossfadeDuration = static_cast<int>(sampleRate_ * CROSSFADE_DURATION_MS / 1000.0);
        crossfadeSamples_.store(crossfadeDuration, std::memory_order_release);
    }
}

inline int FDNCoreManager::getFDNSize() const
{
    return activeSize_.load(std::memory_order_acquire);
}

inline void FDNCoreManager::process(const float* inputL, const float* inputR,
                                    float* outputL, float* outputR, int numSamples)
{
    int crossfadeRemaining = crossfadeSamples_.load(std::memory_order_acquire);

    if (crossfadeRemaining > 0) {
        // CROSSFADE MODE: Process both instances and blend
        int currentActive = activeSize_.load(std::memory_order_acquire);
        int target = targetSize_.load(std::memory_order_acquire);

        // Process both FDNs
        fdn8_.process(inputL, inputR, tempOutL8_, tempOutR8_, numSamples);
        fdn16_.process(inputL, inputR, tempOutL16_, tempOutR16_, numSamples);

        // Crossfade blend
        for (int i = 0; i < numSamples; i++) {
            // Compute blend factor (linear crossfade)
            int totalDuration = static_cast<int>(sampleRate_ * CROSSFADE_DURATION_MS / 1000.0);
            float t = 1.0f - (static_cast<float>(crossfadeRemaining) / totalDuration);
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;

            // Blend outputs
            if (currentActive == 8 && target == 16) {
                // 8→16 transition
                outputL[i] = (1.0f - t) * tempOutL8_[i] + t * tempOutL16_[i];
                outputR[i] = (1.0f - t) * tempOutR8_[i] + t * tempOutR16_[i];
            } else if (currentActive == 16 && target == 8) {
                // 16→8 transition
                outputL[i] = (1.0f - t) * tempOutL16_[i] + t * tempOutL8_[i];
                outputR[i] = (1.0f - t) * tempOutR16_[i] + t * tempOutR8_[i];
            } else {
                // Should not happen, but fallback to current active
                if (currentActive == 8) {
                    outputL[i] = tempOutL8_[i];
                    outputR[i] = tempOutR8_[i];
                } else {
                    outputL[i] = tempOutL16_[i];
                    outputR[i] = tempOutR16_[i];
                }
            }

            // Decrement crossfade counter
            crossfadeRemaining--;
            if (crossfadeRemaining == 0) {
                // Crossfade complete - switch to target
                activeSize_.store(target, std::memory_order_release);
            }
        }

        // Update remaining crossfade samples
        crossfadeSamples_.store(crossfadeRemaining, std::memory_order_release);

    } else {
        // NORMAL MODE: Route to active instance only
        int currentActive = activeSize_.load(std::memory_order_acquire);

        if (currentActive == 8) {
            fdn8_.process(inputL, inputR, outputL, outputR, numSamples);
            // Keep 16×16 processing to maintain state
            fdn16_.process(inputL, inputR, tempOutL16_, tempOutR16_, numSamples);
        } else {
            fdn16_.process(inputL, inputR, outputL, outputR, numSamples);
            // Keep 8×8 processing to maintain state
            fdn8_.process(inputL, inputR, tempOutL8_, tempOutR8_, numSamples);
        }
    }
}

// Parameter forwarding (all parameters go to both instances)

inline void FDNCoreManager::setDecayTime(float rt60Seconds)
{
    fdn8_.setDecayTime(rt60Seconds);
    fdn16_.setDecayTime(rt60Seconds);
}

inline void FDNCoreManager::setFDNSizeMs(float sizeMs)
{
    fdn8_.setFDNSize(sizeMs);
    fdn16_.setFDNSize(sizeMs);
}

inline void FDNCoreManager::setModulationAmount(float amount)
{
    fdn8_.setModulationAmount(amount);
    fdn16_.setModulationAmount(amount);
}

inline void FDNCoreManager::setModulationRate(float hz)
{
    fdn8_.setModulationRate(hz);
    fdn16_.setModulationRate(hz);
}

inline void FDNCoreManager::setEQEnabled(bool lowShelf, bool highShelf, bool lowpass)
{
    fdn8_.setEQEnabled(lowShelf, highShelf, lowpass);
    fdn16_.setEQEnabled(lowShelf, highShelf, lowpass);
}

inline void FDNCoreManager::setEQFrequencies(float lowFreq, float highFreq, float cutoff)
{
    fdn8_.setEQFrequencies(lowFreq, highFreq, cutoff);
    fdn16_.setEQFrequencies(lowFreq, highFreq, cutoff);
}

inline void FDNCoreManager::setEQGains(float lowGain, float highGain)
{
    fdn8_.setEQGains(lowGain, highGain);
    fdn16_.setEQGains(lowGain, highGain);
}

inline void FDNCoreManager::setAbsorptionCutoff(float cutoffHz)
{
    fdn8_.setAbsorptionCutoff(cutoffHz);
    fdn16_.setAbsorptionCutoff(cutoffHz);
}

inline void FDNCoreManager::setAbsorptionAmount(float amount01)
{
    fdn8_.setAbsorptionAmount(amount01);
    fdn16_.setAbsorptionAmount(amount01);
}

} // namespace FDN
} // namespace Cloudseed
