/*
FDN Core Processing Implementation

ADC-IMPLEMENTS: <reverb-v1-fdn-topology-impl-01>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-injection>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-03>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-04>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-05>
*/

#include "FDNCore.h"
#include "../Utils.h"
#include <cstring>

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-datamodel-01>
FDNCore::FDNCore()
    : currentSampleRate_(48000.0)
    , decayGain_(0.8f)
    , sizeMultiplier_(1.0f)  // ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-size-scaling>
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
    , eqNeedsUpdate_(false)
    , delayLengthsNeedUpdate_(false)  // ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-size-scaling>
{
    // Initialize LFO phases with unique offsets per line (avoid correlation)
    for (int i = 0; i < FDNConfig::N; i++) {
        lfoPhases_[i] = (i * 2.0f * 3.14159265f) / FDNConfig::N;
    }

    // Initialize delay lengths at 48kHz
    updateDelayLengths();
}

void FDNCore::prepare(double sampleRate, int maxBlockSize)
{
    currentSampleRate_ = sampleRate;

    // Compute delay lengths for current sample rate with coprimality preservation
    updateDelayLengths();

    // Initialize delay lines (ModulatedDelay uses SampleDelay field directly)
    for (int i = 0; i < FDNConfig::N; i++) {
        delayLines_[i].SampleDelay = scaledDelayLengths_[i];
        delayLines_[i].ModAmount = 0.0f;  // Will be set in process()
        delayLines_[i].ModRate = 0.0f;
    }

    // Initialize EQ filters
    updateEQFilters();

    // Reset state
    reset();
}

void FDNCore::reset()
{
    // Clear all delay line buffers
    for (int i = 0; i < FDNConfig::N; i++) {
        delayLines_[i].ClearBuffers();
    }

    // Reset LFO phases
    for (int i = 0; i < FDNConfig::N; i++) {
        lfoPhases_[i] = (i * 2.0f * 3.14159265f) / FDNConfig::N;
    }

    // Clear processing buffers
    delayOutputs_.fill(0.0f);
    feedbackSignals_.fill(0.0f);
    feedbackInput_.fill(0.0f);
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-impl-01>
// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-injection>
// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-modulation>
/**
 * Main FDN processing loop with divergent stereo injection
 *
 * Per-sample processing:
 * 1. Read delay lines → apply modulation → apply EQ
 * 2. Hadamard matrix feedback
 * 3. Scale by decay gain
 * 4. DIVERGENT INJECTION: L→lines 0-3, R→lines 4-7
 * 5. Write feedback + input to delay lines
 * 6. Sum outputs: L=lines[0-3], R=lines[4-7]
 *
 * v1.1 Updates:
 * - Prime-based modulation frequency offsets per line
 * - Complete bypass when modulation depth = 0
 * - Reduced modulation depth range (0-0.5 samples)
 */
void FDNCore::process(const float* inputL, const float* inputR,
                     float* outputL, float* outputR, int numSamples)
{
    // Prime multipliers for frequency offset (not just phase offset)
    // Each line modulates at slightly different rate for decorrelation
    // ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-modulation>
    static constexpr float primeMultipliers[FDNConfig::N] = {
        1.000f,  // Line 0: base frequency
        1.013f,  // Line 1: +1.3% (prime: 101/100)
        1.031f,  // Line 2: +3.1% (prime: 103/100)
        1.061f,  // Line 3: +6.1% (prime: 107/100)
        1.091f,  // Line 4: +9.1% (prime: 109/100)
        1.131f,  // Line 5: +13.1% (prime: 113/100)
        1.193f,  // Line 6: +19.3% (prime: 127/100)
        1.311f   // Line 7: +31.1% (prime: 131/100)
    };

    // Load current parameters (atomic reads, lock-free)
    const float decay = decayGain_.load(std::memory_order_relaxed);
    const float modDepth = modulationDepth_.load(std::memory_order_relaxed);
    const float modFreq = modulationFreq_.load(std::memory_order_relaxed);

    // v1.1: Complete bypass when depth=0 (CRITICAL FIX)
    const bool modulationEnabled = (modDepth > 0.00001f);

    // Update EQ if parameters changed
    if (eqNeedsUpdate_) {
        updateEQFilters();
        eqNeedsUpdate_ = false;
    }

    // Update delay lengths if size changed
    // ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-size-scaling>
    if (delayLengthsNeedUpdate_) {
        updateDelayLengths();
        delayLengthsNeedUpdate_ = false;
    }

    // Energy normalization factor for output sum
    // 1/sqrt(N/2) maintains constant output level
    const float outputNormalization = 1.0f / std::sqrt(static_cast<float>(FDNConfig::N / 2));

    // Temporary buffers for delay processing
    float delayInput[BUFFER_SIZE];
    float delayOutput[BUFFER_SIZE];

    // Process each sample
    for (int sample = 0; sample < numSamples; sample++) {
        // ================================================
        // STEP 1: Read from delay lines + Apply Modulation with Prime Offsets + Apply EQ
        // ================================================
        for (int i = 0; i < FDNConfig::N; i++) {
            // v1.1: Apply prime-based frequency offsets for each line
            // Each line gets unique modulation frequency (not just phase)
            // ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-modulation>
            if (modulationEnabled) {
                float lineFreq = modFreq * primeMultipliers[i];
                delayLines_[i].ModAmount = modDepth;
                delayLines_[i].ModRate = lineFreq;
            } else {
                // Complete bypass when depth=0
                delayLines_[i].ModAmount = 0.0f;
                delayLines_[i].ModRate = 0.0f;
            }

            // Process delay line: write feedback from previous iteration, read delayed output
            // Note: feedbackInput_ was prepared in previous iteration
            delayLines_[i].Process(&feedbackInput_[i], &delayOutput[0], 1);

            // Apply per-line EQ to delayed output
            float eqOut = processEQ(i, delayOutput[0]);
            delayOutputs_[i] = eqOut;
        }

        // ================================================
        // STEP 2: Apply Hadamard feedback matrix
        // ================================================
        HadamardTransform::apply(delayOutputs_.data(), feedbackSignals_.data());

        // ================================================
        // STEP 3: DIVERGENT INPUT INJECTION - Prepare feedback for next iteration
        // ================================================
        const float inputSampleL = inputL[sample];
        const float inputSampleR = inputR[sample];

        // LEFT CHANNEL: Inject into lines 0-3 ONLY
        for (int i = 0; i < FDNConfig::N / 2; i++) {
            feedbackInput_[i] = inputSampleL + feedbackSignals_[i] * decay;
        }

        // RIGHT CHANNEL: Inject into lines 4-7 ONLY
        for (int i = FDNConfig::N / 2; i < FDNConfig::N; i++) {
            feedbackInput_[i] = inputSampleR + feedbackSignals_[i] * decay;
        }

        // ================================================
        // STEP 4: Output extraction (sum groups with normalization)
        // ================================================
        float outL = 0.0f;
        float outR = 0.0f;

        // Left output: Sum lines 0-3
        for (int i = 0; i < FDNConfig::N / 2; i++) {
            outL += delayOutputs_[i];
        }

        // Right output: Sum lines 4-7
        for (int i = FDNConfig::N / 2; i < FDNConfig::N; i++) {
            outR += delayOutputs_[i];
        }

        // Apply energy normalization
        outputL[sample] = outL * outputNormalization;
        outputR[sample] = outR * outputNormalization;

        // ================================================
        // STEP 5: Advance LFO phases
        // ================================================
        for (int i = 0; i < FDNConfig::N; i++) {
            float lfoIncrement = 2.0f * 3.14159265f * modulationFreq_.load(std::memory_order_relaxed) / currentSampleRate_;
            lfoPhases_[i] += lfoIncrement;
            if (lfoPhases_[i] > 2.0f * 3.14159265f) {
                lfoPhases_[i] -= 2.0f * 3.14159265f;
            }
        }
    }
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-03>
/**
 * Compute feedback gain for desired RT60 decay time
 *
 * Formula: g = 10^(-3 * D / (sampleRate * RT60))
 * where D = average delay length in samples
 *
 * This ensures the reverb tail decays by 60dB in RT60 seconds.
 */
float FDNCore::computeDecayGain(float rt60Seconds)
{
    // Clamp RT60 to safe range
    if (rt60Seconds < 0.05f) rt60Seconds = 0.05f;
    if (rt60Seconds > 60.0f) rt60Seconds = 60.0f;

    // Compute average delay length
    float avgDelayLengthSamples = 0.0f;
    for (int i = 0; i < FDNConfig::N; i++) {
        avgDelayLengthSamples += scaledDelayLengths_[i];
    }
    avgDelayLengthSamples /= FDNConfig::N;

    // Compute feedback gain for desired RT60
    // RT60 is time for 60dB decay (amplitude factor of 1/1000)
    float exponent = -3.0f * avgDelayLengthSamples / (currentSampleRate_ * rt60Seconds);
    float gain = std::pow(10.0f, exponent);

    // Clamp to safe range [0.0, 0.999]
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 0.999f) gain = 0.999f;

    return gain;
}

void FDNCore::setDecayTime(float rt60Seconds)
{
    float gain = computeDecayGain(rt60Seconds);
    decayGain_.store(gain, std::memory_order_relaxed);
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-04>
// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-size-scaling>
void FDNCore::setFDNSize(float sizeMs)
{
    // Map parameter value (20-1000ms) to size multiplier
    // At 236ms (default), multiplier = 1.0 (no scaling)
    // Range: 0.5x to 2.0x scaling

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
void FDNCore::setModulationAmount(float amount)
{
    // v1.1: Reduced depth range for subtle chorus effect
    // Map parameter [0, 1] to actual depth [0, 0.5] samples
    constexpr float maxDepthSamples = 0.5f;

    // Clamp to [0, 1] range
    if (amount < 0.0f) amount = 0.0f;
    if (amount > 1.0f) amount = 1.0f;

    float depthSamples = amount * maxDepthSamples;
    modulationDepth_.store(depthSamples, std::memory_order_relaxed);
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-modulation>
void FDNCore::setModulationRate(float hz)
{
    // v1.1: Accept rate directly (logarithmic scaling done in parameter mapping)
    // Clamp to reasonable range [0.01, 2 Hz] for natural modulation
    if (hz < 0.01f) hz = 0.01f;
    if (hz > 2.0f) hz = 2.0f;

    modulationFreq_.store(hz, std::memory_order_relaxed);
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-05>
void FDNCore::setEQEnabled(bool lowShelf, bool highShelf, bool lowpass)
{
    eqLowShelfEnabled_.store(lowShelf, std::memory_order_relaxed);
    eqHighShelfEnabled_.store(highShelf, std::memory_order_relaxed);
    eqLowpassEnabled_.store(lowpass, std::memory_order_relaxed);
    eqNeedsUpdate_ = true;
}

void FDNCore::setEQFrequencies(float lowFreq, float highFreq, float cutoff)
{
    eqLowFreq_.store(lowFreq, std::memory_order_relaxed);
    eqHighFreq_.store(highFreq, std::memory_order_relaxed);
    eqCutoff_.store(cutoff, std::memory_order_relaxed);
    eqNeedsUpdate_ = true;
}

void FDNCore::setEQGains(float lowGain, float highGain)
{
    eqLowGain_.store(lowGain, std::memory_order_relaxed);
    eqHighGain_.store(highGain, std::memory_order_relaxed);
    eqNeedsUpdate_ = true;
}

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-05>
void FDNCore::updateEQFilters()
{
    const float lowFreq = eqLowFreq_.load(std::memory_order_relaxed);
    const float highFreq = eqHighFreq_.load(std::memory_order_relaxed);
    const float cutoffFreq = eqCutoff_.load(std::memory_order_relaxed);
    const float lowGain = eqLowGain_.load(std::memory_order_relaxed);
    const float highGain = eqHighGain_.load(std::memory_order_relaxed);

    for (int i = 0; i < FDNConfig::N; i++) {
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

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-05>
float FDNCore::processEQ(int lineIndex, float input)
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
void FDNCore::updateDelayLengths()
{
    // Get current size multiplier
    float sizeMult = sizeMultiplier_.load(std::memory_order_relaxed);

    // Compute delay lengths with size multiplier applied
    DelayLengthCalculator::computeAllDelayLengths(scaledDelayLengths_, currentSampleRate_, sizeMult);

    // Update delay line buffer sizes
    for (int i = 0; i < FDNConfig::N; i++) {
        delayLines_[i].SampleDelay = scaledDelayLengths_[i];
    }
}

} // namespace FDN
} // namespace Cloudseed
