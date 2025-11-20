/*
FDN Core Processing Engine
8x8 Feedback Delay Network with Hadamard matrix, modulation, and per-line EQ.

ADC-IMPLEMENTS: <reverb-v1-fdn-topology-datamodel-01>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-impl-01>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-injection>
*/

#pragma once

// Define CloudSeedCore constants if not already defined
#ifndef BUFFER_SIZE
#define BUFFER_SIZE 512
#endif

#include <array>
#include <atomic>
#include <cmath>
#include "../ModulatedDelay.h"
#include "../Biquad.h"
#include "DelayConfig.h"
#include "HadamardTransform.h"

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-datamodel-01>
/**
 * FDNCore: 8x8 Feedback Delay Network
 *
 * Architecture:
 * - 8 delay lines with modulated read positions
 * - Per-line EQ (low shelf, high shelf, lowpass)
 * - Hadamard 8x8 orthogonal feedback matrix
 * - Divergent stereo injection (L→lines 0-3, R→lines 4-7)
 * - Unified decay control via feedback gain
 *
 * Signal Flow (per sample):
 * 1. Read from all delay lines → apply modulation + EQ
 * 2. Apply Hadamard matrix to delay outputs
 * 3. Scale by decay gain
 * 4. Add input injection (L/R split) and write back to delays
 * 5. Extract outputs (sum lines 0-3 for L, 4-7 for R)
 *
 * Key Properties:
 * - Guaranteed stability (orthogonal matrix → eigenvalues on unit circle)
 * - Natural stereo coupling (matrix feedback, no explicit crossfeed)
 * - Coprime delay lengths (prevents comb filtering)
 * - Real-time safe (no allocations in process())
 */
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
    void setModulationAmount(float amount);  // 0-1
    void setModulationRate(float hz);
    void setEQEnabled(bool lowShelf, bool highShelf, bool lowpass);
    void setEQFrequencies(float lowFreq, float highFreq, float cutoff);
    void setEQGains(float lowGain, float highGain);

private:
    // Delay lines (8 total, using ModulatedDelay directly)
    std::array<ModulatedDelay, FDNConfig::N> delayLines_;

    // Modulation (8 LFO phases, one per line)
    std::array<float, FDNConfig::N> lfoPhases_;

    // EQ (3 filters per line: low shelf, high shelf, lowpass)
    std::array<Biquad, FDNConfig::N> lowShelfFilters_;
    std::array<Biquad, FDNConfig::N> highShelfFilters_;
    std::array<Biquad, FDNConfig::N> lowpassFilters_;

    // Processing buffers (reused each block to avoid allocations)
    std::array<float, FDNConfig::N> delayOutputs_;
    std::array<float, FDNConfig::N> feedbackSignals_;
    std::array<float, FDNConfig::N> feedbackInput_;  // Input prepared for next delay iteration

    // Parameters (atomic for thread-safe updates from message thread)
    std::atomic<float> decayGain_;
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

    // State
    double currentSampleRate_;
    int scaledDelayLengths_[FDNConfig::N];
    bool eqNeedsUpdate_;

    // Internal methods
    void updateDelayLengths();
    void updateEQFilters();
    float computeDecayGain(float rt60Seconds);
    float processEQ(int lineIndex, float input);
    float advanceLFO(int lineIndex);
};

} // namespace FDN
} // namespace Cloudseed
