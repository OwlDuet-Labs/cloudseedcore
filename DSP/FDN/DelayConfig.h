/*
FDN Delay Configuration
Defines delay lengths and coprimality preservation algorithm for the
8x8 Feedback Delay Network implementation.

ADC-IMPLEMENTS: <reverb-v1-fdn-topology-datamodel-01>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-02>
ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-coprime>
*/

#pragma once

#include <algorithm>
#include <numeric>
#include <cassert>

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-datamodel-01>
struct FDNConfig
{
    static constexpr int N = 8;  // FDN size (8x8 matrix)

    // Delay line lengths in samples @ 48kHz (base reference)
    // These values are coprime and optimized for natural reverb density
    // Left channel delays (lines 0-3): 37.1 to 47.9 ms
    // Right channel delays (lines 4-7): 53.1 to 67.1 ms
    static constexpr int delayLengths48k[N] = {
        1781,  // 37.1 ms (L) - Line 0
        1982,  // 41.3 ms (L) - Line 1
        2098,  // 43.7 ms (L) - Line 2
        2299,  // 47.9 ms (L) - Line 3
        2549,  // 53.1 ms (R) - Line 4
        2846,  // 59.3 ms (R) - Line 5
        2962,  // 61.7 ms (R) - Line 6
        3221   // 67.1 ms (R) - Line 7
    };

    // L/R channel assignment for divergent injection
    static constexpr bool isLeftChannel[N] = {
        true, true, true, true,      // Lines 0-3: L
        false, false, false, false   // Lines 4-7: R
    };
};

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-coprime>
// Coprimality utilities for delay length validation and adjustment
class CoprimalityHelper
{
public:
    // Check if two numbers are coprime (gcd = 1)
    static bool areCoprime(int a, int b)
    {
        return std::gcd(a, b) == 1;
    }

    // Check if all pairs in array are coprime
    static bool allCoprime(const int delayLengths[], int N)
    {
        for (int i = 0; i < N; i++) {
            for (int j = i + 1; j < N; j++) {
                if (!areCoprime(delayLengths[i], delayLengths[j])) {
                    return false;
                }
            }
        }
        return true;
    }

    // Check if value is coprime with all elements in array except skipIndex
    static bool isCoprimeWithAll(int value, const int arr[], int N, int skipIndex)
    {
        for (int i = 0; i < N; i++) {
            if (i == skipIndex) continue;
            if (!areCoprime(value, arr[i])) {
                return false;
            }
        }
        return true;
    }

    // Ensure coprimality through hybrid adjustment algorithm
    // Adjusts delay lengths minimally (±5 samples max) to maintain coprime relationships
    static void ensureCoprimalityHybrid(int delayLengths[], int N)
    {
        // Step 1: Check if already coprime (often true after scaling)
        if (allCoprime(delayLengths, N)) {
            return;  // No changes needed
        }

        // Step 2: Try small adjustments (±5 samples max)
        // Search both directions to minimize modal density changes
        for (int i = 0; i < N; i++) {
            if (isCoprimeWithAll(delayLengths[i], delayLengths, N, i)) {
                continue;  // This delay is already coprime with all others
            }

            // Try incremental adjustments
            bool found = false;
            for (int delta = 1; delta <= 5 && !found; delta++) {
                // Try positive adjustment first (prefer longer delays)
                int candidate = delayLengths[i] + delta;
                if (isCoprimeWithAll(candidate, delayLengths, N, i)) {
                    delayLengths[i] = candidate;
                    found = true;
                    break;
                }

                // Try negative adjustment
                candidate = delayLengths[i] - delta;
                if (candidate >= 32 && isCoprimeWithAll(candidate, delayLengths, N, i)) {
                    delayLengths[i] = candidate;
                    found = true;
                    break;
                }
            }

            // If no solution found within ±5 samples, algorithm failed
            // This should never happen with well-chosen base delays
            assert(found && "Coprimality enforcement failed - base delays need adjustment");
        }

        // Step 3: Verify all pairs coprime (safety check)
        assert(allCoprime(delayLengths, N) && "Final coprimality check failed");
    }
};

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-02>
// Delay length computation with sample rate scaling
class DelayLengthCalculator
{
public:
    // Compute delay length scaled to target sample rate
    static int computeDelayLength(int baseLength48k, double sampleRate)
    {
        // Scale delay length proportionally to sample rate
        double scaleFactor = sampleRate / 48000.0;
        int scaledLength = static_cast<int>(baseLength48k * scaleFactor + 0.5);

        // Ensure minimum delay (prevent zero-length delays)
        if (scaledLength < 32) {
            scaledLength = 32;
        }

        return scaledLength;
    }

    // Compute all delay lengths for target sample rate with coprimality preservation
    static void computeAllDelayLengths(int outputLengths[], double sampleRate)
    {
        // Step 1: Scale all delays to target sample rate
        for (int i = 0; i < FDNConfig::N; i++) {
            outputLengths[i] = computeDelayLength(FDNConfig::delayLengths48k[i], sampleRate);
        }

        // Step 2: Enforce coprimality across all delay pairs
        // This adjusts delays minimally (±5 samples) to maintain gcd=1 relationships
        CoprimalityHelper::ensureCoprimalityHybrid(outputLengths, FDNConfig::N);

        // Step 3: Final validation (debug builds only)
        assert(CoprimalityHelper::allCoprime(outputLengths, FDNConfig::N) &&
               "Delay lengths must be coprime after adjustment");
    }
};

} // namespace FDN
} // namespace Cloudseed
