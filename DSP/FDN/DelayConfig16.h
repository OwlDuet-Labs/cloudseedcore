/*
DelayConfig16: 16×16 FDN Delay Length Configuration

Provides coprime delay lengths for 16×16 FDN with extended range (37-74 ms).

Academic Reference: Gardner (1992) Section 4.2

ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
*/

#pragma once

#include <numeric>  // for std::gcd

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
/**
 * FDNConfig16: 16×16 FDN Configuration
 *
 * Delay Length Properties:
 *   - All 16 delays mutually coprime (gcd = 1)
 *   - Range: 37-74 ms @ 48kHz
 *   - Even distribution across delay spectrum
 *   - L/R asymmetry (L shorter than R)
 *
 * Echo Density:
 *   - Coupled echoes: 16² = 256 (vs 8² = 64 for 8×8)
 *   - Modal density: 16 × 55ms = 880ms (vs 8 × 50ms = 400ms)
 *   - Smoother, more diffuse reverb tail
 *
 * CPU Cost:
 *   - Hadamard WHT: 64 ops (vs 24 for 8×8)
 *   - Filter ops: 2× increase (16 lines vs 8)
 *   - Expected: 4-5% CPU @ 48kHz (vs 2-3% for 8×8)
 */
struct FDNConfig16
{
    static constexpr int N = 16;

    // Base delay lengths @ 48kHz (samples)
    // Generated using coprime selection algorithm
    // Range: 1781-3539 samples (37.1-73.7 ms)
    static constexpr int delayLengths48k[N] = {
        // Left channel (lines 0-7): 37-52 ms
        1781,  // 37.1 ms
        1861,  // 38.8 ms
        1951,  // 40.6 ms
        2053,  // 42.8 ms
        2161,  // 45.0 ms
        2269,  // 47.3 ms
        2381,  // 49.6 ms
        2503,  // 52.1 ms

        // Right channel (lines 8-15): 54-74 ms
        2621,  // 54.6 ms
        2749,  // 57.3 ms
        2879,  // 60.0 ms
        3001,  // 62.5 ms
        3137,  // 65.4 ms
        3271,  // 68.1 ms
        3407,  // 71.0 ms
        3539   // 73.7 ms
    };

    // L/R channel assignment
    static constexpr bool isLeftChannel[N] = {
        true, true, true, true, true, true, true, true,      // Lines 0-7: L
        false, false, false, false, false, false, false, false  // Lines 8-15: R
    };

    // Verify all delay lengths are mutually coprime (debug/testing)
    static bool verifyCoprimeDelays()
    {
        for (int i = 0; i < N; i++) {
            for (int j = i + 1; j < N; j++) {
                int gcdValue = std::gcd(delayLengths48k[i], delayLengths48k[j]);
                if (gcdValue != 1) {
                    return false;  // NOT COPRIME - Algorithm failure
                }
            }
        }
        return true;  // All pairs coprime ✓
    }
};

} // namespace FDN
} // namespace Cloudseed
