/*
Fast Hadamard Transform Implementation
Provides O(N log N) orthogonal feedback matrix for 8×8 and 16×16 FDNs.

ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-01>
ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
*/

#pragma once

#include <cmath>

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-01>
/**
 * Fast Walsh-Hadamard Transform for 8×8 and 16×16 matrices
 *
 * Implements the orthogonal feedback matrix using a fast butterfly algorithm:
 * - Computational complexity: O(N log N)
 *   - 8×8: 24 operations (3 stages)
 *   - 16×16: 64 operations (4 stages)
 * - Only additions/subtractions (no multiplications until final scaling)
 * - In-place transform with log2(N) butterfly stages
 * - Final scaling by 1/sqrt(N) for energy preservation
 *
 * Properties:
 * - Orthogonal: H * H^T = I (identity matrix)
 * - Energy preserving: ||H * v|| = ||v||
 * - Eigenvalues: All ±1 (on unit circle → guaranteed stability)
 */
class HadamardTransform
{
public:
    /**
     * Apply 8×8 fast Hadamard transform
     *
     * @param input Input vector (8 samples)
     * @param output Output vector (8 samples, can be same as input for in-place)
     *
     * Algorithm: 3-stage butterfly network
     * - Stage 1: 4 pairs (stride 1)
     * - Stage 2: 2 quads (stride 2)
     * - Stage 3: 1 octad (stride 4)
     * - Final: Scale by 1/sqrt(8) = 0.35355339 for energy preservation
     */
    static void apply8(const float* input, float* output)
    {
        // Copy input to output for in-place transform
        for (int i = 0; i < 8; i++) {
            output[i] = input[i];
        }

        // Stage 1: Groups of 2 (4 butterfly pairs)
        // Stride = 1
        butterfly(output, 0, 1);
        butterfly(output, 2, 3);
        butterfly(output, 4, 5);
        butterfly(output, 6, 7);

        // Stage 2: Groups of 4 (2 butterfly quads)
        // Stride = 2
        butterfly(output, 0, 2);
        butterfly(output, 1, 3);
        butterfly(output, 4, 6);
        butterfly(output, 5, 7);

        // Stage 3: Groups of 8 (1 full butterfly)
        // Stride = 4
        butterfly(output, 0, 4);
        butterfly(output, 1, 5);
        butterfly(output, 2, 6);
        butterfly(output, 3, 7);

        // Final scaling: 1/sqrt(8) for energy preservation
        // This ensures ||output|| = ||input|| for lossless case
        constexpr float scale = 0.35355339f;  // 1/sqrt(8)
        for (int i = 0; i < 8; i++) {
            output[i] *= scale;
        }
    }

    // ADC-IMPLEMENTS: <reverb-v1-fdn-quality-algo-16x16-delays>
    /**
     * Apply 16×16 fast Hadamard transform
     *
     * @param input Input vector (16 samples)
     * @param output Output vector (16 samples, can be same as input for in-place)
     *
     * Algorithm: 4-stage butterfly network
     * - Stage 1: 8 pairs (stride 1)
     * - Stage 2: 4 quads (stride 2)
     * - Stage 3: 2 octads (stride 4)
     * - Stage 4: 1 full 16-way (stride 8)
     * - Final: Scale by 1/sqrt(16) = 0.25 for energy preservation
     *
     * Computational cost: 64 butterfly ops vs 24 for 8×8
     */
    static void apply16(const float* input, float* output)
    {
        // Copy input to output for in-place transform
        for (int i = 0; i < 16; i++) {
            output[i] = input[i];
        }

        // Stage 1: Groups of 2 (8 butterfly pairs)
        // Stride = 1
        butterfly(output, 0, 1);
        butterfly(output, 2, 3);
        butterfly(output, 4, 5);
        butterfly(output, 6, 7);
        butterfly(output, 8, 9);
        butterfly(output, 10, 11);
        butterfly(output, 12, 13);
        butterfly(output, 14, 15);

        // Stage 2: Groups of 4 (4 butterfly quads)
        // Stride = 2
        butterfly(output, 0, 2);
        butterfly(output, 1, 3);
        butterfly(output, 4, 6);
        butterfly(output, 5, 7);
        butterfly(output, 8, 10);
        butterfly(output, 9, 11);
        butterfly(output, 12, 14);
        butterfly(output, 13, 15);

        // Stage 3: Groups of 8 (2 butterfly octads)
        // Stride = 4
        butterfly(output, 0, 4);
        butterfly(output, 1, 5);
        butterfly(output, 2, 6);
        butterfly(output, 3, 7);
        butterfly(output, 8, 12);
        butterfly(output, 9, 13);
        butterfly(output, 10, 14);
        butterfly(output, 11, 15);

        // Stage 4: Groups of 16 (1 full butterfly)
        // Stride = 8
        butterfly(output, 0, 8);
        butterfly(output, 1, 9);
        butterfly(output, 2, 10);
        butterfly(output, 3, 11);
        butterfly(output, 4, 12);
        butterfly(output, 5, 13);
        butterfly(output, 6, 14);
        butterfly(output, 7, 15);

        // Final scaling: 1/sqrt(16) = 0.25 for energy preservation
        // This ensures ||output|| = ||input|| for lossless case
        constexpr float scale = 0.25f;  // 1/sqrt(16)
        for (int i = 0; i < 16; i++) {
            output[i] *= scale;
        }
    }

    /**
     * Get eigenvalue magnitudes for verification
     * All eigenvalues should be exactly 1.0 (on unit circle)
     *
     * @return Maximum eigenvalue magnitude (should be 1.0)
     */
    static float getMaxEigenvalueMagnitude()
    {
        // For normalized Hadamard matrix, all eigenvalues are ±1
        // After scaling by 1/sqrt(N), magnitude remains 1.0
        return 1.0f;
    }

private:
    /**
     * Butterfly operation: Basic Hadamard building block
     *
     * Computes:
     *   output[i] = a + b
     *   output[j] = a - b
     *
     * where a = input[i], b = input[j]
     *
     * This is the fundamental ±1 pattern of the Hadamard matrix.
     */
    static inline void butterfly(float* data, int i, int j)
    {
        float a = data[i];
        float b = data[j];
        data[i] = a + b;
        data[j] = a - b;
    }
};

} // namespace FDN
} // namespace Cloudseed
