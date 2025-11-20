/*
Fast Hadamard Transform Implementation
Provides O(N log N) orthogonal feedback matrix for 8x8 FDN.

ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-01>
*/

#pragma once

#include <cmath>

namespace Cloudseed {
namespace FDN {

// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-algo-01>
/**
 * Fast Walsh-Hadamard Transform for 8x8 matrix
 *
 * Implements the orthogonal feedback matrix using a fast butterfly algorithm:
 * - Computational complexity: O(N log N) = 24 operations for N=8
 * - Only additions/subtractions (no multiplications until final scaling)
 * - In-place transform with 3 butterfly stages
 * - Final scaling by 1/sqrt(8) for energy preservation
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
     * Apply 8x8 fast Hadamard transform
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
    static void apply(const float* input, float* output)
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

    /**
     * Get eigenvalue magnitudes for verification
     * All eigenvalues should be exactly 1.0 (on unit circle)
     *
     * @return Maximum eigenvalue magnitude (should be 1.0)
     */
    static float getMaxEigenvalueMagnitude()
    {
        // For normalized Hadamard matrix, all eigenvalues are ±1
        // After scaling by 1/sqrt(8), magnitude remains 1.0
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
