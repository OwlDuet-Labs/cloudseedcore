/*
Tier 1 Mathematical Foundation Tests for FDN
Runtime: <5 seconds (blocking, must pass 100%)

ADC-TESTS: <reverb-v1-fdn-metrics-tier1-01>
ADC-TESTS: <reverb-v1-fdn-topology-algo-01>
ADC-TESTS: <reverb-v1-fdn-topology-algo-coprime>
ADC-TESTS: <reverb-v1-fdn-topology-algo-injection>
*/

#include "FDNCore.h"
#include "HadamardTransform.h"
#include "DelayConfig.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <numeric>

using namespace Cloudseed::FDN;

// ANSI color codes for terminal output
#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET "\033[0m"

struct TestResult {
    bool passed;
    std::string name;
    std::string message;
    double duration_ms;
};

class Tier1Tests {
private:
    std::vector<TestResult> results;

    double getTimeMs() {
        return std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now().time_since_epoch()
        ).count();
    }

    void recordResult(bool passed, const std::string& name, const std::string& message, double duration_ms) {
        results.push_back({passed, name, message, duration_ms});

        const char* status = passed ? COLOR_GREEN "PASS" COLOR_RESET : COLOR_RED "FAIL" COLOR_RESET;
        std::cout << status << ": " << name << " (" << std::fixed << std::setprecision(2) << duration_ms << "ms)";
        if (!passed) {
            std::cout << "\n      " << COLOR_RED << message << COLOR_RESET;
        }
        std::cout << std::endl;
    }

public:
    // ADC-TESTS: <reverb-v1-fdn-metrics-tier1-01>
    // T1.1: Hadamard Orthogonality Test
    void testHadamardOrthogonality() {
        double start = getTimeMs();

        // Create 8×8 identity input (one impulse at a time)
        float identity[FDNConfig::N][FDNConfig::N] = {0};
        float result[FDNConfig::N][FDNConfig::N] = {0};

        // For each column of identity matrix
        for (int col = 0; col < FDNConfig::N; col++) {
            float input[FDNConfig::N] = {0};
            input[col] = 1.0f;

            float output[FDNConfig::N];
            HadamardTransform::apply(input, output);

            // Store in result matrix
            for (int row = 0; row < FDNConfig::N; row++) {
                result[row][col] = output[row];
            }
        }

        // Compute H × H^T (should equal I × scaling factor)
        // H is normalized by 1/sqrt(8), so H × H^T = (1/sqrt(8))^2 × I = (1/8) × I
        float product[FDNConfig::N][FDNConfig::N] = {0};
        for (int i = 0; i < FDNConfig::N; i++) {
            for (int j = 0; j < FDNConfig::N; j++) {
                for (int k = 0; k < FDNConfig::N; k++) {
                    product[i][j] += result[i][k] * result[j][k];
                }
            }
        }

        // Expected: Diagonal = 1.0, Off-diagonal = 0.0 (after accounting for 1/sqrt(8) scaling)
        // Since H = (1/sqrt(8)) × H_unscaled, H × H^T = (1/8) × I
        // But we want to verify H_unscaled × H_unscaled^T = 8 × I (unnormalized)
        // Actually, the normalized version should give us I directly if done correctly

        // Let's check: H is scaled by 1/sqrt(8), so H × H^T should give us I
        const float tolerance = 1e-5f;
        bool passed = true;
        std::string message;

        for (int i = 0; i < FDNConfig::N; i++) {
            for (int j = 0; j < FDNConfig::N; j++) {
                float expected = (i == j) ? 1.0f : 0.0f;
                float error = std::abs(product[i][j] - expected);

                if (error > tolerance) {
                    passed = false;
                    message = "H×H^T[" + std::to_string(i) + "," + std::to_string(j) + "] = " +
                             std::to_string(product[i][j]) + ", expected " + std::to_string(expected) +
                             " (error: " + std::to_string(error) + ")";
                    break;
                }
            }
            if (!passed) break;
        }

        double duration = getTimeMs() - start;
        recordResult(passed, "T1.1 Hadamard Orthogonality (H×H^T = I)", message, duration);
    }

    // ADC-TESTS: <reverb-v1-fdn-metrics-tier1-02>
    // T1.2: Eigenvalue Magnitude Test
    void testEigenvalueMagnitude() {
        double start = getTimeMs();

        // For Hadamard matrix, eigenvalues are all ±1, so magnitude = 1.0
        float maxEigenvalue = HadamardTransform::getMaxEigenvalueMagnitude();

        const float tolerance = 1e-6f;
        bool passed = std::abs(maxEigenvalue - 1.0f) < tolerance;

        std::string message;
        if (!passed) {
            message = "Max eigenvalue magnitude = " + std::to_string(maxEigenvalue) +
                     ", expected 1.0 (error: " + std::to_string(std::abs(maxEigenvalue - 1.0f)) + ")";
        }

        double duration = getTimeMs() - start;
        recordResult(passed, "T1.2 Eigenvalue Magnitude (≤ 1.0)", message, duration);
    }

    // ADC-TESTS: <reverb-v1-fdn-metrics-tier1-03>
    // T1.3: Energy Preservation Test
    void testEnergyPreservation() {
        double start = getTimeMs();

        // Test multiple random inputs
        const int numTests = 100;
        bool passed = true;
        std::string message;
        const float tolerance = 0.01f; // ±0.01 dB tolerance

        for (int test = 0; test < numTests; test++) {
            // Generate random input with known energy
            float input[FDNConfig::N];
            float inputEnergy = 0.0f;

            for (int i = 0; i < FDNConfig::N; i++) {
                input[i] = (rand() / (float)RAND_MAX) * 2.0f - 1.0f; // [-1, 1]
                inputEnergy += input[i] * input[i];
            }

            // Apply Hadamard transform
            float output[FDNConfig::N];
            HadamardTransform::apply(input, output);

            // Measure output energy
            float outputEnergy = 0.0f;
            for (int i = 0; i < FDNConfig::N; i++) {
                outputEnergy += output[i] * output[i];
            }

            // Energy should be preserved (ratio should be ~1.0)
            float energyRatio = outputEnergy / (inputEnergy + 1e-10f);
            float errorDb = 20.0f * std::log10(energyRatio);

            if (std::abs(errorDb) > tolerance) {
                passed = false;
                message = "Test " + std::to_string(test) + ": Energy ratio = " +
                         std::to_string(energyRatio) + " (" + std::to_string(errorDb) + " dB)";
                break;
            }
        }

        double duration = getTimeMs() - start;
        recordResult(passed, "T1.3 Energy Preservation (||out|| = ||in||, " + std::to_string(numTests) + " tests)", message, duration);
    }

    // ADC-TESTS: <reverb-v1-fdn-metrics-tier1-04>
    // T1.4: Delay Coprimality Test
    void testDelayCoprimality() {
        double start = getTimeMs();

        // Test all standard sample rates
        const double sampleRates[] = {44100.0, 48000.0, 88200.0, 96000.0, 176400.0, 192000.0};
        const int numRates = sizeof(sampleRates) / sizeof(sampleRates[0]);

        bool passed = true;
        std::string message;

        for (int r = 0; r < numRates; r++) {
            double sr = sampleRates[r];
            int delays[FDNConfig::N];

            DelayLengthCalculator::computeAllDelayLengths(delays, sr);

            // Check all pairs are coprime
            for (int i = 0; i < FDNConfig::N; i++) {
                for (int j = i + 1; j < FDNConfig::N; j++) {
                    int g = std::gcd(delays[i], delays[j]);

                    if (g != 1) {
                        passed = false;
                        message = "@ " + std::to_string((int)sr) + " Hz: gcd(" +
                                 std::to_string(delays[i]) + ", " + std::to_string(delays[j]) +
                                 ") = " + std::to_string(g) + " (expected 1)";
                        break;
                    }
                }
                if (!passed) break;
            }
            if (!passed) break;
        }

        double duration = getTimeMs() - start;
        recordResult(passed, "T1.4 Delay Coprimality (gcd = 1, " + std::to_string(numRates) + " sample rates)", message, duration);
    }

    // ADC-TESTS: <reverb-v1-fdn-metrics-tier1-05>
    // T1.5: Divergent Injection Test
    void testDivergentInjection() {
        double start = getTimeMs();

        // Note: This test verifies the injection strategy conceptually
        // Full process() test requires complete CloudSeedCore dependencies
        // For now, we verify the configuration is correct

        // Verify delay line assignment from DelayConfig
        bool passed = true;
        std::string message;

        // Lines 0-3 should be assigned to L channel
        // Lines 4-7 should be assigned to R channel
        // This is specified in DelayConfig::getChannelAssignment()

        for (int i = 0; i < FDNConfig::N / 2; i++) {
            if (FDNConfig::getChannelAssignment(i) != FDNConfig::Channel::LEFT) {
                passed = false;
                message = "Line " + std::to_string(i) + " not assigned to LEFT channel";
                break;
            }
        }

        if (passed) {
            for (int i = FDNConfig::N / 2; i < FDNConfig::N; i++) {
                if (FDNConfig::getChannelAssignment(i) != FDNConfig::Channel::RIGHT) {
                    passed = false;
                    message = "Line " + std::to_string(i) + " not assigned to RIGHT channel";
                    break;
                }
            }
        }

        double duration = getTimeMs() - start;
        recordResult(passed, "T1.5 Divergent Injection (L→lines 0-3, R→lines 4-7)", message, duration);
    }

    // Run all Tier 1 tests
    bool runAll() {
        std::cout << "\n" << COLOR_YELLOW << "=== Tier 1 Mathematical Foundation Tests ===" << COLOR_RESET << "\n";
        std::cout << "Runtime: <5 seconds (blocking, must pass 100%)\n\n";

        testHadamardOrthogonality();
        testEigenvalueMagnitude();
        testEnergyPreservation();
        testDelayCoprimality();
        testDivergentInjection();

        // Summary
        int passed = 0, failed = 0;
        double totalTime = 0.0;

        for (const auto& r : results) {
            if (r.passed) passed++;
            else failed++;
            totalTime += r.duration_ms;
        }

        std::cout << "\n" << COLOR_YELLOW << "=== Summary ===" << COLOR_RESET << "\n";
        std::cout << "Total: " << results.size() << " tests\n";
        std::cout << COLOR_GREEN << "Passed: " << passed << COLOR_RESET << "\n";

        if (failed > 0) {
            std::cout << COLOR_RED << "Failed: " << failed << COLOR_RESET << "\n";
        }

        std::cout << "Total time: " << std::fixed << std::setprecision(2) << totalTime << "ms\n";

        if (failed == 0) {
            std::cout << "\n" << COLOR_GREEN << "✓ All Tier 1 tests PASSED - Ready for Tier 2" << COLOR_RESET << "\n";
            return true;
        } else {
            std::cout << "\n" << COLOR_RED << "✗ BLOCKING: Fix failed tests before proceeding" << COLOR_RESET << "\n";
            return false;
        }
    }
};

int main(int argc, char* argv[]) {
    Tier1Tests tests;
    bool success = tests.runAll();
    return success ? 0 : 1;
}
