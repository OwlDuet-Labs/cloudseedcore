/*
FDN Compilation Test
Verifies that all FDN components compile successfully.
*/

#include "FDNCore.h"
#include "HadamardTransform.h"
#include "DelayConfig.h"

using namespace Cloudseed::FDN;

int main()
{
    // Test 1: Delay configuration
    int delayLengths[FDNConfig::N];
    DelayLengthCalculator::computeAllDelayLengths(delayLengths, 48000.0);

    // Test 2: Hadamard transform
    float input[8] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float output[8];
    HadamardTransform::apply(input, output);

    // Test 3: FDN Core
    FDNCore fdn;
    fdn.prepare(48000.0, 512);
    fdn.setDecayTime(2.0f);
    fdn.setModulationAmount(0.1f);
    fdn.setModulationRate(1.0f);

    float inL[512] = {0};
    float inR[512] = {0};
    float outL[512];
    float outR[512];

    inL[0] = 1.0f;  // Unit impulse
    fdn.process(inL, inR, outL, outR, 512);

    return 0;
}
