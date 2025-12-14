// ADC-IMPLEMENTS: <reverbv1-topology-algo-04> (consolidated into reverbv1-architecture-adc-001)
//
// Gain Staging Module Implementation
//
// Contract: reverbv1-architecture-adc-001 (consolidated from reverbv1-architecture-adc-001)
// Purpose: Reduce feedback loop energy independently of damping coefficient
//
// This module provides simple gain multiplication to reduce feedback loop energy.
// All implementation is in the header file (header-only class, matching CloudSeed style).
//
// Key Features:
// - dB to linear conversion: gainLinear = 10^(gainDB/20)
// - Default: -6.1 dB (0.494 linear gain)
// - Range: -12.0 to 0.0 dB
// - 10ms parameter smoothing to prevent zipper noise
// - Real-time safe (no allocations in Process())
//
// Integration with ReverbChannel:
// - Apply gain staging after HPF/LPF processing
// - Before scalar damping and injection into late diffusion
// - Contributes to overall loop stability: gainStaging × crossfeedAmount × damping < 1.0
//
// Test Configurations (per contract):
// - Unity Gain (0.0 dB): Unstable baseline test
// - Minimal Attenuation (-3.0 dB): Marginal stability test
// - Default Safe (-6.1 dB): Recommended production setting
// - Conservative (-9.0 dB): High-safety margin
// - Over-Damped (-12.0 dB): Stable but may reduce crossfeed effect

#include "GainStaging.h"

// Note: All implementation is in the header file following CloudSeed's header-only pattern
// for simple DSP modules (see Lp1.h, Hp1.h as references).
