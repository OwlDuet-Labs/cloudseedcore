/*
Copyright (c) 2024 Ghost Note Engineering Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// ADC-IMPLEMENTS: <reverbv1-topology-algo-04> (consolidated into reverbv1-architecture-adc-001)
//
// Decorrelation Module Implementation
//
// Contract: reverbv1-architecture-adc-001 (consolidated from reverbv1-architecture-adc-001)
// Purpose: Break up frequency-specific resonances in feedback path via slight pitch modulation
//
// This module provides delay line modulation to create imperceptible pitch shifting
// that reduces frequency-specific buildup and resonances in the crossfeed path.
// All implementation is in the header file (header-only class, matching CloudSeed style).
//
// Key Features:
// - Variable delay line (~10ms, 480 samples @ 48kHz)
// - Sine LFO @ 0.5 Hz
// - Pitch modulation: ±0.5 cents maximum (imperceptible)
// - Fractional delay with linear interpolation
// - Real-time safe (pre-allocate buffer in Initialize())
//
// Expected Resonance Reduction (per contract):
// - 500 Hz resonance: +2.5 dB → +0.8 dB (-1.7 dB reduction)
// - 1 kHz resonance: +4.0 dB → +1.2 dB (-2.8 dB reduction)
// - 3 kHz resonance: +3.5 dB → +0.6 dB (-2.9 dB reduction)
//
// Performance Impact:
// - CPU: +0.3-0.5% per channel (linear interpolation + sine LFO)
// - Memory: +4 KB per channel (960 samples × 4 bytes)
// - Latency: +10ms (inherent delay, acceptable for reverb tail)
//
// Pitch Shifting Threshold:
// - Operating at ±0.5 cents
// - Perceptual threshold: ~5-10 cents
// - 10-20× below perceptual threshold (imperceptible)
//
// Integration with ReverbChannel:
// - Apply in feedback path after HPF
// - Before LPF and gain staging
// - Toggleable for A/B testing and validation

#include "DecorrelationModule.h"

// Note: All implementation is in the header file following CloudSeed's header-only pattern
// for DSP modules. This allows for better inlining and optimization by the compiler.
