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

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

namespace Cloudseed
{
	// ADC-IMPLEMENTS: <reverbv1-topology-algo-04> (consolidated into reverbv1-cloudseeddspcore-adc-002)
	//
	// Gain Staging Module - Reduces feedback loop energy independently of damping coefficient
	//
	// Contract: reverbv1-cloudseeddspcore-adc-002 (consolidated from reverbv1-crossfeed-topology-adc-004)
	// Purpose: Apply simple multiplication by linear gain factor derived from dB parameter
	//          to reduce feedback loop energy and prevent instability
	//
	// Formula: gainLinear = 10^(gainStaging_dB / 20)
	// Default: -6.1 dB = 0.494 linear gain
	// Range: -12.0 to 0.0 dB
	//
	// Parameter smoothing: 10ms ramp time to prevent zipper noise
	// Real-time safe: No allocations in Process()
	class GainStaging
	{
	private:
		float fs;                    // Sample rate
		float gainLinear_;           // Current linear gain (computed from dB)
		float targetGainLinear_;     // Target linear gain for smoothing
		float smoothingCoeff_;       // Smoothing coefficient for parameter changes
		float gainDB_;               // Current gain in dB

		// Update smoothing coefficient based on sample rate
		// 10ms ramp time for parameter changes
		void UpdateSmoothingCoeff()
		{
			// Time constant for 10ms ramp
			const float smoothingTimeMs = 10.0f;
			const float smoothingTimeSamples = (smoothingTimeMs * 0.001f) * fs;

			// First-order smoothing coefficient
			smoothingCoeff_ = 1.0f - expf(-1.0f / smoothingTimeSamples);
		}

	public:
		GainStaging()
		{
			fs = 48000.0f;
			gainDB_ = -6.1f;              // Default: -6.1 dB per contract
			gainLinear_ = 0.494f;         // 10^(-6.1/20) ≈ 0.494
			targetGainLinear_ = gainLinear_;
			UpdateSmoothingCoeff();
		}

		float GetSamplerate()
		{
			return fs;
		}

		void SetSamplerate(float samplerate)
		{
			fs = samplerate;
			UpdateSmoothingCoeff();
		}

		// Get current gain in dB
		float GetGainDB()
		{
			return gainDB_;
		}

		// Set gain in dB (range: -12.0 to 0.0 dB)
		// Conversion: gainLinear = 10^(gainDB / 20)
		void SetGainDB(float gainDB)
		{
			// Clamp to valid range (no std::clamp in C++14)
			if (gainDB < -12.0f)
				gainDB = -12.0f;
			if (gainDB > 0.0f)
				gainDB = 0.0f;

			gainDB_ = gainDB;

			// Compute target linear gain: gainLinear = 10^(gainDB / 20)
			targetGainLinear_ = powf(10.0f, gainDB / 20.0f);
		}

		// Get current linear gain value (for debugging/validation)
		float GetGainLinear()
		{
			return gainLinear_;
		}

		// Clear state (reset to target gain immediately)
		void Reset()
		{
			gainLinear_ = targetGainLinear_;
		}

		// Process single sample with smoothed gain
		float Process(float input)
		{
			// Smooth gain parameter changes (10ms ramp)
			gainLinear_ += (targetGainLinear_ - gainLinear_) * smoothingCoeff_;

			// Apply gain
			return input * gainLinear_;
		}

		// Process buffer with smoothed gain
		void Process(float* input, float* output, int len)
		{
			for (int i = 0; i < len; i++)
			{
				// Smooth gain parameter changes per sample
				gainLinear_ += (targetGainLinear_ - gainLinear_) * smoothingCoeff_;

				// Apply gain
				output[i] = input[i] * gainLinear_;
			}
		}

		// In-place processing
		void Process(float* buffer, int len)
		{
			for (int i = 0; i < len; i++)
			{
				// Smooth gain parameter changes per sample
				gainLinear_ += (targetGainLinear_ - gainLinear_) * smoothingCoeff_;

				// Apply gain
				buffer[i] *= gainLinear_;
			}
		}
	};
}
