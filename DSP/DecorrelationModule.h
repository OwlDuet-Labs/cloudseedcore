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
#include <vector>

namespace Cloudseed
{
	// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-algorithm-05>
	//
	// Decorrelation Module - Breaks up frequency-specific resonances via pitch modulation
	//
	// Contract: reverbv1-crossfeed-topology-adc-004, lines 779-908
	// Purpose: Apply slight pitch modulation to reduce frequency-specific buildup
	//          in feedback path without audible artifacts
	//
	// Implementation: Variable delay line with sine LFO modulation
	// - Center delay: ~10ms (480 samples @ 48kHz)
	// - Modulation depth: 0-2 samples (maps to ±0.5 cents max)
	// - Modulation rate: 0.5 Hz (slow enough to avoid pitch perception)
	// - Linear interpolation for fractional delay reads
	//
	// Pitch shifting: At ±0.5 cents, pitch shift is imperceptible
	// - Threshold of pitch perception: ~5-10 cents
	// - Operating at 10-20× below perceptual threshold
	//
	// Real-time safe: Buffer pre-allocated in Initialize()
	class DecorrelationModule
	{
	private:
		std::vector<float> delayBuffer_;    // Circular delay buffer (~20ms capacity)
		std::vector<float> tempBuffer_;     // Pre-allocated temp buffer for in-place processing
		int bufferSize_;                    // Total buffer size in samples
		int writePos_;                      // Current write position
		float modulationPhase_;             // LFO phase (0 to 2π)
		float modulationDepth_;             // 0.0 to 1.0 (maps to ±0.5 cents)
		bool enabled_;                      // Bypass flag
		float fs_;                          // Sample rate

		// Center delay in samples (~10ms)
		static const int centerDelaySamples_ = 480;  // @ 48kHz

		// Modulation rate in Hz
		static constexpr float modulationRateHz_ = 0.5f;

		// Maximum modulation in samples (±2 samples @ depth=1.0)
		static constexpr float maxModulationSamples_ = 2.0f;

	public:
		DecorrelationModule()
		{
			bufferSize_ = 0;
			writePos_ = 0;
			modulationPhase_ = 0.0f;
			modulationDepth_ = 0.0f;
			enabled_ = false;
			fs_ = 48000.0f;
		}

		// Initialize with sample rate and maximum block size
		// Pre-allocates delay buffer for real-time safety
		void Initialize(double sampleRate, int maxBlockSize)
		{
			fs_ = static_cast<float>(sampleRate);

			// Buffer size: center delay + max modulation + safety margin
			// Scale center delay based on sample rate ratio
			float sampleRateRatio = fs_ / 48000.0f;
			int scaledCenterDelay = static_cast<int>(centerDelaySamples_ * sampleRateRatio);

			bufferSize_ = scaledCenterDelay + static_cast<int>(maxModulationSamples_) + 10;

			// Pre-allocate buffers (real-time safe)
			delayBuffer_.resize(bufferSize_);
			tempBuffer_.resize(maxBlockSize);  // FIX: Pre-allocate temp buffer for in-place processing

			Reset();
		}

		// Reset state: clear buffer and reset phase
		void Reset()
		{
			// Clear delay buffer
			for (int i = 0; i < bufferSize_; i++)
				delayBuffer_[i] = 0.0f;

			writePos_ = 0;
			modulationPhase_ = 0.0f;
		}

		// Enable/disable decorrelation (bypass control)
		void SetEnabled(bool enabled)
		{
			enabled_ = enabled;
		}

		// Get enabled state
		bool GetEnabled() const
		{
			return enabled_;
		}

		// Set modulation depth (0.0 to 1.0 maps to ±0.5 cents)
		void SetDepth(float depth)
		{
			// Clamp to valid range (no std::clamp in C++14)
			if (depth < 0.0f)
				depth = 0.0f;
			if (depth > 1.0f)
				depth = 1.0f;

			modulationDepth_ = depth;
		}

		// Get modulation depth
		float GetDepth() const
		{
			return modulationDepth_;
		}

		// Process audio buffer
		// If disabled, performs direct copy (bypass mode)
		// If enabled, applies pitch modulation via delay line modulation
		void Process(const float* input, float* output, int numSamples)
		{
			// Bypass mode: direct copy
			if (!enabled_)
			{
				for (int i = 0; i < numSamples; i++)
					output[i] = input[i];
				return;
			}

			// Active mode: apply pitch modulation
			float sampleRateRatio = fs_ / 48000.0f;
			float scaledCenterDelay = centerDelaySamples_ * sampleRateRatio;
			float maxModulation = maxModulationSamples_ * modulationDepth_;

			// Phase increment per sample: (2π × modulationRate) / sampleRate
			float phaseIncrement = (2.0f * M_PI * modulationRateHz_) / fs_;

			for (int i = 0; i < numSamples; i++)
			{
				// Update LFO phase
				modulationPhase_ += phaseIncrement;
				if (modulationPhase_ > 2.0f * M_PI)
					modulationPhase_ -= 2.0f * M_PI;

				// Compute modulated delay length
				float modulation = maxModulation * sinf(modulationPhase_);
				float delayLength = scaledCenterDelay + modulation;

				// Extract integer and fractional parts for interpolation
				int delayIntegerPart = static_cast<int>(floorf(delayLength));
				float delayFractionalPart = delayLength - delayIntegerPart;

				// Calculate read positions (with circular buffer wrap)
				int readPos1 = writePos_ - delayIntegerPart;
				if (readPos1 < 0)
					readPos1 += bufferSize_;

				int readPos2 = readPos1 - 1;
				if (readPos2 < 0)
					readPos2 += bufferSize_;

				// Read from delay buffer with linear interpolation
				float sample1 = delayBuffer_[readPos1];
				float sample2 = delayBuffer_[readPos2];
				float interpolatedSample = sample1 + delayFractionalPart * (sample2 - sample1);

				// Write input to delay buffer
				delayBuffer_[writePos_] = input[i];

				// Advance write position (circular)
				writePos_ = (writePos_ + 1) % bufferSize_;

				// Output interpolated sample
				output[i] = interpolatedSample;
			}
		}

		// In-place processing variant
		// FIX: Uses pre-allocated tempBuffer_ instead of allocating on every call
		void Process(float* buffer, int numSamples)
		{
			// REAL-TIME SAFE: Uses pre-allocated tempBuffer_ from Initialize()
			Process(buffer, tempBuffer_.data(), numSamples);

			// Copy back to input buffer
			for (int i = 0; i < numSamples; i++)
				buffer[i] = tempBuffer_[i];
		}
	};
}
