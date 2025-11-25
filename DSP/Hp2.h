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
	// 2nd-order Butterworth highpass filter for crossfeed DC blocking and subsonic rejection
	// Provides -12 dB/octave rolloff below cutoff frequency
	//
	// ADC Contract Specification:
	// - Type: 2nd-order Butterworth (maximally flat passband)
	// - Default cutoff: 40 Hz (subsonic + DC rejection for crossfeed path)
	// - Rolloff: -12 dB/octave (stronger than 1st-order Hp1)
	// - Purpose: Prevent low-frequency modal buildup in feedback loop
	// - Attenuation @ 20 Hz: ≥12 dB (measured against 1 kHz passband)
	//
	// Implementation: Standard biquad (Direct Form I)
	// Q = 1/sqrt(2) = 0.7071 (Butterworth Q factor for maximally flat response)
	class Hp2
	{
	private:
		float fs;

		// Biquad coefficients (normalized by a0)
		float b0, b1, b2;  // Feedforward (numerator)
		float a1, a2;      // Feedback (denominator, a0=1 after normalization)

		// State variables (Direct Form I)
		float x1, x2;      // Input delay states
		float y1, y2;      // Output delay states

		float cutoffHz;

	public:
		float Output;

		Hp2()
		{
			fs = 48000;
			b0 = 1;
			b1 = 0;
			b2 = 0;
			a1 = 0;
			a2 = 0;
			x1 = 0;
			x2 = 0;
			y1 = 0;
			y2 = 0;
			cutoffHz = 100;  // Match Hp1 default (will be set to 40 Hz in ReverbChannel)
		}

		float GetSamplerate()
		{
			return fs;
		}

		void SetSamplerate(float samplerate)
		{
			fs = samplerate;
		}

		float GetCutoffHz()
		{
			return cutoffHz;
		}

		void SetCutoffHz(float hz)
		{
			cutoffHz = hz;
			Update();
		}

		void ClearBuffers()
		{
			x1 = 0;
			x2 = 0;
			y1 = 0;
			y2 = 0;
			Output = 0;
		}

		void Update()
		{
			// Prevent going over the Nyquist frequency
			if (cutoffHz >= fs * 0.5f)
				cutoffHz = fs * 0.499f;

			// Standard 2nd-order Butterworth highpass design
			// Q = 1/sqrt(2) = 0.7071 for Butterworth (maximally flat passband)
			const float Q = 0.70710678118f;  // sqrt(2)/2

			// Bilinear transform frequency warping
			float w0 = 2.0f * M_PI * cutoffHz / fs;
			float cosw0 = cosf(w0);
			float sinw0 = sinf(w0);
			float alpha = sinw0 / (2.0f * Q);

			// Butterworth highpass biquad coefficients (unnormalized)
			float b0_unnorm = (1.0f + cosw0) / 2.0f;
			float b1_unnorm = -(1.0f + cosw0);
			float b2_unnorm = (1.0f + cosw0) / 2.0f;
			float a0 = 1.0f + alpha;
			float a1_unnorm = -2.0f * cosw0;
			float a2_unnorm = 1.0f - alpha;

			// Normalize by a0
			b0 = b0_unnorm / a0;
			b1 = b1_unnorm / a0;
			b2 = b2_unnorm / a0;
			a1 = a1_unnorm / a0;
			a2 = a2_unnorm / a0;
		}

		float Process(float input)
		{
			// Optimization: If input is zero and state variables are near zero,
			// skip processing to prevent denormal numbers
			if (input == 0 && fabsf(y1) < 0.000001f && fabsf(y2) < 0.000001f)
			{
				Output = 0;
				x1 = 0;
				x2 = 0;
				y1 = 0;
				y2 = 0;
			}
			else
			{
				// Direct Form I biquad implementation
				// y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
				Output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

				// Update state variables
				x2 = x1;
				x1 = input;
				y2 = y1;
				y1 = Output;
			}

			return Output;
		}

		void Process(float* input, float* output, int len)
		{
			for (int i = 0; i < len; i++)
				output[i] = Process(input[i]);
		}
	};
}
