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

#include <map>
#include <memory>
#include "../Parameters.h"
#include "ModulatedDelay.h"
#include "MultitapDelay.h"
#include "RandomBuffer.h"
#include "Lp1.h"
#include "Hp1.h"
#include "DelayLine.h"
#include "AllpassDiffuser.h"
#include <cmath>
#include "ReverbChannel.h"
#include "Utils.h"

namespace Cloudseed
{
	enum class ChannelLR
	{
		Left,
		Right
	};

	class ReverbChannel
	{
	private:
		static const int TotalLineCount = 12;

		double paramsScaled[Parameter::COUNT] = { 0.0 };
		int samplerate;

		ModulatedDelay preDelay;
		MultitapDelay multitap;
		AllpassDiffuser diffuser;
		DelayLine lines[TotalLineCount];
		RandomBuffer rand;
		Hp1 highPass;
		Lp1 lowPass;

		int delayLineSeed;
		int postDiffusionSeed;

		// Used the the main process loop
		int lineCount;

		bool lowCutEnabled;
		bool highCutEnabled;
		bool multitapEnabled;
		bool diffuserEnabled;
		float inputMix;
		float dryOut;
		float earlyOut;
		float lineOut;
		float crossSeed;
		ChannelLR channelLr;

		// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-datamodel-01>
		// Phase 2: Crossfeed buffer coordination infrastructure
		// Phase 3: Active DSP implementation - buffers populated during processing

		// Input crossfeed buffers (from opposite channel's previous processing)
		const float* crossfeedEarlyInputBuffer_;  // Early reflections from opposite channel
		const float* crossfeedLateInputBuffer_;   // Late diffusion feedback from opposite channel

		// Output buffers (for storing intermediate results to share with opposite channel)
		float earlyOutputBuffer_[BUFFER_SIZE];   // Early reflection output for crossfeed
		float lateOutputBuffer_[BUFFER_SIZE];    // Late diffusion output for crossfeed feedback

		// Enable flags for independent early/late crossfeed control
		bool earlyCrossfeedEnabled_;
		bool lateCrossfeedEnabled_;

		// Buffer size for validation
		int crossfeedBufferSize_;

	public:

		ReverbChannel(int samplerate, ChannelLR leftOrRight) :
			crossfeedEarlyInputBuffer_(nullptr),
			crossfeedLateInputBuffer_(nullptr),
			earlyCrossfeedEnabled_(false),
			lateCrossfeedEnabled_(false),
			crossfeedBufferSize_(0)
		{
			this->channelLr = leftOrRight;
			crossSeed = 0.0;
			lineCount = 8;
			diffuser.SetInterpolationEnabled(true);
			highPass.SetCutoffHz(20);
			lowPass.SetCutoffHz(20000);
			SetSamplerate(samplerate);
		}

		int GetSamplerate()
		{
			return samplerate;
		}

		void SetSamplerate(int samplerate)
		{
			this->samplerate = samplerate;
			highPass.SetSamplerate(samplerate);
			lowPass.SetSamplerate(samplerate);
			diffuser.SetSamplerate(samplerate);

			for (int i = 0; i < TotalLineCount; i++)
				lines[i].SetSamplerate(samplerate);

			ReapplyAllParams();
			ClearBuffers();
			UpdateLines();
		}

		void ReapplyAllParams()
		{
			for (int i = 0; i < Parameter::COUNT; i++)
				SetParameter(i, paramsScaled[i]);
		}

		void SetParameter(int para, double scaledValue)
		{
			paramsScaled[para] = scaledValue;

			switch (para)
			{
			case Parameter::Interpolation:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetInterpolationEnabled(scaledValue >= 0.5);
				break;
			case Parameter::LowCutEnabled:
				lowCutEnabled = scaledValue >= 0.5;
				if (lowCutEnabled)
					highPass.ClearBuffers();
				break;
			case Parameter::HighCutEnabled:
				highCutEnabled = scaledValue >= 0.5;
				if (highCutEnabled)
					lowPass.ClearBuffers();
				break;
			case Parameter::InputMix:
				inputMix = scaledValue;
				break;
			case Parameter::LowCut:
				highPass.SetCutoffHz(scaledValue);
				break;
			case Parameter::HighCut:
				lowPass.SetCutoffHz(scaledValue);
				break;
			case Parameter::DryOut:
				dryOut = scaledValue <= -30 ? 0.0 : Utils::DB2Gainf(scaledValue);
				break;
			case Parameter::EarlyOut:
				earlyOut = scaledValue <= -30 ? 0.0 : Utils::DB2Gainf(scaledValue);
				break;
			case Parameter::LateOut:
				lineOut = scaledValue <= -30 ? 0.0 : Utils::DB2Gainf(scaledValue);
				break;


			case Parameter::TapEnabled:
			{
				auto newVal = scaledValue >= 0.5;
				if (newVal != multitapEnabled)
					multitap.ClearBuffers();
				multitapEnabled = newVal;
				break;
			}
			case Parameter::TapCount:
				multitap.SetTapCount((int)scaledValue);
				break;
			case Parameter::TapDecay:
				multitap.SetTapDecay(scaledValue);
				break;
			case Parameter::TapPredelay:
				preDelay.SampleDelay = (int)Ms2Samples(scaledValue);
				break;
			case Parameter::TapLength:
				multitap.SetTapLength((int)Ms2Samples(scaledValue));
				break;


			case Parameter::EarlyDiffuseEnabled:
			{
				auto newVal = scaledValue >= 0.5;
				if (newVal != diffuserEnabled)
					diffuser.ClearBuffers();
				diffuserEnabled = newVal;
				break;
			}
			case Parameter::EarlyDiffuseCount:
				diffuser.Stages = (int)scaledValue;
				break;
			case Parameter::EarlyDiffuseDelay:
				diffuser.SetDelay((int)Ms2Samples(scaledValue));
				break;
			case Parameter::EarlyDiffuseModAmount:
				diffuser.SetModulationEnabled(scaledValue > 0.5);
				diffuser.SetModAmount(Ms2Samples(scaledValue));
				break;
			case Parameter::EarlyDiffuseFeedback:
				diffuser.SetFeedback(scaledValue);
				break;
			case Parameter::EarlyDiffuseModRate:
				diffuser.SetModRate(scaledValue);
				break;


			case Parameter::LateMode:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].TapPostDiffuser = scaledValue >= 0.5;
				break;
			case Parameter::LateLineCount:
				lineCount = (int)scaledValue;
				break;
			case Parameter::LateDiffuseEnabled:
				for (int i = 0; i < TotalLineCount; i++)
				{
					auto newVal = scaledValue >= 0.5;
					if (newVal != lines[i].DiffuserEnabled)
						lines[i].ClearDiffuserBuffer();
					lines[i].DiffuserEnabled = newVal;
				}
				break;
			case Parameter::LateDiffuseCount:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetDiffuserStages((int)scaledValue);
				break;
			case Parameter::LateLineSize:
				UpdateLines();
				break;
			case Parameter::LateLineModAmount:
				UpdateLines();
				break;
			case Parameter::LateDiffuseDelay:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetDiffuserDelay((int)Ms2Samples(scaledValue));
				break;
			case Parameter::LateDiffuseModAmount:
				UpdateLines();
				break;
			case Parameter::LateLineDecay:
				UpdateLines();
				break;
			case Parameter::LateLineModRate:
				UpdateLines();
				break;
			case Parameter::LateDiffuseFeedback:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetDiffuserFeedback(scaledValue);
				break;
			case Parameter::LateDiffuseModRate:
				UpdateLines();
				break;


			case Parameter::EqLowShelfEnabled:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].LowShelfEnabled = scaledValue >= 0.5;
				break;
			case Parameter::EqHighShelfEnabled:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].HighShelfEnabled = scaledValue >= 0.5;
				break;
			case Parameter::EqLowpassEnabled:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].CutoffEnabled = scaledValue >= 0.5;
				break;
			case Parameter::EqLowFreq:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetLowShelfFrequency(scaledValue);
				break;
			case Parameter::EqHighFreq:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetHighShelfFrequency(scaledValue);
				break;
			case Parameter::EqCutoff:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetCutoffFrequency(scaledValue);
				break;
			case Parameter::EqLowGain:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetLowShelfGain(scaledValue);
				break;
			case Parameter::EqHighGain:
				for (int i = 0; i < TotalLineCount; i++)
					lines[i].SetHighShelfGain(scaledValue);
				break;


			case Parameter::EqCrossSeed:
				crossSeed = channelLr == ChannelLR::Right ? 0.5 * scaledValue : 1 - 0.5 * scaledValue;
				multitap.SetCrossSeed(crossSeed);
				diffuser.SetCrossSeed(crossSeed);
				UpdateLines();
				UpdatePostDiffusion();
				break;


			case Parameter::SeedTap:
				multitap.SetSeed((int)scaledValue);
				break;
			case Parameter::SeedDiffusion:
				diffuser.SetSeed((int)scaledValue);
				break;
			case Parameter::SeedDelay:
				delayLineSeed = (int)scaledValue;
				UpdateLines();
				break;
			case Parameter::SeedPostDiffusion:
				postDiffusionSeed = (int)scaledValue;
				UpdatePostDiffusion();
				break;

			// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-datamodel-01>
			// Phase 2: Crossfeed parameter handling (infrastructure only)
			case Parameter::CrossfeedEnabled:
				// Handled by ReverbController's SetCrossfeedMode
				break;
			case Parameter::EarlyCrossfeedAmount:
			case Parameter::LateCrossfeedAmount:
			case Parameter::CrossfeedDamping:
				// Phase 3 will use these for actual DSP mixing
				// Phase 2: Just store in paramsScaled array
				break;
			}
		}

		// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-datamodel-01>
		// Phase 2: Crossfeed buffer coordination methods (infrastructure only)
		// These methods allow ReverbController to set up buffer sharing between L/R channels

		// Set crossfeed input buffers (from opposite channel)
		// Phase 3 will read from these buffers during processing for crossfeed mixing
		void SetCrossfeedBuffers(const float* earlyInput, const float* lateInput, int bufferSize)
		{
			crossfeedEarlyInputBuffer_ = earlyInput;
			crossfeedLateInputBuffer_ = lateInput;
			crossfeedBufferSize_ = bufferSize;
		}

		// Enable/disable early and late crossfeed independently
		void SetEarlyCrossfeedEnabled(bool enabled)
		{
			earlyCrossfeedEnabled_ = enabled;
		}

		void SetLateCrossfeedEnabled(bool enabled)
		{
			lateCrossfeedEnabled_ = enabled;
		}

		// Get enable states for validation
		bool GetEarlyCrossfeedEnabled() const { return earlyCrossfeedEnabled_; }
		bool GetLateCrossfeedEnabled() const { return lateCrossfeedEnabled_; }

		// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-datamodel-01>
		// Phase 3: Buffer extraction methods for crossfeed coordination
		// Allow ReverbController to access early/late outputs for buffer exchange
		const float* GetEarlyOutputBuffer() const { return earlyOutputBuffer_; }
		const float* GetLateOutputBuffer() const { return lateOutputBuffer_; }

		void Process(float* input, float* output, int bufSize)
		{
			float tempBuffer[BUFFER_SIZE];
			float earlyOutBuffer[BUFFER_SIZE];
			float lineOutBuffer[BUFFER_SIZE];
			float lineSumBuffer[BUFFER_SIZE];

			Utils::Copy(tempBuffer, input, bufSize);

			if (lowCutEnabled)
				highPass.Process(tempBuffer, tempBuffer, bufSize);
			if (highCutEnabled)
				lowPass.Process(tempBuffer, tempBuffer, bufSize);

			// completely zero if no input present
			// Previously, the very small values were causing some really strange CPU spikes
			for (int i = 0; i < bufSize; i++)
			{
				auto n = tempBuffer[i];
				if (n * n < 0.000000001)
					tempBuffer[i] = 0;
			}

			preDelay.Process(tempBuffer, tempBuffer, bufSize);
			if (multitapEnabled)
				multitap.Process(tempBuffer, tempBuffer, bufSize);
			if (diffuserEnabled)
				diffuser.Process(tempBuffer, tempBuffer, bufSize);

			Utils::Copy(earlyOutBuffer, tempBuffer, bufSize);

			// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-algorithm-02>
			// Phase 3: Early reflection crossfeed - mix early reflections from opposite channel
			// This creates discrete L→R and R→L reflections simulating room wall reflections
			// Apply IMMEDIATELY (same block), not delayed like late crossfeed
			if (earlyCrossfeedEnabled_ && crossfeedEarlyInputBuffer_ != nullptr)
			{
				float earlyAmount = paramsScaled[Parameter::EarlyCrossfeedAmount];

				// Mix opposite channel's early reflections into this channel's early output
				// This simulates discrete wall reflections crossing the stereo field
				for (int i = 0; i < bufSize; i++)
				{
					earlyOutBuffer[i] += crossfeedEarlyInputBuffer_[i] * earlyAmount;
				}
			}

			// Store early output for opposite channel's use (via ReverbController coordination)
			Utils::Copy(earlyOutputBuffer_, earlyOutBuffer, bufSize);

			// ADC-IMPLEMENTS: <reverbv1-topology-algo-01>
			// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-algo-03>
			// Phase 3: Late diffusion feedback crossfeed with damping for stability
			// CRITICAL: Inject feedback BEFORE late diffusion processing (not at output)
			// This creates a true feedback loop where late diffusion processes the crossfeed energy
			// Damping coefficient prevents runaway feedback (ensures loop gain < 1.0)
			if (lateCrossfeedEnabled_ && crossfeedLateInputBuffer_ != nullptr)
			{
				float lateAmount = paramsScaled[Parameter::LateCrossfeedAmount];
				float damping = paramsScaled[Parameter::CrossfeedDamping];

				// Apply damping to prevent feedback instability
				// Loop gain = lateAmount × damping must be < 1.0
				// Example: 0.3 × 0.85 = 0.255 (stable)
				float dampedGain = lateAmount * damping;

				// Inject damped feedback from opposite channel's PREVIOUS block into late diffusion input
				// This is NOT output mixing - it's feedback injection into the processing chain
				for (int i = 0; i < bufSize; i++)
				{
					tempBuffer[i] += crossfeedLateInputBuffer_[i] * dampedGain;
				}
			}

			Utils::ZeroBuffer(lineSumBuffer, bufSize);
			for (int i = 0; i < lineCount; i++)
			{
				lines[i].Process(tempBuffer, lineOutBuffer, bufSize);
				Utils::Mix(lineSumBuffer, lineOutBuffer, 1.0f, bufSize);
			}

			auto perLineGain = GetPerLineGain();
			Utils::Gain(lineSumBuffer, perLineGain, bufSize);

			// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-algo-03>
			// Phase 3: Store late diffusion output for feedback to opposite channel's NEXT block
			// This creates the 1-block delay inherent in the feedback loop (causality requirement)
			Utils::Copy(lateOutputBuffer_, lineSumBuffer, bufSize);

			for (int i = 0; i < bufSize; i++)
			{
				output[i] = dryOut * input[i]
					+ earlyOut * earlyOutBuffer[i]
					+ lineOut * lineSumBuffer[i];
			}
		}

		void ClearBuffers()
		{
			lowPass.ClearBuffers();
			highPass.ClearBuffers();
			preDelay.ClearBuffers();
			multitap.ClearBuffers();
			diffuser.ClearBuffers();
			for (int i = 0; i < TotalLineCount; i++)
				lines[i].ClearBuffers();

			// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-datamodel-01>
			// Phase 2: Clear crossfeed buffer references
			crossfeedEarlyInputBuffer_ = nullptr;
			crossfeedLateInputBuffer_ = nullptr;
			crossfeedBufferSize_ = 0;

			// Phase 3: Clear output buffers to prevent stale data in feedback loop
			Utils::ZeroBuffer(earlyOutputBuffer_, BUFFER_SIZE);
			Utils::ZeroBuffer(lateOutputBuffer_, BUFFER_SIZE);
		}


	private:
		float GetPerLineGain()
		{
			return 1.0 / std::sqrt(lineCount);
		}

		void UpdateLines()
		{
			auto lineDelaySamples = (int)Ms2Samples(paramsScaled[Parameter::LateLineSize]);
			auto lineDecayMillis = paramsScaled[Parameter::LateLineDecay] * 1000;
			auto lineDecaySamples = Ms2Samples(lineDecayMillis);

			auto lineModAmount = Ms2Samples(paramsScaled[Parameter::LateLineModAmount]);
			auto lineModRate = paramsScaled[Parameter::LateLineModRate];

			auto lateDiffusionModAmount = Ms2Samples(paramsScaled[Parameter::LateDiffuseModAmount]);
			auto lateDiffusionModRate = paramsScaled[Parameter::LateDiffuseModRate];

			auto delayLineSeeds = RandomBuffer::Generate(delayLineSeed, TotalLineCount * 3, crossSeed);

			for (int i = 0; i < TotalLineCount; i++)
			{
				auto modAmount = lineModAmount * (0.7 + 0.3 * delayLineSeeds[i]);
				auto modRate = lineModRate * (0.7 + 0.3 * delayLineSeeds[TotalLineCount + i]) / samplerate;

				auto delaySamples = (0.5 + 1.0 * delayLineSeeds[TotalLineCount * 2 + i]) * lineDelaySamples;
				if (delaySamples < modAmount + 2) // when the delay is set really short, and the modulation is very high
					delaySamples = modAmount + 2; // the mod could actually take the delay time negative, prevent that! -- provide 2 extra sample as margin of safety

				auto dbAfter1Iteration = delaySamples / lineDecaySamples * (-60); // lineDecay is the time it takes to reach T60
				auto gainAfter1Iteration = Utils::DB2Gainf(dbAfter1Iteration);

				lines[i].SetDelay((int)delaySamples);
				lines[i].SetFeedback(gainAfter1Iteration);
				lines[i].SetLineModAmount(modAmount);
				lines[i].SetLineModRate(modRate);
				lines[i].SetDiffuserModAmount(lateDiffusionModAmount);
				lines[i].SetDiffuserModRate(lateDiffusionModRate);
			}
		}

		void UpdatePostDiffusion()
		{
			for (int i = 0; i < TotalLineCount; i++)
				lines[i].SetDiffuserSeed((postDiffusionSeed) * (i + 1), crossSeed);
		}

		float Ms2Samples(float value)
		{
			return value / 1000.0f * samplerate;
		}

	};
}
