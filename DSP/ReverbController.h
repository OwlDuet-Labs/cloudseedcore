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

#include <vector>
#include "../Parameters.h"
#include "ReverbChannel.h"
#include "AllpassDiffuser.h"
#include "MultitapDelay.h"
#include "Utils.h"

namespace Cloudseed
{
	// ADC-IMPLEMENTS: <reverbv1-crossfeed-statemgmt-datamodel-02>
	// Crossfeed mode enumeration for internal state management
	enum class CrossfeedMode
	{
		Disabled = 0,    // Baseline CloudSeed behavior (default)
		EarlyOnly = 1,   // Early reflections crossfeed only
		LateOnly = 2,    // Late diffusion crossfeed only
		Both = 3         // Both early and late crossfeed (full mode)
	};

	class ReverbController
	{
	private:
		int samplerate;

		ReverbChannel channelL;
		ReverbChannel channelR;
		double parameters[(int)Parameter::COUNT] = {0};

		// ADC-IMPLEMENTS: <reverbv1-crossfeed-statemgmt-datamodel-02>
		// Phase 2: Crossfeed mode state (infrastructure only, no DSP implementation yet)
		CrossfeedMode crossfeedMode_;

		// Crossfeed buffer coordination (Phase 2: pointers only, buffer sharing in Phase 3)
		// These pointers allow ReverbController to coordinate buffer exchange between L/R channels
		float* leftEarlyOutputBuffer_;
		float* rightEarlyOutputBuffer_;
		float* leftLateOutputBuffer_;
		float* rightLateOutputBuffer_;
		int crossfeedBufferSize_;

	public:
		ReverbController(int samplerate) :
			channelL(samplerate, ChannelLR::Left),
			channelR(samplerate, ChannelLR::Right),
			crossfeedMode_(CrossfeedMode::Disabled),  // Default: crossfeed disabled (baseline behavior)
			leftEarlyOutputBuffer_(nullptr),
			rightEarlyOutputBuffer_(nullptr),
			leftLateOutputBuffer_(nullptr),
			rightLateOutputBuffer_(nullptr),
			crossfeedBufferSize_(0)
		{
			this->samplerate = samplerate;
		}

		int GetSamplerate()
		{
			return samplerate;
		}

		void SetSamplerate(int samplerate)
		{
			this->samplerate = samplerate;
			channelL.SetSamplerate(samplerate);
			channelR.SetSamplerate(samplerate);
		}

		int GetParameterCount()
		{
			return Parameter::COUNT;
		}

		double* GetAllParameters()
		{
			return parameters;
		}

		void SetParameter(int paramId, double value)
		{
			parameters[paramId] = value;
			auto scaled = ScaleParam(value, paramId);
			channelL.SetParameter(paramId, scaled);
			channelR.SetParameter(paramId, scaled);

			// ADC-IMPLEMENTS: <reverbv1-crossfeed-statemgmt-datamodel-02>
			// Update crossfeed mode when CrossfeedEnabled parameter changes
			if (paramId == Parameter::CrossfeedEnabled)
			{
				// Phase 2: Simple mode flag update (no DSP implementation yet)
				// Mode determined by CrossfeedEnabled parameter (off vs on)
				// Future phases will implement EarlyOnly/LateOnly/Both modes
				bool enabled = (scaled >= 0.5);
				SetCrossfeedMode(enabled ? CrossfeedMode::Both : CrossfeedMode::Disabled);
			}
		}

		// ADC-IMPLEMENTS: <reverbv1-crossfeed-statemgmt-impl-01>
		// Phase 2: Safe mode switching infrastructure
		// Clears buffers to prevent stale reverb tails when switching modes
		void SetCrossfeedMode(CrossfeedMode newMode)
		{
			if (crossfeedMode_ == newMode)
				return;  // No change needed

			// Clear internal state to prevent artifacts (addresses Bug #1 from previous attempts)
			// This ensures no stale reverb tails bleed through during mode transitions
			ClearBuffers();

			// Update mode flag
			crossfeedMode_ = newMode;

			// Note: Crossfeed amount ramping handled by CloudSeed's existing parameter
			// interpolation system (no additional smoothing needed)
		}

		// ADC-IMPLEMENTS: <reverbv1-crossfeed-statemgmt-datamodel-02>
		// Query current crossfeed mode
		CrossfeedMode GetCrossfeedMode() const
		{
			return crossfeedMode_;
		}

		// ADC-IMPLEMENTS: <reverbv1-crossfeed-statemgmt-feature-01>
		// Phase 2: Enhanced buffer clearing for mode transitions
		void ClearBuffers()
		{
			channelL.ClearBuffers();
			channelR.ClearBuffers();

			// Clear crossfeed buffer references (Phase 2: pointers only, no data allocated yet)
			// Phase 3 will manage actual buffer data
			leftEarlyOutputBuffer_ = nullptr;
			rightEarlyOutputBuffer_ = nullptr;
			leftLateOutputBuffer_ = nullptr;
			rightLateOutputBuffer_ = nullptr;
		}

		void Process(float* inL, float* inR, float* outL, float* outR, int bufSize)
		{
			float outLTemp[BUFFER_SIZE];
			float outRTemp[BUFFER_SIZE];

			while (bufSize > 0)
			{
				int subBufSize = bufSize > BUFFER_SIZE ? BUFFER_SIZE : bufSize;
				ProcessChunk(inL, inR, outLTemp, outRTemp, subBufSize);
				Utils::Copy(outL, outLTemp, subBufSize);
				Utils::Copy(outR, outRTemp, subBufSize);
				inL = &inL[subBufSize];
				inR = &inR[subBufSize];
				outL = &outL[subBufSize];
				outR = &outR[subBufSize];
				bufSize -= subBufSize;
			}
		}

	private:
		void ProcessChunk(float* inL, float* inR, float* outL, float* outR, int bufSize)
		{
			float leftChannelIn[BUFFER_SIZE];
			float rightChannelIn[BUFFER_SIZE];

			float inputMix = ScaleParam(parameters[Parameter::InputMix], Parameter::InputMix);
			float cm = inputMix * 0.5;
			float cmi = (1 - cm);

			for (int i = 0; i < bufSize; i++)
			{
				leftChannelIn[i] = inL[i] * cmi + inR[i] * cm;
				rightChannelIn[i] = inR[i] * cmi + inL[i] * cm;
			}

			// ADC-IMPLEMENTS: <reverbv1-crossfeed-statemgmt-impl-02>
			// ADC-IMPLEMENTS: <reverbv1-crossfeed-topology-algo-03>
			// Phase 3: Crossfeed buffer coordination based on mode

			if (crossfeedMode_ == CrossfeedMode::Disabled)
			{
				// Baseline mode: Independent L/R processing with no crossfeed
				channelL.Process(leftChannelIn, outL, bufSize);
				channelR.Process(rightChannelIn, outR, bufSize);
			}
			else
			{
				// Crossfeed mode: Set up buffer sharing before processing
				// Configure which crossfeed stages are enabled based on mode
				bool earlyEnabled = (crossfeedMode_ == CrossfeedMode::EarlyOnly ||
				                     crossfeedMode_ == CrossfeedMode::Both);
				bool lateEnabled = (crossfeedMode_ == CrossfeedMode::LateOnly ||
				                    crossfeedMode_ == CrossfeedMode::Both);

				channelL.SetEarlyCrossfeedEnabled(earlyEnabled);
				channelL.SetLateCrossfeedEnabled(lateEnabled);
				channelR.SetEarlyCrossfeedEnabled(earlyEnabled);
				channelR.SetLateCrossfeedEnabled(lateEnabled);

				// CRITICAL: Set up crossfeed input buffers BEFORE processing
				// Each channel reads from the opposite channel's PREVIOUS block outputs
				// This creates the 1-block delay inherent in the feedback loop
				channelL.SetCrossfeedBuffers(
					channelR.GetEarlyOutputBuffer(),  // L reads R's previous early output
					channelR.GetLateOutputBuffer(),   // L reads R's previous late output (feedback)
					bufSize
				);
				channelR.SetCrossfeedBuffers(
					channelL.GetEarlyOutputBuffer(),  // R reads L's previous early output
					channelL.GetLateOutputBuffer(),   // R reads L's previous late output (feedback)
					bufSize
				);

				// Process channels with crossfeed enabled
				// Each channel will:
				// 1. Mix early reflections from opposite channel (early crossfeed)
				// 2. Inject damped late feedback from opposite channel before late diffusion
				// 3. Store early/late outputs for opposite channel's NEXT block
				channelL.Process(leftChannelIn, outL, bufSize);
				channelR.Process(rightChannelIn, outR, bufSize);

				// Note: Buffer outputs are automatically stored in ReverbChannel::Process()
				// for use in the next processing block (feedback loop continuity)
			}
		}
	};
}
