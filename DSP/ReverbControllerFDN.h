/*
FDN-Integrated Reverb Controller
Replaces late parallel delay lines with 8x8 FDN while preserving early processing.

ADC-IMPLEMENTS: <reverb-v1-fdn-topology-impl-02>
*/

#pragma once

#include <vector>
#include "../Parameters.h"
#include "ReverbChannel.h"
#include "FDN/FDNCore.h"
#include "Utils.h"

namespace Cloudseed
{
	// ADC-IMPLEMENTS: <reverb-v1-fdn-topology-impl-02>
	/**
	 * ReverbControllerFDN: CloudSeed with FDN late reverb
	 *
	 * Architecture:
	 * - Early processing (per channel): Input filters, pre-delay, multitap, early diffusion
	 * - Late processing (stereo coupled): 8x8 FDN with Hadamard matrix
	 * - Output mixing: Dry + Early + Late signals
	 *
	 * Changes from original CloudSeed:
	 * - Removed: 12 parallel delay lines per channel
	 * - Removed: Late allpass diffusion (in feedback loop)
	 * - Added: FDN with divergent L/R injection and matrix coupling
	 * - Preserved: All early processing completely unchanged
	 */
	class ReverbControllerFDN
	{
	private:
		int samplerate;

		ReverbChannel channelL;
		ReverbChannel channelR;
		FDN::FDNCore fdnCore;

		double parameters[(int)Parameter::COUNT] = {0};

	public:
		ReverbControllerFDN(int samplerate) :
			channelL(samplerate, ChannelLR::Left),
			channelR(samplerate, ChannelLR::Right)
		{
			this->samplerate = samplerate;
			fdnCore.prepare(samplerate, BUFFER_SIZE);
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
			fdnCore.prepare(samplerate, BUFFER_SIZE);
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

			// Route parameters to appropriate destinations
			switch (paramId)
			{
				// ========================================================
				// FDN-SPECIFIC PARAMETERS
				// ========================================================
				case Parameter::LateLineDecay:
					// RT60 decay time → FDN decay gain
					fdnCore.setDecayTime(scaled);
					break;

				case Parameter::LateLineModAmount:
					// Modulation depth → FDN modulation (normalize to 0-1)
					fdnCore.setModulationAmount(scaled / 2.5f);
					break;

				case Parameter::LateLineModRate:
					// Modulation frequency → FDN modulation rate
					fdnCore.setModulationRate(scaled);
					break;

				case Parameter::EqLowShelfEnabled:
				case Parameter::EqHighShelfEnabled:
				case Parameter::EqLowpassEnabled:
					// EQ enable flags → FDN EQ
					fdnCore.setEQEnabled(
						parameters[Parameter::EqLowShelfEnabled] >= 0.5,
						parameters[Parameter::EqHighShelfEnabled] >= 0.5,
						parameters[Parameter::EqLowpassEnabled] >= 0.5
					);
					break;

				case Parameter::EqLowFreq:
				case Parameter::EqHighFreq:
				case Parameter::EqCutoff:
					// EQ frequencies → FDN EQ
					fdnCore.setEQFrequencies(
						ScaleParam(parameters[Parameter::EqLowFreq], Parameter::EqLowFreq),
						ScaleParam(parameters[Parameter::EqHighFreq], Parameter::EqHighFreq),
						ScaleParam(parameters[Parameter::EqCutoff], Parameter::EqCutoff)
					);
					break;

				case Parameter::EqLowGain:
				case Parameter::EqHighGain:
					// EQ gains → FDN EQ
					fdnCore.setEQGains(
						ScaleParam(parameters[Parameter::EqLowGain], Parameter::EqLowGain),
						ScaleParam(parameters[Parameter::EqHighGain], Parameter::EqHighGain)
					);
					break;

				// ========================================================
				// DEPRECATED PARAMETERS (ignored in FDN mode)
				// ========================================================
				case Parameter::LateMode:
				case Parameter::LateLineCount:
				case Parameter::LateDiffuseEnabled:
				case Parameter::LateDiffuseCount:
				case Parameter::LateDiffuseDelay:
				case Parameter::LateDiffuseModAmount:
				case Parameter::LateDiffuseFeedback:
				case Parameter::LateDiffuseModRate:
				case Parameter::SeedDelay:
				case Parameter::SeedPostDiffusion:
					// These parameters are not used in FDN architecture
					// FDN has fixed 8 lines, no late diffusion, fixed delays
					break;

				// ========================================================
				// EARLY PROCESSING PARAMETERS (pass through to channels)
				// ========================================================
				default:
					// All other parameters go to early processing
					channelL.SetParameter(paramId, scaled);
					channelR.SetParameter(paramId, scaled);
					break;
			}
		}

		void ClearBuffers()
		{
			channelL.ClearBuffers();
			channelR.ClearBuffers();
			fdnCore.reset();
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
			// Temporary buffers for processing stages
			float leftChannelIn[BUFFER_SIZE];
			float rightChannelIn[BUFFER_SIZE];
			float earlyOutL[BUFFER_SIZE];
			float earlyOutR[BUFFER_SIZE];
			float lateOutL[BUFFER_SIZE];
			float lateOutR[BUFFER_SIZE];

			// ========================================================
			// STAGE 1: Input mixing (stereo cross-mix)
			// ========================================================
			float inputMix = ScaleParam(parameters[Parameter::InputMix], Parameter::InputMix);
			float cm = inputMix * 0.5f;
			float cmi = (1.0f - cm);

			for (int i = 0; i < bufSize; i++)
			{
				leftChannelIn[i] = inL[i] * cmi + inR[i] * cm;
				rightChannelIn[i] = inR[i] * cmi + inL[i] * cm;
			}

			// ========================================================
			// STAGE 2: Early processing (per channel, preserved from CloudSeed)
			// ========================================================
			// Process through early chain: filters, pre-delay, multitap, early diffusion
			// NOTE: ReverbChannel now only does early processing (late lines disabled)
			channelL.ProcessEarlyOnly(leftChannelIn, earlyOutL, bufSize);
			channelR.ProcessEarlyOnly(rightChannelIn, earlyOutR, bufSize);

			// ========================================================
			// STAGE 3: FDN late reverb (stereo coupled)
			// ========================================================
			// FDN processes early outputs with divergent injection:
			// - L channel → lines 0-3
			// - R channel → lines 4-7
			// - Matrix feedback creates natural stereo coupling
			fdnCore.process(earlyOutL, earlyOutR, lateOutL, lateOutR, bufSize);

			// ========================================================
			// STAGE 4: Output mixing
			// ========================================================
			float dryOut = ScaleParam(parameters[Parameter::DryOut], Parameter::DryOut);
			float earlyOut = ScaleParam(parameters[Parameter::EarlyOut], Parameter::EarlyOut);
			float lateOut = ScaleParam(parameters[Parameter::LateOut], Parameter::LateOut);

			// Convert dB to linear gain
			dryOut = (dryOut <= -30.0f) ? 0.0f : Utils::DB2Gainf(dryOut);
			earlyOut = (earlyOut <= -30.0f) ? 0.0f : Utils::DB2Gainf(earlyOut);
			lateOut = (lateOut <= -30.0f) ? 0.0f : Utils::DB2Gainf(lateOut);

			for (int i = 0; i < bufSize; i++)
			{
				outL[i] = dryOut * inL[i] + earlyOut * earlyOutL[i] + lateOut * lateOutL[i];
				outR[i] = dryOut * inR[i] + earlyOut * earlyOutR[i] + lateOut * lateOutR[i];
			}
		}
	};
}
