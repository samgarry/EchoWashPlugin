#pragma once

#ifndef __echoplexTapeDelay__
#define __echoplexTapeDelay__

#include "fxobjects.h"
#include "sytemNoiseGenerator.h"


/**
\struct echoplexTapeDelayParameters
\ingroup FX-Objects
\brief
Custom parameter structure for the echoplexTapeDelay object.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
struct echoplexTapeDelayParameters : public AudioDelayParameters
{
	echoplexTapeDelayParameters() {}

	/** all FXObjects parameter objects require overloaded= operator so remember to add new entries if you add new variables. */
	echoplexTapeDelayParameters& operator=(const echoplexTapeDelayParameters& params)	// need this override for collections to work
	{
		// --- it is possible to try to make the object equal to itself
		//     e.g. thisObject = thisObject; so this code catches that
		//     trivial case and just returns this object
		if (this == &params)
			return *this;

		// --- copy from params (argument) INTO our variables
		bandLimitHPFFc_Hz = params.bandLimitHPFFc_Hz;
		bandLimitLPFFc_Hz = params.bandLimitLPFFc_Hz;
		limiterThreshold_dB = params.limiterThreshold_dB;
		limiterMakeUpGain_dB = params.limiterMakeUpGain_dB;
		recordLevel_dB = params.recordLevel_dB;
		playbackLevel_dB = params.playbackLevel_dB;
		waveshaperSaturation = params.waveshaperSaturation;
		sixtyHzNoiseAmplitude = params.sixtyHzNoiseAmplitude;
		tapeNoiseFc_Hz = params.tapeNoiseFc_Hz;
		tapeNoiseAmplitude = params.tapeNoiseAmplitude;
		outputAmplitude_dB = params.outputAmplitude_dB;

		// --- call base "class" function to implement it
		AudioDelayParameters::operator=(params);

		// --- MUST be last
		return *this;
	}

	// --- individual parameters
	//other objects
	double bandLimitHPFFc_Hz = 20.0; 
	double bandLimitLPFFc_Hz = 8000.0;
	double limiterThreshold_dB = -3.0; 
	double limiterMakeUpGain_dB = 0.0; 
	double recordLevel_dB = 0.0; 
	double playbackLevel_dB = 0.0; 

	//noise generator
	double waveshaperSaturation = 1.0;
	double sixtyHzNoiseAmplitude = 1.0;
	double tapeNoiseFc_Hz = 10000.0;
	double tapeNoiseAmplitude = 0.0;
	double outputAmplitude_dB = -40.0;
	//systemNoiseGeneratorParameters sysNoiseGenParameters;
};


/**
\class echoplexTapeDelay
\ingroup FX-Objects
\brief
The echoplexTapeDelay object implements ....

Audio I/O:
- Processes mono input to mono output.
- *** Optionally, process frame *** Modify this according to your object functionality

Control I/F:
- Use echoplexTapeDelayParameters structure to get/set object params.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
class echoplexTapeDelay : public AudioDelay
{
public:
	echoplexTapeDelay(void) {}	/* C-TOR */
	~echoplexTapeDelay(void) {}	/* D-TOR */

public:
	/** reset members to initialized state */
	virtual bool reset(double _sampleRate)
	{
		// --- store the sample rate
		sampleRate = (_sampleRate);

		// --- do any other per-audio-run inits here

		// --- Audio Filters
		for (int i = 0; i < 2; i++)
		{
			AudioFilterParameters lpfParams = bandlimitingLPF[i].getParameters();
			AudioFilterParameters hpfParams = bandlimitingHPF[i].getParameters();

			lpfParams.algorithm = filterAlgorithm::kLPF1;
			hpfParams.algorithm = filterAlgorithm::kHPF1;

			bandlimitingLPF[i].setParameters(lpfParams);
			bandlimitingHPF[i].setParameters(hpfParams);
			bandlimitingHPF[i].reset(_sampleRate);
			bandlimitingLPF[i].reset(_sampleRate);
		}

		// --- System Noise Generator
		systemNoiseGenerator.reset(_sampleRate);

		// --- Peak Limiters
		for (int i = 0; i < 2; i++)
		{
			saturatingNLP[i].setThreshold_dB(echoplexParameters.limiterThreshold_dB);
			saturatingNLP[i].setMakeUpGain_dB(echoplexParameters.limiterMakeUpGain_dB);
			saturatingNLP[i].reset(_sampleRate);
		}
		 
		// --- Audio Delay
		AudioDelay::reset(_sampleRate);
		AudioDelay::createDelayBuffers(_sampleRate, 5000.0);


		return true;
	}

	/** process MONO input */
	/**
	\param xn input
	\return the processed sample
	*/
	virtual double processAudioSample(double xn)
	{
		// --- the output variable
		double yn = 0.0;
		double rawRecord = pow(10, echoplexParameters.recordLevel_dB/20);
		double rawPlayback = pow(10, echoplexParameters.playbackLevel_dB/20);
		double dry = pow(10, echoplexParameters.dryLevel_dB/20);

		// --- do your DSP magic here to create yn

		// --- read delay
		SignalGenData noiseSignal = systemNoiseGenerator.renderAudioOutput();
		double noiseOutput = noiseSignal.normalOutput;
		yn = delayBuffer_L.readBuffer(delayInSamples_L) + noiseOutput;

		double peakOutput = saturatingNLP[0].processAudioSample(yn);
		double ynProcessed = peakOutput * echoplexParameters.feedback_Pct;



		// --- create input for delay buffer
		//double dn = xn + (parameters.feedback_Pct / 100.0) * yn;
		double xnSignal = (xn * rawRecord) + ynProcessed;
		double dnOne = bandlimitingHPF[0].processAudioSample(xnSignal);
		double dn = bandlimitingLPF[0].processAudioSample(dnOne);


		// --- write to delay buffer
		delayBuffer_L.writeBuffer(dn);

		// --- form mixture out = dry*xn + wet*yn
		double drySignal = xn * dry;
		double wetSignal = peakOutput * rawPlayback;
		//double output = dryMix * xn + wetMix * yn;
		double output = drySignal + wetSignal;

		return output;

		// --- done
		//return yn;
	}

	/** query to see if this object can process frames */
	virtual bool canProcessAudioFrame() { return true; } // <-- change this!

	/** process audio frame: implement this function if you answer "true" to above query */
	virtual bool processAudioFrame(const float* inputFrame,	/* ptr to one frame of data: pInputFrame[0] = left, pInputFrame[1] = right, etc...*/
					     float* outputFrame,
					     uint32_t inputChannels,
					     uint32_t outputChannels)
	{
		double rawRecord = pow(10.0, echoplexParameters.recordLevel_dB / 20.0);
		double rawPlayback = pow(10.0, echoplexParameters.playbackLevel_dB / 20.0);
		double dry = pow(10.0, echoplexParameters.dryLevel_dB / 20.0);

		// --- make sure we have input and outputs
		if (inputChannels == 0 || outputChannels == 0)
			return false;

		// --- make sure we support this delay algorithm
		if (parameters.algorithm != delayAlgorithm::kNormal/* &&
			parameters.algorithm != delayAlgorithm::kPingPong*/)
			return false;

		// --- if only one output channel, revert to mono operation
		if (outputChannels == 1)
		{
			// --- process left channel only
			outputFrame[0] = processAudioSample(inputFrame[0]);
			return true;
		}

		// --- if we get here we know we have 2 output channels
		//
		// --- pick up inputs
		//
		// --- LEFT channel
		double xnL = inputFrame[0];

		// --- RIGHT channel (duplicate left input if mono-in)
		double xnR = inputChannels > 1 ? inputFrame[1] : xnL;

		// --- do your DSP magic here to create yn
		
		// --- read delay
		SignalGenData noiseSignal = systemNoiseGenerator.renderAudioOutput();
		double noiseOutput = noiseSignal.normalOutput;
		
		double ynL = delayBuffer_L.readBuffer(delayInSamples_L) +noiseOutput;
		double ynR = delayBuffer_R.readBuffer(delayInSamples_R) +noiseOutput;

		
		double peakOutputL = saturatingNLP[0].processAudioSample(ynL);
		double peakOutputR = saturatingNLP[1].processAudioSample(ynR);
		double ynProcessedL = peakOutputL * echoplexParameters.feedback_Pct;
		double ynProcessedR = peakOutputR * echoplexParameters.feedback_Pct;
		

		// --- create input for delay buffer
		//double dn = xn + (parameters.feedback_Pct / 100.0) * yn;
		double xnSignalL = (xnL * rawRecord) + ynProcessedL;
		double xnSignalR = (xnR * rawRecord) + ynProcessedR;
		double dnOneL = bandlimitingHPF[0].processAudioSample(xnSignalL);
		double dnOneR = bandlimitingHPF[1].processAudioSample(xnSignalR);
		double dnL = bandlimitingLPF[0].processAudioSample(dnOneL);
		double dnR = bandlimitingLPF[1].processAudioSample(dnOneR);

		// --- write to delay buffer
		delayBuffer_L.writeBuffer(dnL);
		delayBuffer_R.writeBuffer(dnR);


		// --- form mixture out = dry*xn + wet*yn
		double drySignalL = xnL * dry;
		double wetSignalL = peakOutputL * rawPlayback;
		double outputL = drySignalL + wetSignalL;

		double drySignalR = xnR * dry;
		double wetSignalR = peakOutputR * rawPlayback;
		double outputR = drySignalR + wetSignalR;

		// --- set left channel
		outputFrame[0] = outputL;

		// --- set right channel
		outputFrame[1] = outputR;

		return true; 
	}


	/** get parameters: note use of custom structure for passing param data */
	/**
	\return echoplexTapeDelayParameters custom data structure
	*/
	echoplexTapeDelayParameters getParameters()
	{
		return echoplexParameters;
	}

	/** set parameters: note use of custom structure for passing param data */
	/**
	\param echoplexTapeDelayParameters custom data structure
	*/
	void setParameters(const echoplexTapeDelayParameters& _params)
	{
		// --- copy them; note you may choose to ignore certain items
		//     and copy the variables one at a time, or you may test
		//     to see if cook-able variables have changed; if not, then
		//     do not re-cook them as it just wastes CPU
		echoplexParameters = _params;

		// --- cook parameters here

		// --- Peak Limiters
		for (int i = 0; i < 2; i++)
		{
			saturatingNLP[i].setThreshold_dB(echoplexParameters.limiterThreshold_dB);
			saturatingNLP[i].setMakeUpGain_dB(echoplexParameters.limiterMakeUpGain_dB);
		}


		// --- Audio Filters
		for (int i = 0; i < 2; i++)
		{
			AudioFilterParameters lpfParams = bandlimitingLPF[i].getParameters();
			AudioFilterParameters hpfParams = bandlimitingHPF[i].getParameters();
			lpfParams.fc = echoplexParameters.bandLimitLPFFc_Hz;
			hpfParams.fc = echoplexParameters.bandLimitHPFFc_Hz;
			bandlimitingLPF[i].setParameters(lpfParams);
			bandlimitingHPF[i].setParameters(hpfParams);
		}

		// --- Audio Delay
		AudioDelay::setParameters(_params);
		
		// --- Noise Generator
		systemNoiseGeneratorParameters sysParams = systemNoiseGenerator.getParameters();
		sysParams.outputAmplitude_dB = echoplexParameters.outputAmplitude_dB;
		sysParams.sixtyHzNoiseAmplitude = echoplexParameters.sixtyHzNoiseAmplitude;
		sysParams.tapeNoiseAmplitude = echoplexParameters.tapeNoiseAmplitude;
		sysParams.tapeNoiseFc_Hz = echoplexParameters.tapeNoiseFc_Hz;
		sysParams.waveshaperSaturation = echoplexParameters.waveshaperSaturation;
		systemNoiseGenerator.setParameters(sysParams);
	}

protected:  
	echoplexTapeDelayParameters echoplexParameters;   

	// --- in series, these form a bandpass filter   
	//enum {CH_LEFT, CH_RIGHT, NUM_CHANNELS};  
	AudioFilter bandlimitingLPF[2];  
	AudioFilter bandlimitingHPF[2]; 

	// --- limiters  
	PeakLimiter saturatingNLP[2];

	// --- system noise   
	systemNoiseGenerator systemNoiseGenerator;

	// --- local variables used by this object  
	double sampleRate = 0.0; ///< sample rate 

};

#endif