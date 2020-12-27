#pragma once

#ifndef __systemNoiseGenerator__
#define __systemNoiseGenerator__

#include "fxobjects.h"
#include "lfoex.h"
#include "noisegen.h"

/**
\struct systemNoiseGeneratorParameters
\ingroup FX-Objects
\brief
Custom parameter structure for the systemNoiseGenerator object.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
struct systemNoiseGeneratorParameters
{
	systemNoiseGeneratorParameters() {}

	/** all FXObjects parameter objects require overloaded= operator so remember to add new entries if you add new variables. */
	systemNoiseGeneratorParameters& operator=(const systemNoiseGeneratorParameters& params)	// need this override for collections to work
	{
		// --- it is possible to try to make the object equal to itself
		//     e.g. thisObject = thisObject; so this code catches that
		//     trivial case and just returns this object
		if (this == &params)
			return *this;

		// --- copy from params (argument) INTO our variables
		waveshaperSaturation = params.waveshaperSaturation;
		sixtyHzNoiseAmplitude = params.sixtyHzNoiseAmplitude;
		tapeNoiseFc_Hz = params.tapeNoiseFc_Hz;
		tapeNoiseAmplitude = params.tapeNoiseAmplitude;
		outputAmplitude_dB = params.outputAmplitude_dB;

		// --- MUST be last
		return *this;
	}

	// --- individual parameters
	double waveshaperSaturation = 1.0; 
	double sixtyHzNoiseAmplitude = 1.0; 
	double tapeNoiseFc_Hz = 10000.0; 
	double tapeNoiseAmplitude = 0.0; 

	// --- want to use dB for setting noise floor amplitdude 
	double outputAmplitude_dB = -40.0;  
};


/**
\class systemNoiseGenerator
\ingroup FX-Objects
\brief
The systemNoiseGenerator object implements ....

Audio I/O:
- Processes mono input to mono output.
- *** Optionally, process frame *** Modify this according to your object functionality

Control I/F:
- Use systemNoiseGeneratorParameters structure to get/set object params.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
class systemNoiseGenerator : public IAudioSignalGenerator
{
public:
	systemNoiseGenerator(void) {}	/* C-TOR */
	~systemNoiseGenerator(void) {}	/* D-TOR */

public:
	/** reset members to initialized state */
	virtual bool reset(double _sampleRate)
	{
		// --- 60 Hz LFO
		LFOExParameters lfoParams = lfo.getParameters();
		lfoParams.frequency_Hz = 60.0;
		lfoParams.waveform = generatorWaveform::kSin;
		lfo.setParameters(lfoParams);
		lfo.reset(_sampleRate);

		// --- Tape Hiss Noise Generator
		NoiseGeneratorParameters noiseParams = noiseGenerator.getParameters();
		noiseParams.lpf_fc_Hz = SystemNoiseGeneratorParameters.tapeNoiseFc_Hz;
		noiseGenerator.setParameters(noiseParams);
		noiseGenerator.reset(_sampleRate);

		// --- store the sample rate
		sampleRate = (_sampleRate);

		// --- do any other per-audio-run inits here

		return true;
	}

	/** render a new audio output structure */
	virtual const SignalGenData renderAudioOutput()
	{
		SignalGenData generatorOutput;

		// --- LFOex Object
		SignalGenData lfoOutput;
		lfoOutput = lfo.renderAudioOutput();
		double lfoOut = lfoOutput.normalOutput;
		double waveShaped = atanWaveShaper(lfoOut, SystemNoiseGeneratorParameters.waveshaperSaturation);
		double lfoGain = SystemNoiseGeneratorParameters.sixtyHzNoiseAmplitude;

		
		// --- NoiseGen Object
		NoiseGenData noiseOutput;
		noiseOutput = noiseGenerator.renderAudioOutput();
		double noiseOut = noiseOutput.filteredWhiteNoiseOut;
		double noiseGain = SystemNoiseGeneratorParameters.tapeNoiseAmplitude;
		

		// --- Sum the two objects
		double sumOutput = noiseOut * noiseGain + waveShaped * lfoGain;

		// --- Assign to the output
		generatorOutput.normalOutput = (pow(10.0, SystemNoiseGeneratorParameters.outputAmplitude_dB / 20) * sumOutput);


		return generatorOutput;
	}

	/** get parameters: note use of custom structure for passing param data */
	/**
	\return systemNoiseGeneratorParameters custom data structure
	*/
	systemNoiseGeneratorParameters getParameters()
	{
		return SystemNoiseGeneratorParameters;
	}

	/** set parameters: note use of custom structure for passing param data */
	/**
	\param systemNoiseGeneratorParameters custom data structure
	*/
	void setParameters(const systemNoiseGeneratorParameters& _params)
	{
		// --- copy them; note you may choose to ignore certain items
		//     and copy the variables one at a time, or you may test
		//     to see if cook-able variables have changed; if not, then
		//     do not re-cook them as it just wastes CPU
		SystemNoiseGeneratorParameters = _params;

		LFOExParameters lfoParams = lfo.getParameters();
		lfoParams.outputAmplitude = SystemNoiseGeneratorParameters.sixtyHzNoiseAmplitude;
		lfo.setParameters(lfoParams);

		NoiseGeneratorParameters noiseParams = noiseGenerator.getParameters();
		noiseParams.outputAmplitude = SystemNoiseGeneratorParameters.tapeNoiseAmplitude;
		noiseParams.lpf_fc_Hz = SystemNoiseGeneratorParameters.tapeNoiseFc_Hz;
		noiseGenerator.setParameters(noiseParams);

		// --- cook parameters here
	}

private:
	systemNoiseGeneratorParameters SystemNoiseGeneratorParameters; ///< object parameters

	// --- local variables used by this object
	LFOEx lfo;
	NoiseGenerator noiseGenerator;

	double sampleRate = 0.0;	///< sample rate

};

#endif