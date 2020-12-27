#pragma once

#ifndef __tripleLFOModulator__
#define __tripleLFOModulator__

#include "fxobjects.h"
#include "lfoex.h"

/**
\struct tripleLFOModulatorParameters
\ingroup FX-Objects
\briefr
Custom parameter structure for the tripleLFOModulator object.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
struct tripleLFOModulatorParameters
{
	tripleLFOModulatorParameters() {}

	/** all FXObjects parameter objects require overloaded= operator so remember to add new entries if you add new variables. */
	tripleLFOModulatorParameters& operator=(const tripleLFOModulatorParameters& params)	// need this override for collections to work
	{
		// --- it is possible to try to make the object equal to itself
		//     e.g. thisObject = thisObject; so this code catches that
		//     trivial case and just returns this object
		if (this == &params)
			return *this;

		// --- copy from params (argument) INTO our variables
		for (int i = 0; i <= 2; i++)
		{
			lfoFrequency_Hz[i] = params.lfoFrequency_Hz[i];
			//lfoStartPhase[i] = params.lfoStartPhase[i];
			lfoAmplitude[i] = params.lfoAmplitude[i];
		}
		outputAmplitude = params.outputAmplitude;

		// --- MUST be last
		return *this;
	}
	// --- individual parameters
	double lfoFrequency_Hz[3] = { 2.5, 5.0, 26.0 };  
	//double lfoStartPhase[3] = { 0.0, 0.0, 0.0 }; 
	double lfoAmplitude[3] = { 0.0, 0.0, 0.0 }; 
	double outputAmplitude = 0.71; // not dB 
};


/**
\class tripleLFOModulator
\ingroup FX-Objects
\brief
The tripleLFOModulator object implements ....

Audio I/O:
- Processes mono input to mono output.
- *** Optionally, process frame *** Modify this according to your object functionality

Control I/F:
- Use tripleLFOModulatorParameters structure to get/set object params.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
class tripleLFOModulator : public IAudioSignalGenerator
{
public:
	tripleLFOModulator(void) {}	/* C-TOR */
	~tripleLFOModulator(void) {}	/* D-TOR */

public:
	/** reset members to initialized state */
	virtual bool reset(double _sampleRate)
	{
		// --- store the sample rate
		sampleRate = (_sampleRate);

		// --- do any other per-audio-run inits here
		for (int i = 0; i <= 2; i++)
		{
			LFOExParameters params = lfo[i].getParameters();
			params.randomizedPhase = true;
			params.waveform = generatorWaveform::kSin;
			lfo[i].setParameters(params);
			lfo[i].reset(sampleRate);
		}

		AudioFilterParameters paramsHPF = hpf.getParameters();
		paramsHPF.algorithm = filterAlgorithm::kHPF2;
		paramsHPF.fc = 0.01;
		hpf.setParameters(paramsHPF);
		hpf.reset(sampleRate);

		return true;
	}

	/** render a new audio output structure */
	virtual const SignalGenData renderAudioOutput()
	{
		SignalGenData generatorOutput;

		// --- LFO ONE
		SignalGenData lfoOne;
		lfoOne = lfo[0].renderAudioOutput();
		double outputOne = lfoOne.normalOutput;
		double oneGain = tripleLFOModulatorParams.lfoAmplitude[0];
		outputOne *= (oneGain * 0.33);

		// --- LFO TWO
		SignalGenData lfoTwo;
		lfoTwo = lfo[1].renderAudioOutput();
		double outputTwo = lfoTwo.normalOutput;
		double twoGain = tripleLFOModulatorParams.lfoAmplitude[1];
		outputTwo *= (twoGain * 0.33);

		// --- LFO THREE
		SignalGenData lfoThree;
		lfoThree = lfo[2].renderAudioOutput();
		double outputThree = lfoThree.normalOutput;
		double threeGain = tripleLFOModulatorParams.lfoAmplitude[2];
		outputThree *= (threeGain * 0.33);

		// --- Final Stuff
		double sum = outputOne + outputTwo + outputThree;
		double output = hpf.processAudioSample(sum);
		generatorOutput.normalOutput = output * tripleLFOModulatorParams.outputAmplitude;

		return generatorOutput;
	}

	/** get parameters: note use of custom structure for passing param data */
	/**
	\return tripleLFOModulatorParameters custom data structure
	*/
	tripleLFOModulatorParameters getParameters()
	{
		return tripleLFOModulatorParams;
	}

	/** set parameters: note use of custom structure for passing param data */
	/**
	\param tripleLFOModulatorParameters custom data structure
	*/
	void setParameters(const tripleLFOModulatorParameters& _params)
	{
		// --- copy them; note you may choose to ignore certain items
		//     and copy the variables one at a time, or you may test
		//     to see if cook-able variables have changed; if not, then
		//     do not re-cook them as it just wastes CPU
		tripleLFOModulatorParams = _params;

		for (int i = 0; i <= 2; i++)
		{
			LFOExParameters params = lfo[i].getParameters();
			params.frequency_Hz = tripleLFOModulatorParams.lfoFrequency_Hz[i];
			params.outputAmplitude = tripleLFOModulatorParams.lfoAmplitude[i];
			lfo[i].setParameters(params);
		}

		// --- cook parameters here
	}

private:
	tripleLFOModulatorParameters tripleLFOModulatorParams; ///< object parameters
	LFOEx lfo[3];
	AudioFilter hpf;

	// --- local variables used by this object
	double sampleRate = 0.0;	///< sample rate

};

#endif