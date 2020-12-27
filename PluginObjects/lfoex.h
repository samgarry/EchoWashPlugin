#pragma once

#ifndef __LFOEx__
#define __LFOEx__

#include "fxobjects.h"

// --- parameters subclassed from base parameters
struct LFOExParameters : public OscillatorParameters
{
	LFOExParameters() {}

	/** all FXObjects parameter objects require overloaded= operator so remember to add new entries if you add new variables. */
	LFOExParameters& operator=(const LFOExParameters& params)	// need this override for collections to work
	{
		// --- it is possible to try to make the object equal to itself
		//     e.g. thisObject = thisObject; so this code catches that
		//     trivial case and just returns this object
		if (this == &params)
			return *this;

		// --- copy from params (argument) INTO our variables
		outputAmplitude = params.outputAmplitude;
		startPhase = params.startPhase;
		randomizedPhase = params.randomizedPhase;

		// --- do base class operation
		OscillatorParameters::operator=(params);

		// --- MUST be last
		return *this;
	}

	// --- individual parameters
	double outputAmplitude = 1.0;
	double startPhase = 0.0;
	bool randomizedPhase = true;
};


/**
\class LFOEx
\ingroup FX-Objects
\brief
The LFOEx object implements ....

Audio I/O:
- Processes mono input to mono output.
- *** Optionally, process frame *** Modify this according to your object functionality

Control I/F:
- Use LFOExParameters structure to get/set object params.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
class LFOEx : public LFO
{
public:
	// --- LFO constructor is called first
	LFOEx(void)  { }	/* C-TOR */
	~LFOEx(void) { }	/* D-TOR */

public:
	/** reset members to initialized state */
	virtual bool reset(double _sampleRate)
	{
		// --- set phase inc based on fs
		sampleRate = _sampleRate;
		phaseInc = lfoexParameters.frequency_Hz / sampleRate;

		// --- timebase variables
		double modCounterOffset = lfoexParameters.startPhase;
		if (lfoexParameters.randomizedPhase)
		{
			double phaseOffset = doWhiteNoise();
			phaseOffset = bipolarToUnipolar(phaseOffset); // [0, 1]
			lfoexParameters.startPhase = phaseOffset;
		}

		modCounter = modCounterOffset;			///< modulo counter gets Start Phase
		modCounterQP = modCounterOffset + 0.25;	///< Quad Phase modulo counter [0.0, +1.0]
		
		if (modCounterQP > 1.0)				///< check and wrap modulo if needed
			modCounterQP -= 1.0;

		return true;
	}

	// --- override the render function
	virtual const SignalGenData renderAudioOutput()
	{
		// --- base class does its thang
		SignalGenData lfoOutput = LFO::renderAudioOutput();

		// --- we scale the output
		lfoOutput.normalOutput *= lfoexParameters.outputAmplitude;
		lfoOutput.invertedOutput *= lfoexParameters.outputAmplitude;
		lfoOutput.quadPhaseOutput_neg *= lfoexParameters.outputAmplitude;
		lfoOutput.quadPhaseOutput_pos *= lfoexParameters.outputAmplitude;

		return lfoOutput;
	}

	/** get parameters: note use of custom structure for passing param data */
	/**
	\return OscillatorParameters custom data structure
	*/
	LFOExParameters getParameters() { return lfoexParameters; }

	/** set parameters: note use of custom structure for passing param data */
	/**
	\param OscillatorParameters custom data structure
	*/
	void setParameters(const LFOExParameters& params)
	{
		// --- save
		lfoexParameters = params;

		// --- forward to base class to set its parameters also
		LFO::setParameters(params);
	}

protected:
	LFOExParameters lfoexParameters;
};

#endif