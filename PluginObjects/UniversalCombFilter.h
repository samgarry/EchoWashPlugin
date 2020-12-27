#pragma once

#ifndef __UniversalCombFilter__
#define __UniversalCombFilter__

#include "fxobjects.h"

/**
\struct UniversalCombFilterParameters
\ingroup FX-Objects
\brief
Custom parameter structure for the UniversalCombFilter object.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/

enum class CombFilterType { combFilter, inverseCombFilter };

struct UniversalCombFilterParameters
{
	UniversalCombFilterParameters() {}

	/** all FXObjects parameter objects require overloaded= operator so remember to add new entries if you add new variables. */
	UniversalCombFilterParameters& operator=(const UniversalCombFilterParameters& params)	// need this override for collections to work
	{
		// --- it is possible to try to make the object equal to itself
		//     e.g. thisObject = thisObject; so this code catches that
		//     trivial case and just returns this object
		if (this == &params)
			return *this;

		// --- copy from params (argument) INTO our variables
		combFilterType = params.combFilterType;
		delayTime = params.delayTime;
		feedbackGain = params.feedbackGain;
		dryGain = params.dryGain;
		wetGain = params.wetGain;

		// --- MUST be last
		return *this;
	}

	// --- individual parameters
	CombFilterType combFilterType = CombFilterType::combFilter;
	double delayTime = 0.0;
	double feedbackGain = 0.0;
	double dryGain = 1.0;
	double wetGain = 1.0;
};


/**
\class UniversalCombFilter
\ingroup FX-Objects
\brief
The UniversalCombFilter object implements ....

Audio I/O:
- Processes mono input to mono output.
- *** Optionally, process frame *** Modify this according to your object functionality

Control I/F:
- Use UniversalCombFilterParameters structure to get/set object params.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
class UniversalCombFilter : public IAudioSignalProcessor
{
public:
	UniversalCombFilter(void) {}	/* C-TOR */
	~UniversalCombFilter(void) {}	/* D-TOR */

public:
	/** reset members to initialized state */
	virtual bool reset(double _sampleRate)
	{   // --- store the sample rate   
		sampleRate = (_sampleRate); 

		// --- do any other per-audio-run inits here   
		combDelayLine.reset(_sampleRate);   
		combDelayLine.createDelayBuffer(_sampleRate, 500.0); 

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
		double delay = 0.0; 
		double feedback = 0.0;

		//Read delay
		double delayedSample = combDelayLine.readDelayAtTime_mSec(universalCombFilterParameters.delayTime);

		// --- do your DSP magic here to create yn

		if (universalCombFilterParameters.combFilterType == CombFilterType::inverseCombFilter)
		{
			delay = xn;
			yn = delayedSample * universalCombFilterParameters.wetGain + xn * universalCombFilterParameters.dryGain;
		}
		else
		{
			feedback = delayedSample * universalCombFilterParameters.feedbackGain;
			delay = xn + feedback;
			yn = delayedSample * universalCombFilterParameters.wetGain;
		} 

		//Write delay
		combDelayLine.writeDelay(delay);
		

		return yn;
	}

	/** query to see if this object can process frames */
	virtual bool canProcessAudioFrame() { return false; } // <-- change this!

	/** process audio frame: implement this function if you answer "true" to above query */
	virtual bool processAudioFrame(const float* inputFrame,	/* ptr to one frame of data: pInputFrame[0] = left, pInputFrame[1] = right, etc...*/
					     float* outputFrame,
					     uint32_t inputChannels,
					     uint32_t outputChannels)
	{
		// --- do nothing
		return false; // NOT handled
	}


	/** get parameters: note use of custom structure for passing param data */
	/**
	\return UniversalCombFilterParameters custom data structure
	*/
	UniversalCombFilterParameters getParameters()
	{
		return universalCombFilterParameters;
	}

	/** set parameters: note use of custom structure for passing param data */
	/**
	\param UniversalCombFilterParameters custom data structure
	*/
	void setParameters(const UniversalCombFilterParameters& _params)
	{
		// --- copy them; note you may choose to ignore certain items
		//     and copy the variables one at a time, or you may test
		//     to see if cook-able variables have changed; if not, then
		//     do not re-cook them as it just wastes CPU
		universalCombFilterParameters = _params;

		SimpleDelayParameters params = combDelayLine.getParameters();
		params.delayTime_mSec = universalCombFilterParameters.delayTime;
		combDelayLine.setParameters(params);

		// --- cook parameters here
	}

private:
	UniversalCombFilterParameters universalCombFilterParameters; ///< object parameters
	SimpleDelay combDelayLine;

	// --- local variables used by this object
	double sampleRate = 0.0;	///< sample rate

};

#endif