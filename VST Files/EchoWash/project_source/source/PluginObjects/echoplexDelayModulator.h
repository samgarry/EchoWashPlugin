#pragma once

#ifndef __echoplexDelayModulator__
#define __echoplexDelayModulator__

#include "fxobjects.h"
#include "sytemNoiseGenerator.h"
#include "tripleLFOModulator.h"
#include "UniversalCombFilter.h"

/**
\struct echoplexDelayModulatorParameters
\ingroup FX-Objects
\brief
Custom parameter structure for the echoplexDelayModulator object.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
struct echoplexDelayModulatorParameters
{
	echoplexDelayModulatorParameters() {}

	/** all FXObjects parameter objects require overloaded= operator so remember to add new entries if you add new variables. */
	echoplexDelayModulatorParameters& operator=(const echoplexDelayModulatorParameters& params)	// need this override for collections to work
	{
		// --- it is possible to try to make the object equal to itself
		//     e.g. thisObject = thisObject; so this code catches that
		//     trivial case and just returns this object
		if (this == &params)
			return *this;

		// --- copy from params (argument) INTO our variables
		//myVariable = params.myVariable;
		for (int i = 0; i <= 2; i++)
		{
			lfoFrequency_Hz[i] = params.lfoFrequency_Hz[i];
			//lfoStartPhase[i] = params.lfoStartPhase[i];
			lfoAmplitude[i] = params.lfoAmplitude[i];
		}
		lfoDepth_Pct = params.lfoDepth_Pct;
		noiseDepth_Pct = params.noiseDepth_Pct;
		noiseFilterFc_Hz = params.noiseFilterFc_Hz;
		noiseFilterAmplitude = params.noiseFilterAmplitude;
		//combFilterDelayTime_mSec = params.combFilterDelayTime_mSec;
		delayTime = params.delayTime;

		// --- MUST be last
		return *this;
	}

	// --- individual parameters
	double lfoFrequency_Hz[3] = { 2.5, 5.0, 26.0 };  
	//double lfoStartPhase[NUM_LFOS] = { 0.0, 0.0, 0.0 }; 
	double lfoAmplitude[3] = { 0.0, 0.0, 0.0 }; 
	double lfoDepth_Pct = 0.5;			// voice the plugin for default val 
	double noiseDepth_Pct = 0.5;		// voice the plugin for default val 
	double noiseFilterFc_Hz = 50.0;		// voice the plugin for default val 
	double noiseFilterAmplitude = 1.0;  // voice the plugin for default val 
	//double combFilterDelayTime_mSec = 10.0; 
	double delayTime = 0.0;   // --- 90.0 to 680mSec for me 


	//data_type myVariable = 0.0;	///< init
};


/**
\class echoplexDelayModulator
\ingroup FX-Objects
\brief
The echoplexDelayModulator object implements ....

Audio I/O:
- Processes mono input to mono output.
- *** Optionally, process frame *** Modify this according to your object functionality

Control I/F:
- Use echoplexDelayModulatorParameters structure to get/set object params.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
class echoplexDelayModulator : public IAudioSignalGenerator
{
public:
	echoplexDelayModulator(void) {}	/* C-TOR */
	~echoplexDelayModulator(void) {}	/* D-TOR */

public:
	/** reset members to initialized state */
	virtual bool reset(double _sampleRate)
	{
		// --- store the sample rate
		sampleRate = (_sampleRate);

		// --- do any other per-audio-run inits here

		// triple lfo
		capstanPinchModulator.reset(_sampleRate);

		// noise generator
		lfDriftModulator.reset(_sampleRate);

		// comb filter 
		scallopingFilter.reset(_sampleRate); 
		// --- inverse comb filter here 
		UniversalCombFilterParameters combParams =  scallopingFilter.getParameters(); 
		combParams.combFilterType = CombFilterType::inverseCombFilter; 
		//combParams.delayTime = echoplexParams.combFilterDelayTime_mSec;
		scallopingFilter.setParameters(combParams);


		return true;
	}

	/** render a new audio output structure */
	virtual const SignalGenData renderAudioOutput()
	{
		SignalGenData generatorOutput;

		// --- LFO MODULATOR
		double lfoModulatorValue = echoplexParams.lfoDepth_Pct;
		SignalGenData lfoSound = capstanPinchModulator.renderAudioOutput();
		double lfoOutput = lfoSound.normalOutput;
		double lfoModulation = doBipolarModulation(lfoModulatorValue, -5.0, 5.0) * lfoOutput;
		 

		// --- NOISE MODULATOR
		double noiseModulatorValue = echoplexParams.noiseDepth_Pct;
		NoiseGenData noiseSound = lfDriftModulator.renderAudioOutput();
		double noiseOutput = noiseSound.filteredWhiteNoiseOut;

		// Drift Depth
		double originalDelayTime = echoplexParams.delayTime;
		double normalizedDelayTime = normalizeValue(originalDelayTime, 90, 680);
		double modulationDepth = calculateDriftDepth(normalizedDelayTime);
		
		double noiseModulation = doBipolarModulation(modulationDepth, -5.0, 5.0);
		double noiseModulationProduct = noiseModulation * noiseModulatorValue * noiseOutput;

		double totalMod = echoplexParams.delayTime + lfoModulation + noiseModulationProduct;

		generatorOutput.normalOutput = scallopingFilter.processAudioSample(totalMod);

		return generatorOutput;
	}

	/** get parameters: note use of custom structure for passing param data */
	/**
	\return echoplexDelayModulatorParameters custom data structure
	*/
	echoplexDelayModulatorParameters getParameters()
	{
		return echoplexParams;
	}

	/** set parameters: note use of custom structure for passing param data */
	/**
	\param echoplexDelayModulatorParameters custom data structure
	*/
	void setParameters(const echoplexDelayModulatorParameters& _params)
	{
		// --- copy them; note you may choose to ignore certain items
		//     and copy the variables one at a time, or you may test
		//     to see if cook-able variables have changed; if not, then
		//     do not re-cook them as it just wastes CPU
		echoplexParams = _params;

		// --- setup the mapped value; this function passes by reference  
		//     so the value is modified directly 
		double mappedValue = echoplexParams.delayTime; 
		// --- call the mapping function 
		mapDoubleValue(mappedValue, 90, 680, 0.5, 5.0); 
		
		UniversalCombFilterParameters combParams = scallopingFilter.getParameters();
		// --- use the value as the inverse comb delay length 
		combParams.delayTime = mappedValue;
		scallopingFilter.setParameters(combParams);

		// --- MY CODE BELOW
		// triple lfo
		for (int i = 0; i <= 2; i++)
		{
			tripleLFOModulatorParameters params = capstanPinchModulator.getParameters();
			params.lfoFrequency_Hz[i] = echoplexParams.lfoFrequency_Hz[i];
			params.lfoAmplitude[i] = echoplexParams.lfoAmplitude[i];
			capstanPinchModulator.setParameters(params);
		}

		// noise generator
		NoiseGeneratorParameters noiseParams = lfDriftModulator.getParameters();
		noiseParams.lpf_fc_Hz = echoplexParams.noiseFilterFc_Hz;
		noiseParams.outputAmplitude = echoplexParams.noiseFilterAmplitude;
		lfDriftModulator.setParameters(noiseParams);

		// --- cook parameters here
	}

	// --- polynomial curve fit to Fig 7 inline 
	double normalizeValue(double value, double min, double max) 
	{ 
		return (value - min) / (max - min); 
	} 



	double mapDoubleValue(double value, double inputMin, double inputMax, double outputMin, double outputMax)
	{
		double slope = 1.0 * (outputMax - outputMin) / (inputMax - inputMin);

		value = outputMin + floor(slope * (value - inputMin) + 0.5);
		
		return value;
	}


	double calculateDriftDepth(double normalizedDelayTime) 
	{
		double y = 0.1803571 - 1.524107 * normalizedDelayTime + 5.245536 * pow(normalizedDelayTime, 2);  

			// --- I am verbose here (calculating y then returning it) so 
			//     I can examine it during debug; then I can streamline later 

			return y;
	}

protected:
	echoplexDelayModulatorParameters echoplexParams; ///< object parameters
	tripleLFOModulator capstanPinchModulator;
	NoiseGenerator lfDriftModulator;
	UniversalCombFilter scallopingFilter;

	

	// --- local variables used by this object
	double sampleRate = 0.0;	///< sample rate

};

#endif