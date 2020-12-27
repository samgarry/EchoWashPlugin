// -----------------------------------------------------------------------------
//    ASPiK Plugin Kernel File:  plugincore.cpp
//
/**
    \file   plugincore.cpp
    \author Will Pirkle
    \date   17-September-2018
    \brief  Implementation file for PluginCore object
    		- http://www.aspikplugins.com
    		- http://www.willpirkle.com
*/
// -----------------------------------------------------------------------------
#include "plugincore.h"
#include "plugindescription.h"

/**
\brief PluginCore constructor is launching pad for object initialization

Operations:
- initialize the plugin description (strings, codes, numbers, see initPluginDescriptors())
- setup the plugin's audio I/O channel support
- create the PluginParameter objects that represent the plugin parameters (see FX book if needed)
- create the presets
*/
PluginCore::PluginCore()
{
    // --- describe the plugin; call the helper to init the static parts you setup in plugindescription.h
    initPluginDescriptors();

    // --- default I/O combinations
	// --- for FX plugins
	if (getPluginType() == kFXPlugin)
	{
		addSupportedIOCombination({ kCFMono, kCFMono });
		addSupportedIOCombination({ kCFMono, kCFStereo });
		addSupportedIOCombination({ kCFStereo, kCFStereo });
	}
	else // --- synth plugins have no input, only output
	{
		addSupportedIOCombination({ kCFNone, kCFMono });
		addSupportedIOCombination({ kCFNone, kCFStereo });
	}

	// --- for sidechaining, we support mono and stereo inputs; auxOutputs reserved for future use
	addSupportedAuxIOCombination({ kCFMono, kCFNone });
	addSupportedAuxIOCombination({ kCFStereo, kCFNone });

	// --- create the parameters
    initPluginParameters();

    // --- create the presets
    initPluginPresets();
}

/**
\brief create all of your plugin parameters here

\return true if parameters were created, false if they already existed
*/
bool PluginCore::initPluginParameters()
{
	if (pluginParameterMap.size() > 0)
		return false;

    // --- Add your plugin parameter instantiation code bewtween these hex codes
	// **--0xDEA7--**


	// --- Declaration of Plugin Parameter Objects 
	PluginParameter* piParam = nullptr;

	// --- continuous control: LFO1 Fo
	piParam = new PluginParameter(controlID::lfoOneFreq, "LFO1 Fo", "", controlVariableType::kDouble, 10.000000, 5000.000000, 10.000000, taper::kAntiLogTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lfoOneFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LFO2 Fo
	piParam = new PluginParameter(controlID::lfoTwoFreq, "LFO2 Fo", "", controlVariableType::kDouble, 10.000000, 5000.000000, 10.000000, taper::kAntiLogTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lfoTwoFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LFO3 Fo
	piParam = new PluginParameter(controlID::lfoThreeFreq, "LFO3 Fo", "", controlVariableType::kDouble, 10.000000, 5000.000000, 26.000000, taper::kAntiLogTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lfoThreeFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LFO1 Amp
	piParam = new PluginParameter(controlID::lfoOneAmp, "LFO1 Amp", "", controlVariableType::kDouble, 0.000000, 1.000000, 0.750000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lfoOneAmp, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LFO2 Amp
	piParam = new PluginParameter(controlID::lfoTwoAmp, "LFO2 Amp", "", controlVariableType::kDouble, 0.000000, 1.000000, 0.750000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lfoTwoAmp, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LFO3 Amp
	piParam = new PluginParameter(controlID::lfoThreeAmp, "LFO3 Amp", "", controlVariableType::kDouble, 0.000000, 1.000000, 0.750000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lfoThreeAmp, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Noise Depth %
	piParam = new PluginParameter(controlID::noiseDepthLvl, "Noise Depth %", "", controlVariableType::kDouble, 0.040000, 0.400000, 0.040000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&noiseDepthLvl, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Noise Filter Frequency
	piParam = new PluginParameter(controlID::noiseFilterFreq, "Noise Filter Frequency", "", controlVariableType::kDouble, 10.000000, 10000.000000, 3050.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&noiseFilterFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Noise Filter Amplitude
	piParam = new PluginParameter(controlID::noiseFilterLvl, "Noise Filter Amplitude", "", controlVariableType::kDouble, 0.000000, 2.000000, 0.350000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&noiseFilterLvl, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Delay Time
	piParam = new PluginParameter(controlID::delayTime, "Delay Time", "mS", controlVariableType::kDouble, 90.000000, 680.000000, 200.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&delayTime, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LPF BandPass Fc
	piParam = new PluginParameter(controlID::lpfBandFreq, "LPF BandPass Fc", "Hz", controlVariableType::kDouble, 10.000000, 10000.000000, 1500.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lpfBandFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: HPF BandPass FC
	piParam = new PluginParameter(controlID::hpfBandFreq, "HPF BandPass FC", "Hz", controlVariableType::kDouble, 20.000000, 15000.000000, 850.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&hpfBandFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Limiter Threshold
	piParam = new PluginParameter(controlID::limiterThresh, "Limiter Threshold", "dB", controlVariableType::kDouble, -60.000000, 12.000000, -32.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&limiterThresh, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Limiter Make Up Gain
	piParam = new PluginParameter(controlID::limiterGain, "Limiter Make Up Gain", "dB", controlVariableType::kDouble, -60.000000, 12.000000, 4.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&limiterGain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Record Lvl
	piParam = new PluginParameter(controlID::recordLvl, "Record Lvl", "dB", controlVariableType::kDouble, -60.000000, 12.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&recordLvl, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Playback Lvl
	piParam = new PluginParameter(controlID::playbackLvl, "Playback Lvl", "dB", controlVariableType::kDouble, -60.000000, 12.000000, 6.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&playbackLvl, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: 60 Hz Saturation
	piParam = new PluginParameter(controlID::saturation, "60 Hz Saturation", "", controlVariableType::kDouble, 0.000000, 8.000000, 2.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&saturation, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: 60 Hz Amplitude
	piParam = new PluginParameter(controlID::lfoAmplitude, "60 Hz Amplitude", "", controlVariableType::kDouble, 0.000000, 2.000000, 0.550000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lfoAmplitude, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Noise LPFc
	piParam = new PluginParameter(controlID::noiseFC, "Noise LPFc", "", controlVariableType::kDouble, 10.000000, 10000.000000, 6500.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&noiseFC, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Noise Amplitude
	piParam = new PluginParameter(controlID::noiseAmplitude, "Noise Amplitude", "", controlVariableType::kDouble, 0.000000, 0.600000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&noiseAmplitude, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Total Noise
	piParam = new PluginParameter(controlID::outputGain, "Total Noise", "", controlVariableType::kDouble, -60.000000, 12.000000, -35.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&outputGain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LFO Depth %
	piParam = new PluginParameter(controlID::lfoDepthLvl, "LFO Depth %", "", controlVariableType::kDouble, 0.000000, 0.200000, 0.100000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&lfoDepthLvl, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Feedback Percent
	piParam = new PluginParameter(controlID::feedbackPercent, "Feedback Percent", "", controlVariableType::kDouble, 0.000000, 1.000000, 0.500000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&feedbackPercent, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Dry Lvl
	piParam = new PluginParameter(controlID::dryLvl, "Dry Lvl", "dB", controlVariableType::kDouble, -60.000000, 12.000000, 6.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&dryLvl, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- Aux Attributes
	AuxParameterAttribute auxAttribute;

	// --- RAFX GUI attributes
	// --- controlID::lfoOneFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483712);
	setParamAuxAttribute(controlID::lfoOneFreq, auxAttribute);

	// --- controlID::lfoTwoFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483712);
	setParamAuxAttribute(controlID::lfoTwoFreq, auxAttribute);

	// --- controlID::lfoThreeFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483712);
	setParamAuxAttribute(controlID::lfoThreeFreq, auxAttribute);

	// --- controlID::lfoOneAmp
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483714);
	setParamAuxAttribute(controlID::lfoOneAmp, auxAttribute);

	// --- controlID::lfoTwoAmp
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483714);
	setParamAuxAttribute(controlID::lfoTwoAmp, auxAttribute);

	// --- controlID::lfoThreeAmp
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483714);
	setParamAuxAttribute(controlID::lfoThreeAmp, auxAttribute);

	// --- controlID::noiseDepthLvl
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::noiseDepthLvl, auxAttribute);

	// --- controlID::noiseFilterFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::noiseFilterFreq, auxAttribute);

	// --- controlID::noiseFilterLvl
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::noiseFilterLvl, auxAttribute);

	// --- controlID::delayTime
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::delayTime, auxAttribute);

	// --- controlID::lpfBandFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::lpfBandFreq, auxAttribute);

	// --- controlID::hpfBandFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::hpfBandFreq, auxAttribute);

	// --- controlID::limiterThresh
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::limiterThresh, auxAttribute);

	// --- controlID::limiterGain
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::limiterGain, auxAttribute);

	// --- controlID::recordLvl
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::recordLvl, auxAttribute);

	// --- controlID::playbackLvl
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::playbackLvl, auxAttribute);

	// --- controlID::saturation
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483651);
	setParamAuxAttribute(controlID::saturation, auxAttribute);

	// --- controlID::lfoAmplitude
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483651);
	setParamAuxAttribute(controlID::lfoAmplitude, auxAttribute);

	// --- controlID::noiseFC
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483657);
	setParamAuxAttribute(controlID::noiseFC, auxAttribute);

	// --- controlID::noiseAmplitude
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483657);
	setParamAuxAttribute(controlID::noiseAmplitude, auxAttribute);

	// --- controlID::outputGain
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483653);
	setParamAuxAttribute(controlID::outputGain, auxAttribute);

	// --- controlID::lfoDepthLvl
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::lfoDepthLvl, auxAttribute);

	// --- controlID::feedbackPercent
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::feedbackPercent, auxAttribute);

	// --- controlID::dryLvl
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::dryLvl, auxAttribute);


	// **--0xEDA5--**
   
    // --- BONUS Parameter
    // --- SCALE_GUI_SIZE
    PluginParameter* piParamBonus = new PluginParameter(SCALE_GUI_SIZE, "Scale GUI", "tiny,small,medium,normal,large,giant", "normal");
    addPluginParameter(piParamBonus);

	// --- create the super fast access array
	initPluginParameterArray();

    return true;
}

/**
\brief initialize object for a new run of audio; called just before audio streams

Operation:
- store sample rate and bit depth on audioProcDescriptor - this information is globally available to all core functions
- reset your member objects here

\param resetInfo structure of information about current audio format

\return true if operation succeeds, false otherwise
*/
bool PluginCore::reset(ResetInfo& resetInfo)
{
    // --- save for audio processing
    audioProcDescriptor.sampleRate = resetInfo.sampleRate;
    audioProcDescriptor.bitDepth = resetInfo.bitDepth;

	delayMod.reset(resetInfo.sampleRate);
	tapeDelay.reset(resetInfo.sampleRate);
	
	/*echoplexTapeDelayParameters params = tapeDelay.getParameters();
	params.updateType = delayUpdateType::kLeftAndRight;
	tapeDelay.setParameters(params);*/

    // --- other reset inits
    return PluginBase::reset(resetInfo);
}

/**
\brief one-time initialize function called after object creation and before the first reset( ) call

Operation:
- saves structure for the plugin to use; you can also load WAV files or state information here
*/
bool PluginCore::initialize(PluginInfo& pluginInfo)
{
	// --- add one-time init stuff here

	return true;
}

/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- syncInBoundVariables when preProcessAudioBuffers is called, it is *guaranteed* that all GUI control change information
  has been applied to plugin parameters; this binds parameter changes to your underlying variables
- NOTE: postUpdatePluginParameter( ) will be called for all bound variables that are acutally updated; if you need to process
  them individually, do so in that function
- use this function to bulk-transfer the bound variable data into your plugin's member object variables

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::preProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
    // --- sync internal variables to GUI parameters; you can also do this manually if you don't
    //     want to use the auto-variable-binding
    syncInBoundVariables();

    return true;
}

void PluginCore::updateParameters()
{
	echoplexDelayModulatorParameters params = delayMod.getParameters();
	params.delayTime = delayTime;
	params.lfoAmplitude[0] = lfoOneAmp;
	params.lfoAmplitude[1] = lfoTwoAmp;
	params.lfoAmplitude[2] = lfoThreeAmp;
	params.lfoDepth_Pct = lfoDepthLvl;
	params.noiseDepth_Pct = noiseDepthLvl;
	params.noiseFilterAmplitude = noiseFilterLvl;
	params.noiseFilterFc_Hz = noiseFilterFreq;
	delayMod.setParameters(params);
	/*
	SignalGenData delayModSignal = delayMod.renderAudioOutput();
	double delayModOutput = delayModSignal.normalOutput;
	*/

	echoplexTapeDelayParameters tapeParams = tapeDelay.getParameters();
	tapeParams.bandLimitHPFFc_Hz = hpfBandFreq;
	tapeParams.bandLimitLPFFc_Hz = lpfBandFreq;
	tapeParams.limiterThreshold_dB = limiterThresh;
	tapeParams.limiterMakeUpGain_dB = limiterGain;
	tapeParams.playbackLevel_dB = playbackLvl;
	tapeParams.recordLevel_dB = recordLvl;
	tapeParams.dryLevel_dB = dryLvl;
	//tapeParams.leftDelay_mSec = delayModOutput;
	//tapeParams.rightDelay_mSec = delayModOutput;
	tapeParams.feedback_Pct = feedbackPercent;
	
	tapeParams.outputAmplitude_dB = outputGain;
	tapeParams.waveshaperSaturation = saturation;
	tapeParams.sixtyHzNoiseAmplitude = lfoAmplitude;
	tapeParams.tapeNoiseFc_Hz = noiseFC;
	tapeParams.tapeNoiseAmplitude = noiseAmplitude;
	tapeDelay.setParameters(tapeParams);
}


/**
\brief frame-processing method

Operation:
- decode the plugin type - for synth plugins, fill in the rendering code; for FX plugins, delete the if(synth) portion and add your processing code
- note that MIDI events are fired for each sample interval so that MIDI is tightly sunk with audio
- doSampleAccurateParameterUpdates will perform per-sample interval smoothing

\param processFrameInfo structure of information about *frame* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processAudioFrame(ProcessFrameInfo& processFrameInfo)
{
    // --- fire any MIDI events for this sample interval
    processFrameInfo.midiEventQueue->fireMidiEvents(processFrameInfo.currentFrame);

	// --- do per-frame updates; VST automation and parameter smoothing
	doSampleAccurateParameterUpdates();
	updateParameters();

	double xnL = 0.0;
	double xnR = 0.0;
	
	SignalGenData delayModSignal = delayMod.renderAudioOutput();
	double delayModOutput = delayModSignal.normalOutput;

	echoplexTapeDelayParameters params = tapeDelay.getParameters();
	params.leftDelay_mSec = delayModOutput;
	params.rightDelay_mSec = delayModOutput;
	tapeDelay.setParameters(params);
	


//	double monoOutput = tapeDelay.processAudioSample(xnL);
    // --- FX Plugin:
    if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFMono)
    {
		// --- pass through code: change this with your signal processing
		xnL = processFrameInfo.audioInputFrame[0];
		processFrameInfo.audioOutputFrame[0] = xnL;


        return true; /// processed
    }

    // --- Mono-In/Stereo-Out
    else if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
        //processFrameInfo.audioOutputFrame[0] = processFrameInfo.audioInputFrame[0];
        //processFrameInfo.audioOutputFrame[1] = processFrameInfo.audioInputFrame[0];

		tapeDelay.processAudioFrame(processFrameInfo.audioInputFrame, processFrameInfo.audioOutputFrame,
			processFrameInfo.numAudioInChannels, processFrameInfo.numAudioOutChannels);

        return true; /// processed
    }

    // --- Stereo-In/Stereo-Out
    else if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFStereo &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
        //processFrameInfo.audioOutputFrame[0] = processFrameInfo.audioInputFrame[0];
        //processFrameInfo.audioOutputFrame[1] = processFrameInfo.audioInputFrame[1];

		tapeDelay.processAudioFrame(processFrameInfo.audioInputFrame, processFrameInfo.audioOutputFrame,
			processFrameInfo.numAudioInChannels, processFrameInfo.numAudioOutChannels);

		/*
		float inputs[2] = { xnL, xnR };
		float outputs[2] = { 0.0, 0.0 };
		outputs[0] = processFrameInfo.audioOutputFrame[0];
		outputs[1] = processFrameInfo.audioOutputFrame[1];
		tapeDelay.processAudioFrame(inputs, outputs, 2, 2);
		*/

        return true; /// processed
    }

    return false; /// NOT processed
}


/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- updateOutBoundVariables sends metering data to the GUI meters

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
	// --- update outbound variables; currently this is meter data only, but could be extended
	//     in the future
	updateOutBoundVariables();

    return true;
}

/**
\brief update the PluginParameter's value based on GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- use base class helper
    setPIParamValue(controlID, controlValue);

    // --- do any post-processing
    postUpdatePluginParameter(controlID, controlValue, paramInfo);

    return true; /// handled
}

/**
\brief update the PluginParameter's value based on *normlaized* GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param normalizedValue the new control value in normalized form
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameterNormalized(int32_t controlID, double normalizedValue, ParameterUpdateInfo& paramInfo)
{
	// --- use base class helper, returns actual value
	double controlValue = setPIParamValueNormalized(controlID, normalizedValue, paramInfo.applyTaper);

	// --- do any post-processing
	postUpdatePluginParameter(controlID, controlValue, paramInfo);

	return true; /// handled
}

/**
\brief perform any operations after the plugin parameter has been updated; this is one paradigm for
	   transferring control information into vital plugin variables or member objects. If you use this
	   method you can decode the control ID and then do any cooking that is needed. NOTE: do not
	   overwrite bound variables here - this is ONLY for any extra cooking that is required to convert
	   the GUI data to meaninful coefficients or other specific modifiers.

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postUpdatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- now do any post update cooking; be careful with VST Sample Accurate automation
    //     If enabled, then make sure the cooking functions are short and efficient otherwise disable it
    //     for the Parameter involved
    /*switch(controlID)
    {
        case 0:
        {
            return true;    /// handled
        }

        default:
            return false;   /// not handled
    }*/

    return false;
}

/**
\brief has nothing to do with actual variable or updated variable (binding)

CAUTION:
- DO NOT update underlying variables here - this is only for sending GUI updates or letting you
  know that a parameter was changed; it should not change the state of your plugin.

WARNING:
- THIS IS NOT THE PREFERRED WAY TO LINK OR COMBINE CONTROLS TOGETHER. THE PROPER METHOD IS
  TO USE A CUSTOM SUB-CONTROLLER THAT IS PART OF THE GUI OBJECT AND CODE.
  SEE http://www.willpirkle.com for more information

\param controlID the control ID value of the parameter being updated
\param actualValue the new control value

\return true if operation succeeds, false otherwise
*/
bool PluginCore::guiParameterChanged(int32_t controlID, double actualValue)
{
	/*
	switch (controlID)
	{
		case controlID::<your control here>
		{

			return true; // handled
		}

		default:
			break;
	}*/

	return false; /// not handled
}

/**
\brief For Custom View and Custom Sub-Controller Operations

NOTES:
- this is for advanced users only to implement custom view and custom sub-controllers
- see the SDK for examples of use

\param messageInfo a structure containing information about the incoming message

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMessage(MessageInfo& messageInfo)
{
	// --- decode message
	switch (messageInfo.message)
	{
		// --- add customization appearance here
	case PLUGINGUI_DIDOPEN:
	{
		return false;
	}

	// --- NULL pointers so that we don't accidentally use them
	case PLUGINGUI_WILLCLOSE:
	{
		return false;
	}

	// --- update view; this will only be called if the GUI is actually open
	case PLUGINGUI_TIMERPING:
	{
		return false;
	}

	// --- register the custom view, grab the ICustomView interface
	case PLUGINGUI_REGISTER_CUSTOMVIEW:
	{

		return false;
	}

	case PLUGINGUI_REGISTER_SUBCONTROLLER:
	case PLUGINGUI_QUERY_HASUSERCUSTOM:
	case PLUGINGUI_USER_CUSTOMOPEN:
	case PLUGINGUI_USER_CUSTOMCLOSE:
	case PLUGINGUI_EXTERNAL_SET_NORMVALUE:
	case PLUGINGUI_EXTERNAL_SET_ACTUALVALUE:
	{

		return false;
	}

	default:
		break;
	}

	return false; /// not handled
}


/**
\brief process a MIDI event

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param event a structure containing the MIDI event data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMIDIEvent(midiEvent& event)
{
	return true;
}

/**
\brief (for future use)

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param vectorJoysickData a structure containing joystick data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::setVectorJoystickParameters(const VectorJoystickData& vectorJoysickData)
{
	return true;
}

/**
\brief use this method to add new presets to the list

NOTES:
- see the SDK for examples of use
- for non RackAFX users that have large paramter counts, there is a secret GUI control you
  can enable to write C++ code into text files, one per preset. See the SDK or http://www.willpirkle.com for details

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginPresets()
{
	// **--0xFF7A--**

	// --- Plugin Presets 
	int index = 0;
	PresetInfo* preset = nullptr;

	// --- Preset: Factory Preset
	preset = new PresetInfo(index++, "Factory Preset");
	initPresetParameters(preset->presetParameters);
	setPresetParameter(preset->presetParameters, controlID::lfoOneFreq, 10.000000);
	setPresetParameter(preset->presetParameters, controlID::lfoTwoFreq, 10.000000);
	setPresetParameter(preset->presetParameters, controlID::lfoThreeFreq, 26.000000);
	setPresetParameter(preset->presetParameters, controlID::lfoOneAmp, 0.750000);
	setPresetParameter(preset->presetParameters, controlID::lfoTwoAmp, 0.750000);
	setPresetParameter(preset->presetParameters, controlID::lfoThreeAmp, 0.750000);
	setPresetParameter(preset->presetParameters, controlID::noiseDepthLvl, 0.040000);
	setPresetParameter(preset->presetParameters, controlID::noiseFilterFreq, 3050.000000);
	setPresetParameter(preset->presetParameters, controlID::noiseFilterLvl, 0.350000);
	setPresetParameter(preset->presetParameters, controlID::delayTime, 200.000000);
	setPresetParameter(preset->presetParameters, controlID::lpfBandFreq, 1500.000000);
	setPresetParameter(preset->presetParameters, controlID::hpfBandFreq, 850.000000);
	setPresetParameter(preset->presetParameters, controlID::limiterThresh, -32.000000);
	setPresetParameter(preset->presetParameters, controlID::limiterGain, 4.000000);
	setPresetParameter(preset->presetParameters, controlID::recordLvl, 0.000000);
	setPresetParameter(preset->presetParameters, controlID::playbackLvl, 6.000000);
	setPresetParameter(preset->presetParameters, controlID::saturation, 2.000000);
	setPresetParameter(preset->presetParameters, controlID::lfoAmplitude, 0.550000);
	setPresetParameter(preset->presetParameters, controlID::noiseFC, 6499.999512);
	setPresetParameter(preset->presetParameters, controlID::noiseAmplitude, 0.000000);
	setPresetParameter(preset->presetParameters, controlID::outputGain, -35.000000);
	setPresetParameter(preset->presetParameters, controlID::lfoDepthLvl, 0.100000);
	setPresetParameter(preset->presetParameters, controlID::feedbackPercent, 0.500000);
	setPresetParameter(preset->presetParameters, controlID::dryLvl, 6.000000);
	addPreset(preset);


	// **--0xA7FF--**

    return true;
}

/**
\brief setup the plugin description strings, flags and codes; this is ordinarily done through the ASPiKreator or CMake

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginDescriptors()
{
    pluginDescriptor.pluginName = PluginCore::getPluginName();
    pluginDescriptor.shortPluginName = PluginCore::getShortPluginName();
    pluginDescriptor.vendorName = PluginCore::getVendorName();
    pluginDescriptor.pluginTypeCode = PluginCore::getPluginType();

	// --- describe the plugin attributes; set according to your needs
	pluginDescriptor.hasSidechain = kWantSidechain;
	pluginDescriptor.latencyInSamples = kLatencyInSamples;
	pluginDescriptor.tailTimeInMSec = kTailTimeMsec;
	pluginDescriptor.infiniteTailVST3 = kVSTInfiniteTail;

    // --- AAX
    apiSpecificInfo.aaxManufacturerID = kManufacturerID;
    apiSpecificInfo.aaxProductID = kAAXProductID;
    apiSpecificInfo.aaxBundleID = kAAXBundleID;  /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.aaxEffectID = "aaxDeveloper.";
    apiSpecificInfo.aaxEffectID.append(PluginCore::getPluginName());
    apiSpecificInfo.aaxPluginCategoryCode = kAAXCategory;

    // --- AU
    apiSpecificInfo.auBundleID = kAUBundleID;
	apiSpecificInfo.auBundleName = kAUBundleName;   /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.auBundleName = kAUBundleName;

    // --- VST3
    apiSpecificInfo.vst3FUID = PluginCore::getVSTFUID(); // OLE string format
    apiSpecificInfo.vst3BundleID = kVST3BundleID;/* MacOS only: this MUST match the bundle identifier in your info.plist file */
	apiSpecificInfo.enableVST3SampleAccurateAutomation = kVSTSAA;
	apiSpecificInfo.vst3SampleAccurateGranularity = kVST3SAAGranularity;

    // --- AU and AAX
    apiSpecificInfo.fourCharCode = PluginCore::getFourCharCode();

    return true;
}

// --- static functions required for VST3/AU only --------------------------------------------- //
const char* PluginCore::getPluginBundleName() { return kAUBundleName; }
const char* PluginCore::getPluginName(){ return kPluginName; }
const char* PluginCore::getShortPluginName(){ return kShortPluginName; }
const char* PluginCore::getVendorName(){ return kVendorName; }
const char* PluginCore::getVendorURL(){ return kVendorURL; }
const char* PluginCore::getVendorEmail(){ return kVendorEmail; }
const char* PluginCore::getAUCocoaViewFactoryName(){ return AU_COCOA_VIEWFACTORY_STRING; }
pluginType PluginCore::getPluginType(){ return kPluginType; }
const char* PluginCore::getVSTFUID(){ return kVSTFUID; }
int32_t PluginCore::getFourCharCode(){ return kFourCharCode; }
