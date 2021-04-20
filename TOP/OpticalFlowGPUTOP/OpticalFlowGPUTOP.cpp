/* Shared Use License: This file is owned by Derivative Inc. (Derivative)
* and can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement
* (which also govern the use of this file). You may share or redistribute
* a modified version of this file provided the following conditions are met:
*
* 1. The shared file or redistribution must retain the information set out
* above and this list of conditions.
* 2. Derivative's name (Derivative Inc.) or its trademarks may not be used
* to endorse or promote products derived from this file without specific
* prior written permission from Derivative.
*/

#include "OpticalFlowGPUTOP.h"
#include "Parameters.h"

#include <cassert>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "cuda_runtime.h"

extern void arrayToMatGPU8UC4(int width, int height, cudaArray* input, cv::cuda::GpuMat& output);
extern void matGPUToArray32FC2(int width, int height, const cv::cuda::GpuMat& input, cudaArray* output);

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
void
FillTOPPluginInfo(TOP_PluginInfo* info)
{
	// This must always be set to this constant
	info->apiVersion = TOPCPlusPlusAPIVersion;

	// Change this to change the executeMode behavior of this plugin.
	info->executeMode = TOP_ExecuteMode::CUDA;

	// For more information on OP_CustomOPInfo see CPlusPlus_Common.h
	OP_CustomOPInfo& customInfo = info->customOPInfo;

	// Unique name of the node which starts with an upper case letter, followed by lower case letters or numbers
	customInfo.opType->setString("Opticalflowgpu");
	// English readable name
	customInfo.opLabel->setString("Optical Flow GPU");
	// Information of the author of the node
	customInfo.authorName->setString("Gabriel Robels");
	customInfo.authorEmail->setString("support@derivative.ca");

	// This TOP takes one input
	customInfo.minInputs = 1;
	customInfo.maxInputs = 1;
}

DLLEXPORT
TOP_CPlusPlusBase*
CreateTOPInstance(const OP_NodeInfo* info, TOP_Context* context)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per TOP that is using the .dll
	return new OpticalFlowGPUTOP(info);
}

DLLEXPORT
void
DestroyTOPInstance(TOP_CPlusPlusBase* instance, TOP_Context *context)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the TOP using that instance is deleted, or
	// if the TOP loads a different DLL
	delete (OpticalFlowGPUTOP*)instance;
}

};


OpticalFlowGPUTOP::OpticalFlowGPUTOP(const OP_NodeInfo*) :
	myFrame{ new cv::cuda::GpuMat() }, myPrev{ new cv::cuda::GpuMat() }, myFlow{ new cv::cuda::GpuMat() }
{
}

OpticalFlowGPUTOP::~OpticalFlowGPUTOP()
{
	delete myFrame;
	delete myPrev;
	delete myFlow;
}

void
OpticalFlowGPUTOP::getGeneralInfo(TOP_GeneralInfo* ginfo, const OP_Inputs*, void*)
{
	ginfo->cookEveryFrameIfAsked = false;
}

bool
OpticalFlowGPUTOP::getOutputFormat(TOP_OutputFormat* format, const OP_Inputs* inputs, void*)
{
	// In this function we could assign variable values to 'format' to specify
	// the pixel format/resolution etc that we want to output to.
	// If we did that, we'd want to return true to tell the TOP to use the settings we've
	// specified.
	// In this example we want a dual channel for output uv vectors
	format->bitsPerChannel = 32;
	format->floatPrecision = true;
	format->redChannel = true;
	format->greenChannel = true;
	format->blueChannel = false;
	format->alphaChannel = false;

	// Query size of top, otherwise if 'Common/Output Resolution' is different than 'Use Input'
	// It scales the size twice
	const OP_TOPInput* top = inputs->getInputTOP(0);
	if (top)
	{
		format->width = top->width;
		format->height = top->height;
	}

	return true;
}


void
OpticalFlowGPUTOP::execute(TOP_OutputFormatSpecs* output, const OP_Inputs* inputs, TOP_Context*, void*)
{
	using namespace cv;

	const OP_TOPInput* top = inputs->getInputTOP(0);
	if (!top || !checkInputTop(top))
		return;
	
	inputTopToMat(top);
	if (myFrame->empty())
		return;

	Size outSize = Size(output->width, output->height);
	cuda::resize(*myFrame, *myFrame, outSize);

	cuda::cvtColor(*myFrame, *myFrame, cv::COLOR_BGRA2GRAY);

	if (myPrev->empty() || myPrev->size() != outSize)
	{
		*myPrev = std::move(*myFrame);
		return;
	}

	if (myFlow->empty() || myFlow->size() != outSize)
	{
		*myFlow = cuda::GpuMat(outSize, CV_32FC2);
	}
	
	int myFlags = myParms.evalUsegaussianfilter(inputs) ? cv::OPTFLOW_FARNEBACK_GAUSSIAN : 0;
	Ptr<cuda::FarnebackOpticalFlow> optFlow = cuda::FarnebackOpticalFlow::create(myParms.evalNumlevels(inputs), myParms.evalPyramidscale(inputs), myParms.evalFastpyramids(inputs), myParms.evalWindowsize(inputs), myParms.evalIterations(inputs),
		myParms.evalPolyn(inputs), myParms.evalPolysigma(inputs), myFlags);

	optFlow->calc(*myPrev, *myFrame, *myFlow);

	*myPrev = std::move(*myFrame);

	cvMatToOutput(*myFlow, output);
}

void
OpticalFlowGPUTOP::setupParameters(OP_ParameterManager* manager, void*)
{
	myParms.setup(manager);
}

void 
OpticalFlowGPUTOP::getErrorString(OP_String* error, void*)
{
	error->setString(myError.c_str());
	myError.clear();
}

void 
OpticalFlowGPUTOP::cvMatToOutput(const cv::cuda::GpuMat& M, TOP_OutputFormatSpecs* out) const
{
	matGPUToArray32FC2(out->width, out->height, M, out->cudaOutput[0]);
}

void 
OpticalFlowGPUTOP::inputTopToMat(const OP_TOPInput* top) const
{
	*myFrame = cv::cuda::GpuMat(top->height, top->width, CV_8UC4);
	arrayToMatGPU8UC4(top->width, top->height, top->cudaInput, *myFrame);
}

bool
OpticalFlowGPUTOP::checkInputTop(const OP_TOPInput* topInput)
{
	if (topInput->pixelFormat != GL_RGBA8)
	{
		myError = "CUDA Kernel is currently only written to handle 8-bit RGBA textures.";
		return false;
	}
	if (topInput->cudaInput == nullptr)
	{
		myError = "CUDA memory for input TOP was not mapped correctly.";
		return false;
	}
	return true;
}