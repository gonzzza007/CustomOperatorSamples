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
#ifndef __OpticalFlowCPUTOP__
#define __OpticalFlowCPUTOP__

#include "TOP_CPlusPlusBase.h"

namespace cv
{
	class Mat;
}

/*
This example implements a TOP to expose cv::cuda::FarnebackOpticalFlow class functionallity. For
more information on the parameters check 
https://docs.opencv.org/3.4/d9/d30/classcv_1_1cuda_1_1FarnebackOpticalFlow.html

It takes the following parameters:
	- Num Levels:	Number of pyramid layers including the intial image.
	- Pyramid Scale:	Image scale to build pyramid layers.
	- Window Size:	Averaging window size.
	- Iterations:	Number of iteration at each pyramid level.
	- Poly N:	Size of the pixel neighborhood used to find polynomial expansion in each pixel.
	- Poly Sigma:	Standard deviation of the Gaussian thta is used to smooth derivatives used as
		basis for the polynomial expansion.
	- Use Gaussian Filter:	Uses the Gaussian Window Size x Window Size filter instead of a box filter.
	- Use Previous Flow:	Use the optical flow of the previous frame as an estimate for the current frame.

This TOP takes one input where the optical flow of sequencial frames is calculated.
*/

enum class OP_TOPInputDownloadType;

enum class Channel
{
	R,
	Mono = R,
	G,
	Second = G,
	B,
	A,
	Invalid
};

// To get more help about these functions, look at TOP_CPlusPlusBase.h
class OpticalFlowCPUTOP : public TOP_CPlusPlusBase
{
public:
    OpticalFlowCPUTOP(const OP_NodeInfo *info);
    virtual ~OpticalFlowCPUTOP();

        virtual void		getGeneralInfo(TOP_GeneralInfo*, const OP_Inputs*, void* reserved) override;

        virtual bool		getOutputFormat(TOP_OutputFormat*, const OP_Inputs*, void* reserved) override;

        virtual void		execute(TOP_OutputFormatSpecs*, const OP_Inputs*, TOP_Context*, void* reserved) override;

	virtual void		setupParameters(OP_ParameterManager*, void* reserved) override;

private:
        void                            handleParameters(const OP_Inputs*);

        void                            inputToMat(const OP_Inputs*) const;

	void 				cvMatToOutput(const cv::Mat&, TOP_OutputFormatSpecs*) const;

	cv::Mat*	myFrame;
	cv::Mat*	myPrev;
	cv::Mat*	myFlow;

    // Parameters
	int		myNumLevels;
	double	myPyrScale;
	int		myWinSize;
	int		myNumIter;
	int		myPolyN;
	double	myPolySigma;
	int		myFlags;
        OP_TOPInputDownloadType myDownloadtype;
        Channel         myChannel;
};

#endif