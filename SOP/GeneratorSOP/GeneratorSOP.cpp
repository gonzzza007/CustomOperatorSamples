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

#include "GeneratorSOP.h"
#include "Parameters.h"

#include <cassert>
#include <array>

using namespace TD;

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
void
FillSOPPluginInfo(SOP_PluginInfo *info)
{
	// For more information on CHOP_PluginInfo see CHOP_CPlusPlusBase.h

	// Always set this to CHOPCPlusPlusAPIVersion.
	info->apiVersion = SOPCPlusPlusAPIVersion;

	// For more information on OP_CustomOPInfo see CPlusPlus_Common.h
	OP_CustomOPInfo& customInfo = info->customOPInfo;

	// Unique name of the node which starts with an upper case letter, followed by lower case letters or numbers
	customInfo.opType->setString("Generator4");
	// English readable name
	customInfo.opLabel->setString("Generator4");
	// Information of the author of the node
	customInfo.authorName->setString("Gabriel Robels");
	customInfo.authorEmail->setString("support@derivative.ca");

	// This CHOP takes one input
	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
}

DLLEXPORT
SOP_CPlusPlusBase*
CreateSOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new GeneratorSOP(info);
}

DLLEXPORT
void
DestroySOPInstance(SOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	delete (GeneratorSOP*)instance;
}

};


GeneratorSOP::GeneratorSOP(const OP_NodeInfo*) :
	myShapeGenerator{} 
{
};

GeneratorSOP::~GeneratorSOP()
{
};

void
GeneratorSOP::getGeneralInfo(SOP_GeneralInfo* ginfo, const TD::OP_Inputs* inputs, void*)
{
	// This will cause the node to cook every frame if the output is used
	ginfo->cookEveryFrameIfAsked = false;

	// Direct shape to GPU loading if asked 
	ginfo->directToGPU = myParms.evalGpudirect(inputs);
}

void
GeneratorSOP::execute(SOP_Output* output, const TD::OP_Inputs* inputs, void*)
{
	ShapeMenuItems shape = myParms.evalShape(inputs);
	Color color = myParms.evalColor(inputs);
	float scale = myParms.evalScale(inputs);
	float spread = myParms.evalSpread(inputs);

	const OP_CHOPInput* input = inputs->getParCHOP(PointsChopName);

	switch (shape)
	{
		case ShapeMenuItems::Point:
		{
			myShapeGenerator.outputDot(output);
			break;
		}
		case ShapeMenuItems::Line:
		{
			myShapeGenerator.outputLine(output);
			break;
		}
		case ShapeMenuItems::Square:
		{
			myShapeGenerator.outputSquare(output);
			break;
		}
		case ShapeMenuItems::Cube:
		{
			myShapeGenerator.outputCube(output);
			break;
		}
		case ShapeMenuItems::Divider:
		{
			if (!input) {
				myError = "Missing CHOP input for Divider point positions.";
				return;
			}

			if (input->numChannels < 3) {
				myError = "To be used as 3D position data CHOP input needs to have 3 channels." + std::to_string(input->numChannels) + ".";
				return;
			}

			myShapeGenerator.outputDivider(input, scale, spread, output);
			break;
		}
		case ShapeMenuItems::Voronoi:
		{
			if (!input) {
				myError = "Missing CHOP input for Voronoi cell point positions.";
				return;
			}

			if (input->numChannels < 3) {
				myError = "To be used as 3D position data CHOP input needs to have 3 channels." + std::to_string(input->numChannels) + ".";
				return;
			}


			// generate some data if there's no input...
			if (input->numSamples < 2) {
				myWarning = "Not enough input CHOP point positions. Need at least 2.";
			}

			myShapeGenerator.outputVoronoi(input, scale, spread, output);
			break;
		}
		case ShapeMenuItems::KDTree:
		default:
		{

			if (!input) {
				myError = "Missing CHOP input for KD-Tree cell point positions.";
				return;
			}

			if (input->numChannels < 3) {
				myError = "To be used as 3D position data CHOP input needs to have 3 channels." + std::to_string(input->numChannels) + ".";
				return;
			}


			// generate some data if there's no input...
			// if (input->numSamples < 2) {
			//	myWarning = "Not enough point positions. Adding random.";
			//}

			myShapeGenerator.outputKDTree(input, scale, spread, output);
			break;
		}
	}

	for (int i = 0; i < output->getNumPoints(); ++i)
	{
		output->setColor(color, i);
	}

	output->setBoundingBox(BoundingBox(-1, -1, -1, 1, 1, 1));
}

void
GeneratorSOP::executeVBO(SOP_VBOOutput* output, const TD::OP_Inputs* inputs, void*)
{
	ShapeMenuItems shape = myParms.evalShape(inputs);
	Color color = myParms.evalColor(inputs);

	output->enableColor();
	output->enableNormal();
	output->enableTexCoord(1);

	switch (shape)
	{
		case ShapeMenuItems::Point:
		{
			myShapeGenerator.outputDotVBO(output);
			break;
		}
		case ShapeMenuItems::Line:
		{
			myShapeGenerator.outputLineVBO(output);
			break;
		}
		case ShapeMenuItems::Square:
		{
			myShapeGenerator.outputSquareVBO(output);
			break;
		}
		case ShapeMenuItems::Cube:
		default:
		{
			myShapeGenerator.outputCubeVBO(output);
			break;
		}
	}

	int numVertices = myShapeGenerator.getLastVBONumVertices();

	Color* colors = output->getColors();
	for (int i = 0; i < numVertices; ++i)
	{
		colors[i] = color;
	}

	output->setBoundingBox(BoundingBox(-1, -1, -1, 1, 1, 1));
	output->updateComplete();
}

void
GeneratorSOP::getErrorString(TD::OP_String* error, void*)
{
	error->setString(myError.c_str());
	// Reset string after reporting it.
	myError = "";
}

void
GeneratorSOP::getWarningString(TD::OP_String* warning, void*)
{
	warning->setString(myWarning.c_str());
	// Reset string after reporting it.
	myWarning = "";
}

void
GeneratorSOP::setupParameters(TD::OP_ParameterManager* manager, void*)
{
	myParms.setup(manager);
}