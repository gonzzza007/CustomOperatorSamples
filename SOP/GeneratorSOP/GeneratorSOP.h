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

#ifndef __GeneratorSOP__
#define __GeneratorSOP__

#include "SOP_CPlusPlusBase.h"
#include "ShapeGenerator.h"
#include "Parameters.h"

#include <string>

/*
This example implements a SOP which takes the following parameters:
	- Shape: One of [Point, Line, Square, Cube] which is the shape this SOP outputs.
	- Color: The color applied to the shape.
	- GPU Direct: Whether the shape is loaded to the GPU.

This SOP is a generator and it takes no input.
*/

// Check methods [getNumInfoCHOPChans, getInfoCHOPChan, getInfoDATSize, getInfoDATEntries]
// if you want to output values to the Info CHOP/DAT

// To get more help about these functions, look at SOP_CPlusPlusBase.h
class GeneratorSOP : public TD::SOP_CPlusPlusBase
{
public:
	GeneratorSOP(const TD::OP_NodeInfo* info);
	virtual ~GeneratorSOP();

	virtual void		getGeneralInfo(TD::SOP_GeneralInfo*, const TD::OP_Inputs*, void*) override;

	virtual void		execute(TD::SOP_Output*, const TD::OP_Inputs*, void*) override;

	virtual void		executeVBO(TD::SOP_VBOOutput*, const TD::OP_Inputs*, void*) override;

	virtual void		setupParameters(TD::OP_ParameterManager*, void*) override;

	virtual void		getErrorString(TD::OP_String*, void*) override;

	virtual void		getWarningString(TD::OP_String*, void*) override;

private:
	// Class to construct geometry
	ShapeGenerator myShapeGenerator;


	std::string myError;
	std::string	myWarning;


	Parameters myParms;
};

#endif // !__GeneratorSOP__
