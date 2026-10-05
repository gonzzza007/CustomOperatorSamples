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

#include "OpenCascadeSOP.h"
#include "Parameters.h"

#include "OcctMesher.h"

#include <cstring>
#include <vector>




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
	// Check to make sure the running TD version supports our API version.
	if (!info->setAPIVersion(SOPCPlusPlusAPIVersion))
		return;

	// For more information on OP_CustomOPInfo see CPlusPlus_Common.h
	OP_CustomOPInfo& customInfo = info->customOPInfo;

	// Unique name of the node which starts with an upper case letter, followed by lower case letters or numbers
	customInfo.opType->setString("Opencascade");
	// English readable name
	customInfo.opLabel->setString("OpenCascade");
	// Information of the author of the node
	customInfo.authorName->setString("Valentin Siltsenko");
	customInfo.authorEmail->setString("gonzzza@gmail.com");

	customInfo.majorVersion = 0;
	customInfo.minorVersion = 1;

	// This SOP takes one input
	customInfo.minInputs = 1;
	customInfo.maxInputs = 1;
}

DLLEXPORT
SOP_CPlusPlusBase*
CreateSOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per SOP that is using the .dll
	return new OpenCascadeSOP(info);
}

DLLEXPORT
void
DestroySOPInstance(SOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the SOP using that instance is deleted, or
	// if the SOP loads a different DLL
	delete (OpenCascadeSOP*)instance;
}

};


OpenCascadeSOP::OpenCascadeSOP(const OP_NodeInfo*)
{
};

OpenCascadeSOP::~OpenCascadeSOP()
{
};

void
OpenCascadeSOP::getGeneralInfo(SOP_GeneralInfo* ginfo, const TD::OP_Inputs* inputs, void*)
{
	// Don't cook every frame: the node still cooks when the input SOP or a parameter changes
	ginfo->cookEveryFrameIfAsked = false;
	ginfo->directToGPU = false;

	// OpenCascade triangles are counter-clockwise around the outward normal.
	// Without this TouchDesigner assumes legacy clockwise winding and the
	// triangles face the opposite way of the output normals
	ginfo->winding = SOP_Winding::CCW;
}

void
OpenCascadeSOP::execute(SOP_Output* output, const TD::OP_Inputs* inputs, void*)
{
	const OP_SOPInput*	sop = inputs->getInputSOP(0);
	if (!sop) return;

	const CacheKey key{
		myParms.evalModifier(inputs),
		myParms.evalRadius(inputs),
		myParms.evalEdgeAngle(inputs),
		myParms.evalDeflection(inputs),
		myParms.evalAngularDeflection(inputs),
		myParms.evalNormals(inputs)
	};

	// Input polygons, flattened so they can be compared with the cached ones:
	// vertex count followed by the point indices, negative count for primitives
	// that are not closed polygons (they are skipped)
	std::vector<int32_t> prims;
	for (int32_t i = 0; i < sop->getNumPrimitives(); i++) {
		const SOP_PrimitiveInfo& prim = sop->getPrimitive(i);
		const bool polygon = prim.type == PrimitiveType::Polygon && prim.isClosed;
		prims.push_back(polygon ? prim.numVertices : -prim.numVertices);
		prims.insert(prims.end(), prim.pointIndices, prim.pointIndices + prim.numVertices);
	}

	const Position* inPos = sop->getPointPositions();
	const int numPoints = sop->getNumPoints();

	// Recompute only if the input or the parameters changed since the last cook
	const bool inputChanged = !myHasCache ||
		!(key == myCachedKey) ||
		prims != myCachedInputPrims ||
		myCachedInputPoints.size() != static_cast<size_t>(numPoints) ||
		(numPoints > 0 && std::memcmp(myCachedInputPoints.data(), inPos, numPoints * sizeof(Position)) != 0);

	if (inputChanged) {
		myCachedInputPoints.assign(inPos, inPos + numPoints);
		myCachedInputPrims = std::move(prims);
		myCachedKey = key;
		myHasCache = true;

		constexpr double degToRad = 3.14159265358979323846 / 180.0;
		const OcctSettings settings{
			key.modifier,
			key.radius,
			key.edgeAngle * degToRad,
			key.deflection,
			key.angularDeflection * degToRad,
			key.normals
		};

		OcctResult result;
		occtProcess(inPos, numPoints, myCachedInputPrims, settings, result);
		myCachedPoints = std::move(result.points);
		myCachedNormals = std::move(result.normals);
		myCachedIndices = std::move(result.indices);
		myCachedWarning = std::move(result.warning);
		myCachedError = std::move(result.error);
	}

	myWarningString = myCachedWarning;
	myErrorString = myCachedError;

	if (!myCachedPoints.empty()) {
		output->addPoints(myCachedPoints.data(), static_cast<int32_t>(myCachedPoints.size()));
		if (!myCachedNormals.empty())
			output->setNormals(myCachedNormals.data(), static_cast<int32_t>(myCachedNormals.size()), 0);
	}
	if (!myCachedIndices.empty()) {
		output->addTriangles(myCachedIndices.data(), static_cast<int32_t>(myCachedIndices.size() / 3));
	}
}

void
OpenCascadeSOP::executeVBO(SOP_VBOOutput* output, const TD::OP_Inputs* inputs, void*)
{
	// Not Called since ginfo->directToGPU is false
}

void
OpenCascadeSOP::setupParameters(TD::OP_ParameterManager* manager, void*)
{
	myParms.setup(manager);
}

void
OpenCascadeSOP::getErrorString(TD::OP_String* error, void*)
{
	error->setString(myErrorString.c_str());
	// Reset string after reporting it.
	myErrorString = "";
}

void
OpenCascadeSOP::getWarningString(OP_String* warning, void*)
{
	warning->setString(myWarningString.c_str());
	// Reset string after reporting it.
	myWarningString = "";
}
