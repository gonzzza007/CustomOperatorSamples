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

#ifndef __SoftBodySOP__
#define __SoftBodySOP__

#include "SOP_CPlusPlusBase.h"
#include "ContainerSDF.h"
#include "Parameters.h"
#include "SoftBodySolver.h"

#include <cstdint>
#include <string>
#include <vector>

/*
Soft body blobs trapped inside a closed container.

Inputs:
	0 - Container: a closed mesh (e.g. a 1x1x1 Box SOP). Polygons are triangulated as fans.
	1 - Seeds: one point per blob (e.g. a Sprinkle SOP filling the container's volume).
	    Optional point attributes per seed: radius (or pscale), stiffness, mass.

Output:
	Triangulated blob meshes with point normals and an integer point attribute "blobid".
*/
class SoftBodySOP : public TD::SOP_CPlusPlusBase
{
public:
	SoftBodySOP(const TD::OP_NodeInfo* info);
	virtual ~SoftBodySOP();

	virtual void		getGeneralInfo(TD::SOP_GeneralInfo*, const TD::OP_Inputs*, void*) override;
	virtual void		execute(TD::SOP_Output*, const TD::OP_Inputs*, void*) override;
	virtual void		executeVBO(TD::SOP_VBOOutput*, const TD::OP_Inputs*, void*) override;
	virtual void		setupParameters(TD::OP_ParameterManager* manager, void*) override;
	virtual void		pulsePressed(const char* name, void*) override;

	virtual int32_t		getNumInfoCHOPChans(void*) override;
	virtual void		getInfoCHOPChan(int32_t index, TD::OP_InfoCHOPChan* chan, void*) override;
	virtual void		getWarningString(TD::OP_String*, void*) override;
	virtual void		getErrorString(TD::OP_String*, void*) override;

private:
	// Returns true if the container geometry (or resolution) changed and the SDF was rebuilt.
	bool		updateContainer(const TD::OP_SOPInput* sop, int resolution);

	// Reads seed points and per-seed overrides; returns a hash of everything read.
	uint64_t	readSeeds(const TD::OP_SOPInput* sop);

	void		writeOutput(TD::SOP_Output* output);

	sb::ContainerSDF				myContainer;
	sb::SoftBodySolver				mySolver;

	std::vector<sb::Vec3>			mySeeds;
	std::vector<sb::SeedOverrides>	myOverrides;
	std::vector<sb::Vec3>			myNormals;

	uint64_t		myContainerHash = 0;
	uint64_t		mySeedHash = 0;
	uint64_t		myGenHash = 0;
	bool			myResetRequested = true;

	// Info CHOP
	float			myStepMs = 0.0f;
	float			mySdfMs = 0.0f;
	sb::ResetReport	myLastReport;

	std::string		myWarningString;
	std::string		myErrorString;
};

#endif
