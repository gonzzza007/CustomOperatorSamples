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

#ifndef __OpenCascadeSOP__
#define __OpenCascadeSOP__

#include "SOP_CPlusPlusBase.h"
#include "Parameters.h"
#include <string>
#include <vector>


/*
This SOP converts the input SOP polygons to an OpenCascade solid, applies
a Fillet or Chamfer to its sharp edges and outputs the triangulated result
*/

// To get more help about these functions, look at SOP_CPlusPlusBase.h
class OpenCascadeSOP : public TD::SOP_CPlusPlusBase
{
public:
	OpenCascadeSOP(const TD::OP_NodeInfo* info);
	virtual ~OpenCascadeSOP();

	virtual void		getGeneralInfo(TD::SOP_GeneralInfo*, const TD::OP_Inputs*, void*) override;

	virtual void		execute(TD::SOP_Output*, const TD::OP_Inputs*, void*) override;

	virtual void		executeVBO(TD::SOP_VBOOutput*, const TD::OP_Inputs*, void*) override;

	virtual void		setupParameters(TD::OP_ParameterManager* manager, void*) override;

	virtual void		getErrorString(TD::OP_String*, void*) override;

	virtual void		getWarningString(TD::OP_String*, void*) override;


private:
	std::string			myWarningString;
	std::string			myErrorString;

	Parameters myParms;

	// Cache of the last computation, re-emitted while input and parameters are unchanged
	struct CacheKey
	{
		ModifierMenuItems	modifier;
		double				radius;
		double				edgeAngle;
		double				deflection;
		double				angularDeflection;
		NormalsMenuItems	normals;

		bool operator==(const CacheKey& o) const
		{
			return modifier == o.modifier && radius == o.radius && edgeAngle == o.edgeAngle &&
				deflection == o.deflection && angularDeflection == o.angularDeflection &&
				normals == o.normals;
		}
	};

	bool						myHasCache = false;
	CacheKey					myCachedKey{};
	std::vector<TD::Position>	myCachedInputPoints;
	std::vector<int32_t>		myCachedInputPrims;		// per primitive: vertex count, then point indices
	std::vector<TD::Position>	myCachedPoints;
	std::vector<TD::Vector>		myCachedNormals;
	std::vector<int32_t>		myCachedIndices;
	std::string					myCachedWarning;
	std::string					myCachedError;
};

#endif // !__OpenCascadeSOP__
