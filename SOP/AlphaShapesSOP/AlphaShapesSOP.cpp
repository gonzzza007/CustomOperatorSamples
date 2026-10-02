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

#include "AlphaShapesSOP.h"
#include "Parameters.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Fixed_alpha_shape_3.h>
#include <CGAL/Fixed_alpha_shape_vertex_base_3.h>
#include <CGAL/Fixed_alpha_shape_cell_base_3.h>

#include <cstring>
#include <unordered_map>
#include <vector>


typedef CGAL::Exact_predicates_inexact_constructions_kernel		Gt;
//typedef CGAL::Tag_true                                        Alpha_cmp_tag;

typedef CGAL::Alpha_shape_vertex_base_3<Gt>										Vb;
//typedef CGAL::Alpha_shape_vertex_base_3<Gt, CGAL::Default, Alpha_cmp_tag>   Vb;

typedef CGAL::Alpha_shape_cell_base_3<Gt>											Fb;
//typedef CGAL::Alpha_shape_cell_base_3<Gt, CGAL::Default, Alpha_cmp_tag>     Fb;

typedef CGAL::Triangulation_data_structure_3<Vb, Fb>					Tds;
typedef CGAL::Delaunay_triangulation_3<Gt, Tds>								Triangulation_3;

typedef CGAL::Alpha_shape_3<Triangulation_3> 									Alpha_shape_3;
//typedef CGAL::Alpha_shape_3<Triangulation_3, Alpha_cmp_tag> 	Alpha_shape_3;

typedef Gt::Point_3																						Point;
typedef Alpha_shape_3::Alpha_iterator													Alpha_iterator;

typedef Alpha_shape_3::Vertex_handle													Vertex_handle;
typedef Alpha_shape_3::Facet																	Facet;
typedef Alpha_shape_3::Cell_handle														Cell_handle;

// Fixed alpha shape: classifies faces for a single alpha value only (no alpha spectrum),
// much cheaper than Alpha_shape_3 when the optimal alpha search is not needed
typedef CGAL::Fixed_alpha_shape_vertex_base_3<Gt>							FVb;
typedef CGAL::Fixed_alpha_shape_cell_base_3<Gt>								FCb;
typedef CGAL::Triangulation_data_structure_3<FVb, FCb>				FTds;
typedef CGAL::Delaunay_triangulation_3<Gt, FTds>							FTriangulation_3;
typedef CGAL::Fixed_alpha_shape_3<FTriangulation_3>						Fixed_alpha_shape_3;

namespace
{

// Collects the geometry of an alpha shape (works for both Alpha_shape_3 and Fixed_alpha_shape_3):
//   Regularized: REGULAR facets (surface of the solid part)
//   General:     REGULAR + SINGULAR facets (dangling triangles),
//                SINGULAR edges as lines and SINGULAR vertices as isolated points
// Only points used by the output primitives are emitted, plus interior points
// unless skipInteriorPoints is set. Points outside the shape are never emitted.
template <class AS>
void
extractShape(const AS& as, bool general, bool skipInteriorPoints,
	std::vector<TD::Position>& points, std::vector<int32_t>& triIndices,
	std::vector<int32_t>& lineIndices)
{
	points.clear();
	triIndices.clear();
	lineIndices.clear();

	// Fewer than 4 points or coplanar input: no 3D alpha shape
	if (as.dimension() < 3)
		return;

	// Output point index of each emitted vertex, points are added on first use
	std::unordered_map<typename AS::Vertex_handle, int32_t> vertex_map;
	vertex_map.reserve(as.number_of_vertices());
	points.reserve(as.number_of_vertices());

	auto indexOf = [&](typename AS::Vertex_handle vh) {
		auto res = vertex_map.try_emplace(vh, static_cast<int32_t>(points.size()));
		if (res.second) {
			const auto& pt = vh->point();
			points.emplace_back(
				static_cast<float>(pt.x()),
				static_cast<float>(pt.y()),
				static_cast<float>(pt.z()));
		}
		return res.first->second;
	};

	// Facets (triangles)
	for (auto fit = as.finite_facets_begin(); fit != as.finite_facets_end(); ++fit) {
		auto type = as.classify(*fit);
		if (type == AS::REGULAR || (general && type == AS::SINGULAR)) {
			typename AS::Cell_handle cell = fit->first;
			int i = fit->second;

			triIndices.push_back(indexOf(cell->vertex((i + 1) & 3)));
			triIndices.push_back(indexOf(cell->vertex((i + 2) & 3)));
			triIndices.push_back(indexOf(cell->vertex((i + 3) & 3)));
		}
	}

	// Dangling edges (lines)
	if (general) {
		for (auto eit = as.finite_edges_begin(); eit != as.finite_edges_end(); ++eit) {
			if (as.classify(*eit) == AS::SINGULAR) {
				lineIndices.push_back(indexOf(eit->first->vertex(eit->second)));
				lineIndices.push_back(indexOf(eit->first->vertex(eit->third)));
			}
		}
	}

	// Isolated points (General) and interior points (unless skipped)
	if (general || !skipInteriorPoints) {
		for (auto vit = as.finite_vertices_begin(); vit != as.finite_vertices_end(); ++vit) {
			typename AS::Vertex_handle vh = vit;
			auto type = as.classify(vh);
			if ((general && type == AS::SINGULAR) ||
				(!skipInteriorPoints && type == AS::INTERIOR)) {
				indexOf(vh);
			}
		}
	}
}

}



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

	// Check to make sure the running TD version supports our API version.
	if (!info->setAPIVersion(SOPCPlusPlusAPIVersion))
		return;

	// For more information on OP_CustomOPInfo see CPlusPlus_Common.h
	OP_CustomOPInfo& customInfo = info->customOPInfo;

	// Unique name of the node which starts with an upper case letter, followed by lower case letters or numbers
	customInfo.opType->setString("Alphashapes");
	// English readable name
	customInfo.opLabel->setString("Alpha Shapes");
	// Information of the author of the node
	customInfo.authorName->setString("Valentin Siltsenko");
	customInfo.authorEmail->setString("gonzzza@gmail.com");

	customInfo.majorVersion = 0;
	customInfo.minorVersion = 5;

	// This CHOP takes one input
	customInfo.minInputs = 1;
	customInfo.maxInputs = 1;
}

DLLEXPORT
SOP_CPlusPlusBase*
CreateSOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new AlphaShapesSOP(info);
}

DLLEXPORT
void
DestroySOPInstance(SOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	delete (AlphaShapesSOP*)instance;
}

};


AlphaShapesSOP::AlphaShapesSOP(const OP_NodeInfo*)
{
};

AlphaShapesSOP::~AlphaShapesSOP()
{
};

void
AlphaShapesSOP::getGeneralInfo(SOP_GeneralInfo* ginfo, const TD::OP_Inputs* inputs, void*)
{
	// Don't cook every frame: the node still cooks when the input SOP or a parameter changes.
	// Cooking every frame would rebuild the output geometry each frame even for a static input
	ginfo->cookEveryFrameIfAsked = false;

	// Don't know what to do with it but TRUE does not work...
	ginfo->directToGPU = false;

}

void
AlphaShapesSOP::execute(SOP_Output* output, const TD::OP_Inputs* inputs, void*)
{
	const OP_SOPInput*	sop = inputs->getInputSOP(0);
	if (!sop) return;

	ModeMenuItems mode = myParms.evalMode(inputs);
	bool useOptimalAlpha = myParms.evalUseOptimalAlpha(inputs);
	bool skipInteriorPoints = myParms.evalSkipInteriorPoints(inputs);
	double alpha = myParms.evalAlpha(inputs);

	bool general = mode == ModeMenuItems::General;

	// Alpha is ignored when searching for the optimal one, so it doesn't invalidate the cache
	CacheKey key{
		mode,
		useOptimalAlpha,
		skipInteriorPoints,
		useOptimalAlpha ? 0.0 : alpha
	};

	const Position* inPos = sop->getPointPositions();
	const int numPoints = sop->getNumPoints();

	// Recompute only if input points or parameters changed since the last cook
	bool inputChanged = !myHasCache ||
		!(key == myCachedKey) ||
		myCachedInput.size() != static_cast<size_t>(numPoints) ||
		(numPoints > 0 && std::memcmp(myCachedInput.data(), inPos, numPoints * sizeof(Position)) != 0);

	if (inputChanged) {
		myCachedInput.assign(inPos, inPos + numPoints);
		myCachedKey = key;
		myCachedWarning.clear();

		std::vector<Point> lp;
		lp.reserve(numPoints);
		for (int i = 0; i < numPoints; ++i) {
			lp.emplace_back(inPos[i].x, inPos[i].y, inPos[i].z);
		}

		if (useOptimalAlpha) {
			// Full alpha spectrum is needed to search for the optimal alpha value.
			// Mode is passed to the constructor: calling set_mode() afterwards
			// re-initializes the alpha maps and re-sorts the alpha spectrum
			Alpha_shape_3 as(lp.begin(), lp.end(), 0,
				general ? Alpha_shape_3::GENERAL : Alpha_shape_3::REGULARIZED);

			if (as.dimension() == 3) {
				Alpha_shape_3::NT alpha_solid = as.find_alpha_solid();
				Alpha_shape_3::Alpha_iterator opt = as.find_optimal_alpha(1);
				if (opt != as.alpha_end()) {
					as.set_alpha(*opt);

					std::stringstream buffer;
					buffer << std::endl << "Smallest alpha value to get a solid through data points is " << alpha_solid << std::endl;
					buffer << "Optimal alpha value to get one connected component is " << *opt << std::endl;
					myCachedWarning = buffer.str();
				}
			}

			extractShape(as, general, skipInteriorPoints,
				myCachedPoints, myCachedIndices, myCachedLineIndices);
		} else {
			// Single alpha value: no need for the alpha spectrum.
			// Fixed_alpha_shape_3 always classifies like GENERAL mode,
			// extractShape() drops SINGULAR faces for Regularized
			Fixed_alpha_shape_3 as(lp.begin(), lp.end(), alpha);
			extractShape(as, general, skipInteriorPoints,
				myCachedPoints, myCachedIndices, myCachedLineIndices);
		}

		myCachedLineSizes.assign(myCachedLineIndices.size() / 2, 2);

		myHasCache = true;
	}

	myWarningString = myCachedWarning;

	if (!myCachedPoints.empty()) {
		output->addPoints(myCachedPoints.data(), static_cast<int32_t>(myCachedPoints.size()));
	}
	if (!myCachedIndices.empty()) {
		output->addTriangles(myCachedIndices.data(), static_cast<int32_t>(myCachedIndices.size() / 3));
	}
	if (!myCachedLineSizes.empty()) {
		output->addLines(myCachedLineIndices.data(), myCachedLineSizes.data(),
			static_cast<int32_t>(myCachedLineSizes.size()));
	}
}

void
AlphaShapesSOP::executeVBO(SOP_VBOOutput* output, const TD::OP_Inputs* inputs, void*)
{
	// Not Called since ginfo->directToGPU is false
}

void
AlphaShapesSOP::setupParameters(TD::OP_ParameterManager* manager, void*)
{
	myParms.setup(manager);
}

void
AlphaShapesSOP::getErrorString(TD::OP_String* error, void*)
{
	error->setString(myErrorString.c_str());
	// Reset string after reporting it.
	myErrorString = "";
}

void
AlphaShapesSOP::getWarningString(OP_String* warning, void*)
{
	warning->setString(myWarningString.c_str());
	// Reset string after reporting it.
	myWarningString = "";
}
