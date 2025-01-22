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

// #include <fstream>
// #include <list>
#include <cassert>


typedef CGAL::Exact_predicates_inexact_constructions_kernel		Gt;
typedef CGAL::Alpha_shape_vertex_base_3<Gt>										Vb;
typedef CGAL::Alpha_shape_cell_base_3<Gt>											Fb;
typedef CGAL::Triangulation_data_structure_3<Vb, Fb>					Tds;
typedef CGAL::Delaunay_triangulation_3<Gt, Tds>								Triangulation_3;
typedef CGAL::Alpha_shape_3<Triangulation_3>									Alpha_shape_3;
typedef Gt::Point_3																						Point;
typedef Alpha_shape_3::Alpha_iterator													Alpha_iterator;

typedef Alpha_shape_3::Vertex_handle													Vertex_handle;
typedef Alpha_shape_3::Facet																	Facet;
typedef Alpha_shape_3::Cell_handle														Cell_handle;



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
	customInfo.opType->setString("Alphashapes");
	// English readable name
	customInfo.opLabel->setString("Alpha Shapes");
	// Information of the author of the node
	customInfo.authorName->setString("Valentin Siltsenko");
	customInfo.authorEmail->setString("gonzzza@gmail.com");

	customInfo.majorVersion = 0;
	customInfo.minorVersion = 3;

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
	// This will cause the node to cook every frame if the output is used
	// We set it to true otherwise the sop does not update when input sop changes
	ginfo->cookEveryFrameIfAsked = true;

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


	std::list<Point> lp;
	const Position* inPos = sop->getPointPositions();
	for (int i = 0; i < sop->getNumPoints(); ++i) {
		lp.emplace_back(inPos[i].x, inPos[i].y, inPos[i].z);
	}

	// Alpha shape computed in REGULARIZED mode by default
	Alpha_shape_3 as(lp.begin(), lp.end());
	if (mode == ModeMenuItems::General) { 
		as.set_mode(Alpha_shape_3::GENERAL);
	}
	
	// search for optimal alpha value?
	if (useOptimalAlpha) {	
		Alpha_shape_3::NT alpha_solid = as.find_alpha_solid();
		Alpha_shape_3::Alpha_iterator opt = as.find_optimal_alpha(1);
		as.set_alpha(*opt);
		
		std::stringstream buffer;
		buffer << std::endl << "Smallest alpha value to get a solid through data points is " << alpha_solid << std::endl;
		buffer << "Optimal alpha value to get one connected component is " << *opt;
		myWarningString = buffer.str();
	
	} else {
		as.set_alpha(alpha);
	}
	
	std::unordered_map<Alpha_shape_3::Vertex_handle, size_t> vertex_map;
	size_t idx = 0;
	for (auto vit = as.vertices_begin(); vit != as.vertices_end(); ++vit) {
		
		// Skip interior points/vertices?
		if (skipInteriorPoints && as.classify(vit) == Alpha_shape_3::INTERIOR)
			continue;
		
		Point pt = vit->point();
		output->addPoint(TD::Position(
			pt.x(),
			pt.y(),
			pt.z()
		));
		
		// remember point indexes
		vertex_map[vit] = idx++;
	}


	// Iterate through all facets (triangles)
	std::vector<std::array<Point, 3>> facets;
	for (Alpha_shape_3::Facet_iterator fit = as.facets_begin();
		fit != as.facets_end(); ++fit) {
		if (as.classify(*fit) == Alpha_shape_3::REGULAR) {
			Alpha_shape_3::Cell_handle cell = fit->first;
			int i = fit->second;

			output->addTriangle(
				vertex_map[cell->vertex((i + 1) & 3)],
				vertex_map[cell->vertex((i + 2) & 3)],
				vertex_map[cell->vertex((i + 3) & 3)]
			);

		}
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
