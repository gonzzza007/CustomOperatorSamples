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

// Parameters of the Soft Body SOP, in the layout used by the cppParsTemplateGen palette component.

#pragma once

#include "SoftBodySolver.h"

namespace TD
{
	class OP_Inputs;
	class OP_ParameterManager;
}

#pragma region ParNames and ParLabels

// Page "Simulation"
constexpr static char SimulateName[] = "Simulate";
constexpr static char SimulateLabel[] = "Simulate";

constexpr static char ResetName[] = "Reset";
constexpr static char ResetLabel[] = "Reset";

constexpr static char StiffnessName[] = "Stiffness";
constexpr static char StiffnessLabel[] = "Stiffness";

constexpr static char BendingName[] = "Bending";
constexpr static char BendingLabel[] = "Bending";

constexpr static char VolumestiffnessName[] = "Volumestiffness";
constexpr static char VolumestiffnessLabel[] = "Volume Stiffness";

constexpr static char PressureName[] = "Pressure";
constexpr static char PressureLabel[] = "Pressure";

constexpr static char FillName[] = "Fill";
constexpr static char FillLabel[] = "Fill";

constexpr static char GrowspeedName[] = "Growspeed";
constexpr static char GrowspeedLabel[] = "Grow Speed";

constexpr static char ShapememoryName[] = "Shapememory";
constexpr static char ShapememoryLabel[] = "Shape Memory";

constexpr static char DampingName[] = "Damping";
constexpr static char DampingLabel[] = "Damping";

constexpr static char FrictionName[] = "Friction";
constexpr static char FrictionLabel[] = "Friction";

constexpr static char RestitutionName[] = "Restitution";
constexpr static char RestitutionLabel[] = "Restitution";

constexpr static char GravityName[] = "Gravity";
constexpr static char GravityLabel[] = "Gravity";

constexpr static char SubstepsName[] = "Substeps";
constexpr static char SubstepsLabel[] = "Substeps";

constexpr static char IterationsName[] = "Iterations";
constexpr static char IterationsLabel[] = "Iterations";

// Page "Blobs"
constexpr static char RadiusminName[] = "Radiusmin";
constexpr static char RadiusminLabel[] = "Radius Min";

constexpr static char RadiusmaxName[] = "Radiusmax";
constexpr static char RadiusmaxLabel[] = "Radius Max";

constexpr static char SeedName[] = "Seed";
constexpr static char SeedLabel[] = "Seed";

constexpr static char EdgelengthName[] = "Edgelength";
constexpr static char EdgelengthLabel[] = "Edge Length";

constexpr static char MinpointsName[] = "Minpoints";
constexpr static char MinpointsLabel[] = "Min Points";

constexpr static char MaxpointsName[] = "Maxpoints";
constexpr static char MaxpointsLabel[] = "Max Points";

constexpr static char MassmodeName[] = "Massmode";
constexpr static char MassmodeLabel[] = "Mass Mode";

constexpr static char MassName[] = "Mass";
constexpr static char MassLabel[] = "Mass";

constexpr static char CollisionradiusName[] = "Collisionradius";
constexpr static char CollisionradiusLabel[] = "Collision Radius";

constexpr static char SurfaceoffsetName[] = "Surfaceoffset";
constexpr static char SurfaceoffsetLabel[] = "Surface Offset";

constexpr static char SdfresolutionName[] = "Sdfresolution";
constexpr static char SdfresolutionLabel[] = "Container Resolution";

#pragma endregion

#pragma region Menus

enum class MassmodeMenuItems
{
	Density,
	Perblob
};

#pragma endregion

#pragma region Parameters

class Parameters
{
public:
	static void		setup(TD::OP_ParameterManager*);

	static bool		evalSimulate(const TD::OP_Inputs* inputs);
	static bool		evalSurfaceoffset(const TD::OP_Inputs* inputs);

	// Parameters that may change every frame.
	static sb::SimSettings			evalSimSettings(const TD::OP_Inputs* inputs);

	// Parameters that require regenerating the blobs.
	static sb::GenerationSettings	evalGenerationSettings(const TD::OP_Inputs* inputs);

	static int		evalSdfresolution(const TD::OP_Inputs* inputs);
};

#pragma endregion
