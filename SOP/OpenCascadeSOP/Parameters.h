#pragma once

namespace TD
{
	class OP_Inputs;
	class OP_ParameterManager;
}

#pragma region ParNames and ParLabels

// Names of the parameters
constexpr static char ModifierName[] = "Modifier";
constexpr static char ModifierLabel[] = "Modifier";

constexpr static char RadiusName[] = "Radius";
constexpr static char RadiusLabel[] = "Radius / Distance";

constexpr static char EdgeAngleName[] = "Edgeangle";
constexpr static char EdgeAngleLabel[] = "Min Edge Angle";

constexpr static char DeflectionName[] = "Deflection";
constexpr static char DeflectionLabel[] = "Mesh Deflection";

constexpr static char AngularDeflectionName[] = "Angulardeflection";
constexpr static char AngularDeflectionLabel[] = "Mesh Angular Deflection";

constexpr static char NormalsName[] = "Normals";
constexpr static char NormalsLabel[] = "Normals";

#pragma endregion

#pragma region Menus
enum class ModifierMenuItems
{
	Fillet, // default
	Chamfer,
	Planecut	// chamfer by cutting convex parts with bevel planes
};

enum class NormalsMenuItems
{
	Off,
	Mesh,	// default, computed from the output triangles
	Surface	// exact normals of the OpenCascade surfaces
};
#pragma endregion

#pragma region Parameters
class Parameters
{
public:
	static void		setup(TD::OP_ParameterManager*);

	// Fillet (BRepFilletAPI_MakeFillet), Chamfer (BRepFilletAPI_MakeChamfer)
	// or Plane Cut (chamfer by cutting convex parts with bevel planes)
	static ModifierMenuItems	evalModifier(const TD::OP_Inputs* input);

	// fillet radius or chamfer distance
	static double	evalRadius(const TD::OP_Inputs* input);

	// only edges whose faces meet at a larger angle (degrees) are modified
	static double	evalEdgeAngle(const TD::OP_Inputs* input);

	// max distance between the triangulation and the real surface
	static double	evalDeflection(const TD::OP_Inputs* input);

	// max angle (degrees) between neighbouring triangles on curved surfaces
	static double	evalAngularDeflection(const TD::OP_Inputs* input);

	// how the point normals are computed
	static NormalsMenuItems	evalNormals(const TD::OP_Inputs* input);
};
#pragma endregion
