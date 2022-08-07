#pragma once

class OP_Inputs;
class OP_ParameterManager;

#pragma region ParNames and ParLabels

// Names of the parameters

constexpr static char ShapeName[] = "Shape";
constexpr static char ShapeLabel[] = "Shape";

constexpr static char ColorName[] = "Color";
constexpr static char ColorLabel[] = "Color";

constexpr static char GpudirectName[] = "Gpudirect";
constexpr static char GpudirectLabel[] = "GPU Direct";

constexpr static char ScaleName[] = "Scale";
constexpr static char ScaleLabel[] = "Cell scale";

constexpr static char SpreadName[] = "Spread";
constexpr static char SpreadLabel[] = "Cell spread";

constexpr static char PointsChopName[] = "Inputpointschop";
constexpr static char PointsChopLabel[] = "Input points CHOP";


#pragma endregion

#pragma region Menus
enum class ShapeMenuItems
{
	Point,
	Line,
	Square,
	Cube,
	Divider,
	Voronoi,
	KDTree
};

#pragma endregion


#pragma region Parameters
class Parameters
{
public:
	static void		setup(TD::OP_ParameterManager*);

	// Shape
	static ShapeMenuItems		evalShape(const TD::OP_Inputs* input);

	// Color
	static TD::Color		evalColor(const TD::OP_Inputs* input);

	// GPU Direct
	static bool		evalGpudirect(const TD::OP_Inputs* input);

	// Cell scale
	static double	   evalScale(const TD::OP_Inputs* input);

	// Cell spread
	static double	   evalSpread(const TD::OP_Inputs* input);

	// Input points CHOP for Divider / Voronoi / KDTree
	static const TD::OP_CHOPInput* evalPointschop(const TD::OP_Inputs* input);


};
#pragma endregion