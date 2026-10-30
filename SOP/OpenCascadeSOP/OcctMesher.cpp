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

#include "OcctMesher.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom_Plane.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Solid.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <sstream>
#include <vector>


namespace
{

// Points shared by polygons are identical, so sewing only needs a tiny tolerance
constexpr double theSewingTolerance = 1e-6;

// Tolerances for merging coplanar faces
constexpr double theUnifyLinearTolerance = 1e-5;
constexpr double theUnifyAngularTolerance = 1e-4;	// radians

// Plane Cut: faces meeting at a smaller angle (radians) count as one flat surface
constexpr double theFlatAngle = 0.5 * 3.14159265358979323846 / 180.0;

// Plane Cut: bevels fade in over this angle (radians) above the Min Edge Angle
constexpr double theAngleFade = 5.0 * 3.14159265358979323846 / 180.0;

constexpr char theInvalidInput[] = "invalid input geometry";

TopoDS_Face
makePolygonFace(const std::vector<gp_Pnt>& pts)
{
	BRepBuilderAPI_MakePolygon poly;
	for (const gp_Pnt& p : pts)
		poly.Add(p);
	poly.Close();
	if (!poly.IsDone())
		return TopoDS_Face();

	// OnlyPlane: non planar polygons fail here and are triangulated by the caller
	BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
	return face.IsDone() ? face.Face() : TopoDS_Face();
}

// Union-find over point indices, used to group polygons into connected pieces
class PointSets
{
public:
	explicit PointSets(int32_t numPoints) : myParent(numPoints)
	{
		for (int32_t i = 0; i < numPoints; i++)
			myParent[i] = i;
	}

	int32_t
	find(int32_t i)
	{
		while (myParent[i] != i) {
			myParent[i] = myParent[myParent[i]];
			i = myParent[i];
		}
		return i;
	}

	void
	join(int32_t a, int32_t b)
	{
		myParent[find(a)] = find(b);
	}

private:
	std::vector<int32_t> myParent;
};

// Converts the input polygons to OpenCascade faces and sews them together.
// Polygons that don't share points (e.g. separate Voronoi or KDTree cells) are
// sewn separately, so touching pieces are never sewn into one non-manifold shell.
// Groups that are not closed (e.g. a Box SOP, where every face has its own
// corner points) are welded by point position, so they can still form a solid.
// Returns the number of polygons that could not be converted.
int
buildSewedPieces(const TD::Position* pos, int32_t numPoints, const std::vector<int32_t>& polygons,
	std::vector<TopoDS_Shape>& pieces)
{
	pieces.clear();
	int skipped = 0;

	// first pass: valid polygons (offset of their first index and vertex count)
	std::vector<std::pair<size_t, int32_t>> valid;
	PointSets sets(numPoints);
	for (size_t i = 0; i < polygons.size(); ) {
		const int32_t count = polygons[i++];
		const size_t first = i;
		i += static_cast<size_t>(std::abs(count));

		bool ok = count >= 3 && i <= polygons.size();
		for (size_t v = first; ok && v < i; v++)
			ok = polygons[v] >= 0 && polygons[v] < numPoints;
		if (!ok) {
			skipped++;
			continue;
		}

		for (size_t v = first + 1; v < i; v++)
			sets.join(polygons[first], polygons[v]);
		valid.emplace_back(first, count);
	}

	// A group is closed if every edge (by point index) is used by exactly two
	// polygons. Points of the open groups are welded by position: a box with
	// unshared corners becomes one closed group, while closed cells that only
	// touch each other stay separate.
	{
		std::map<std::pair<int32_t, int32_t>, int> edgeUse;
		for (const auto& [first, count] : valid) {
			for (int32_t v = 0; v < count; v++) {
				int32_t a = polygons[first + v];
				int32_t b = polygons[first + (v + 1) % count];
				if (a > b)
					std::swap(a, b);
				edgeUse[{ a, b }]++;
			}
		}
		std::vector<char> groupOpen(numPoints, 0);
		for (const auto& [edge, uses] : edgeUse) {
			if (uses != 2)
				groupOpen[sets.find(edge.first)] = 1;
		}

		// adding 0.0f turns -0 into +0 so both weld together
		std::map<std::array<float, 3>, int32_t> welded;
		std::vector<int32_t> openPoints;
		for (const auto& [first, count] : valid) {
			if (!groupOpen[sets.find(polygons[first])])
				continue;
			for (int32_t v = 0; v < count; v++)
				openPoints.push_back(polygons[first + v]);
		}
		for (int32_t p : openPoints) {
			const std::array<float, 3> key = { pos[p].x + 0.0f, pos[p].y + 0.0f, pos[p].z + 0.0f };
			const auto it = welded.emplace(key, p).first;
			if (it->second != p)
				sets.join(p, it->second);
		}
	}

	// group polygons by piece, in input order
	std::vector<int32_t> pieceOfRoot(numPoints, -1);
	std::vector<std::vector<size_t>> pieceFaces;
	for (size_t k = 0; k < valid.size(); k++) {
		const int32_t root = sets.find(polygons[valid[k].first]);
		if (pieceOfRoot[root] < 0) {
			pieceOfRoot[root] = static_cast<int32_t>(pieceFaces.size());
			pieceFaces.emplace_back();
		}
		pieceFaces[pieceOfRoot[root]].push_back(k);
	}

	std::vector<gp_Pnt> pts;
	for (const std::vector<size_t>& faces : pieceFaces) {
		BRepBuilderAPI_Sewing sewing(theSewingTolerance);
		int added = 0;

		for (size_t k : faces) {
			const size_t first = valid[k].first;
			const int32_t count = valid[k].second;
			pts.clear();
			for (int32_t v = 0; v < count; v++) {
				const TD::Position& p = pos[polygons[first + v]];
				pts.emplace_back(p.x, p.y, p.z);
			}

			TopoDS_Face face = makePolygonFace(pts);
			if (!face.IsNull()) {
				sewing.Add(face);
				added++;
				continue;
			}

			// non planar or degenerate polygon: fall back to a triangle fan
			bool any = false;
			for (size_t v = 1; v + 1 < pts.size(); v++) {
				TopoDS_Face tri = makePolygonFace({ pts[0], pts[v], pts[v + 1] });
				if (!tri.IsNull()) {
					sewing.Add(tri);
					added++;
					any = true;
				}
			}
			if (!any)
				skipped++;
		}

		if (added > 0) {
			sewing.Perform();
			pieces.push_back(sewing.SewedShape());
		}
	}
	return skipped;
}

// Splits the sewn shape into independent parts: every closed shell becomes a
// solid, open shells and loose faces are kept as they are. Coplanar faces of
// each part are merged so that the modifier only sees the real edges.
std::vector<TopoDS_Shape>
makeParts(const TopoDS_Shape& sewed, int& numSolids)
{
	std::vector<TopoDS_Shape> parts;

	auto unify = [](const TopoDS_Shape& shape) {
		ShapeUpgrade_UnifySameDomain unifier(shape, Standard_True, Standard_True, Standard_False);
		// the input comes as floats, so coplanar triangles are only coplanar
		// within float precision
		unifier.SetLinearTolerance(theUnifyLinearTolerance);
		unifier.SetAngularTolerance(theUnifyAngularTolerance);
		unifier.Build();

		// merged faces keep the float noise of their vertices: let ShapeFix raise
		// the tolerances so the shape stays valid for the modifiers
		ShapeFix_Shape fix(unifier.Shape());
		fix.Perform();
		return fix.Shape();
	};

	for (TopExp_Explorer e(sewed, TopAbs_SHELL); e.More(); e.Next()) {
		const TopoDS_Shell& shell = TopoDS::Shell(e.Current());
		if (BRep_Tool::IsClosed(shell)) {
			BRepBuilderAPI_MakeSolid makeSolid(shell);
			if (makeSolid.IsDone()) {
				// orient the solid so that its faces point outwards
				ShapeFix_Solid fix(makeSolid.Solid());
				fix.Perform();
				parts.push_back(unify(fix.Solid()));
				numSolids++;
				continue;
			}
		}
		parts.push_back(unify(shell));
	}

	for (TopExp_Explorer e(sewed, TopAbs_FACE, TopAbs_SHELL); e.More(); e.Next())
		parts.push_back(e.Current());

	return parts;
}

bool
planeNormal(const TopoDS_Face& face, gp_Dir& normal)
{
	Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(BRep_Tool::Surface(face));
	if (plane.IsNull())
		return false;

	normal = plane->Pln().Axis().Direction();
	if (face.Orientation() == TopAbs_REVERSED)
		normal.Reverse();
	return true;
}

// Edges shared by two faces meeting at an angle of at least minAngle (radians).
// Edges on curved (non planar) faces are always included.
// forward declaration (defined below)
static double edgeLength(const TopoDS_Edge& edge);

std::vector<TopoDS_Edge>
sharpEdges(const TopoDS_Shape& part, double minAngle, double minEdgeLen = 0.0)
{
	TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
	TopExp::MapShapesAndAncestors(part, TopAbs_EDGE, TopAbs_FACE, edgeFaces);

	std::vector<TopoDS_Edge> edges;
	for (int i = 1; i <= edgeFaces.Extent(); i++) {
		const TopoDS_Edge& edge = TopoDS::Edge(edgeFaces.FindKey(i));
		const TopTools_ListOfShape& faces = edgeFaces.FindFromIndex(i);

		// border edges of open shells can't be filleted
		if (faces.Extent() != 2 || BRep_Tool::Degenerated(edge))
			continue;

		// skip degenerate micro-edges that would make BRepCheck_Analyzer fail
		// and cause the chamfer to be skipped for the whole part
		if (minEdgeLen > 0.0 && edgeLength(edge) < minEdgeLen)
			continue;

		gp_Dir n1, n2;
		if (planeNormal(TopoDS::Face(faces.First()), n1) &&
			planeNormal(TopoDS::Face(faces.Last()), n2) &&
			n1.Angle(n2) < minAngle)
			continue;

		edges.push_back(edge);
	}
	return edges;
}

double
edgeLength(const TopoDS_Edge& edge)
{
	try {
		return GCPnts_AbscissaPoint::Length(BRepAdaptor_Curve(edge));
	} catch (const Standard_Failure&) {
		return std::numeric_limits<double>::infinity();
	}
}

// Builds the modifier on the given edges of the part into result.
// Returns false (and leaves result untouched) if OpenCascade fails.
bool
buildModifier(ModifierMenuItems modifier, double radius, const TopoDS_Shape& part,
	const std::vector<TopoDS_Edge>& edges, TopoDS_Shape& result, std::string& failure)
{
	try {
		if (modifier == ModifierMenuItems::Fillet) {
			BRepFilletAPI_MakeFillet fillet(part);
			for (const TopoDS_Edge& edge : edges)
				fillet.Add(radius, edge);
			fillet.Build();
			if (!fillet.IsDone()) {
				failure = "OpenCascade could not build the fillet";
				return false;
			}
			if (!BRepCheck_Analyzer(fillet.Shape()).IsValid()) {
				failure = "fillet produced invalid geometry";
				return false;
			}
			result = fillet.Shape();
		} else {
			BRepFilletAPI_MakeChamfer chamfer(part);
			for (const TopoDS_Edge& edge : edges)
				chamfer.Add(radius, edge);
			chamfer.Build();
			if (!chamfer.IsDone()) {
				failure = "OpenCascade could not build the chamfer";
				return false;
			}
			if (!BRepCheck_Analyzer(chamfer.Shape()).IsValid()) {
				failure = "chamfer produced invalid geometry";
				return false;
			}
			result = chamfer.Shape();
		}
	} catch (const Standard_Failure& e) {
		failure = e.GetMessageString();
		return false;
	} catch (const std::exception& e) {
		failure = e.what();
		return false;
	}
	return true;
}

// Applies the modifier to all sharp edges of the part.
// Returns false (and leaves the part untouched) if OpenCascade fails.
// shortestEdge receives the length of the shortest modified edge: a fillet or
// chamfer fails once the radius / distance reaches half of it, because the
// modified strips on both ends of that edge would overlap.
bool
applyModifier(ModifierMenuItems modifier, double radius, double minAngle,
	TopoDS_Shape& part, std::string& failure, double& shortestEdge)
{
	// minEdgeLen: skip edges shorter than 2*radius — they would fail the chamfer
	// anyway, and degenerate slivers (e.g. from SoftBody meshing) that are far
	// shorter cause BRepCheck_Analyzer to reject the whole part.
	const double minEdgeLen = 2.0 * radius * (1.0 - 1e-6);
	std::vector<TopoDS_Edge> edges = sharpEdges(part, minAngle, minEdgeLen);
	shortestEdge = std::numeric_limits<double>::infinity();
	if (edges.empty())
		return true;
	for (const TopoDS_Edge& edge : edges)
		shortestEdge = std::min(shortestEdge, edgeLength(edge));

	// the fillet algorithms don't check their input and can crash on broken shapes
	// Run ShapeFix first: SoftBody geometry has float-precision noise that makes
	// BRepCheck_Analyzer reject otherwise-valid shapes, skipping the chamfer entirely.
	{
		ShapeFix_Shape fix(part);
		fix.Perform();
		part = fix.Shape();
	}
	if (!BRepCheck_Analyzer(part).IsValid()) {
		failure = theInvalidInput;
		return false;
	}

	TopoDS_Shape result;
	if (!buildModifier(modifier, radius, part, edges, result, failure))
		return false;
	part = result;
	return true;
}

// ---------------------------------------------------------------------------
// Plane cut chamfer.
// Every convex edge gets one bevel plane, tilted halfway between its two faces.
// Concave edges and edges under the Min Edge Angle stay untouched.
//
// The result must change continuously with the distance, so:
// - Every edge stops by itself at its limit: half the width of its faces, and
//   halfway from the edge to the center of the part. Past the limit the shape
//   no longer changes, and a convex part can't be cut away.
// - The cut works on one welded triangle mesh. Every point is classified once
//   and neighbouring triangles share the new points, so the mesh stays closed.
//   No solid is rebuilt, so there is no validity check that can reject a result.
// - A plane removes only the piece above it that is nearest to its edge, and the
//   points of concave edges can't be removed. A bevel that reaches a concave
//   edge narrows to its end point instead of cutting into the next region.

struct CutMesh
{
	std::vector<gp_XYZ>					points;
	std::vector<std::array<int32_t, 3>>	triangles;		// counter-clockwise from outside
	std::vector<int32_t>				triangleFaces;	// face of every triangle
	std::vector<gp_XYZ>					faceNormals;	// outward
	std::vector<char>					locked;			// points on concave edges, the new points are never locked
};

// keeps the side n.x <= c
struct Bevel
{
	gp_XYZ	n;
	double	c;
	gp_XYZ	a, b;		// the edge
	double	distance;	// after the limits of the edge
};

// Triangulates the solid and welds the points of its faces.
// Returns false if a face has no triangles or is not planar.
bool
buildCutMesh(const TopoDS_Shape& solid, double deflection, double angularDeflection, CutMesh& mesh, double& size)
{
	BRepMesh_IncrementalMesh mesher(solid, deflection, Standard_False, angularDeflection, Standard_True);

	struct RawFace
	{
		Handle(Poly_Triangulation)	tri;
		gp_Trsf						trsf;
		bool						reversed;
	};
	std::vector<RawFace> faces;
	gp_XYZ lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
	for (TopExp_Explorer e(solid, TopAbs_FACE); e.More(); e.Next()) {
		const TopoDS_Face& face = TopoDS::Face(e.Current());
		gp_Dir normal;
		TopLoc_Location loc;
		Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull() || !planeNormal(face, normal))
			return false;
		faces.push_back({ tri, loc.Transformation(), face.Orientation() == TopAbs_REVERSED });
		mesh.faceNormals.push_back(normal.XYZ());
		for (int n = 1; n <= tri->NbNodes(); n++) {
			const gp_XYZ p = tri->Node(n).Transformed(faces.back().trsf).XYZ();
			lo.SetCoord(std::min(lo.X(), p.X()), std::min(lo.Y(), p.Y()), std::min(lo.Z(), p.Z()));
			hi.SetCoord(std::max(hi.X(), p.X()), std::max(hi.Y(), p.Y()), std::max(hi.Z(), p.Z()));
		}
	}
	if (faces.empty())
		return false;
	size = (hi - lo).Modulus();

	// weld the points by position with a grid of cells the size of the tolerance
	const double tol = std::max(10.0 * theSewingTolerance, 1e-7 * size);
	std::map<std::array<int64_t, 3>, std::vector<int32_t>> grid;
	auto weld = [&](const gp_XYZ& p) {
		const std::array<int64_t, 3> cell = {
			static_cast<int64_t>(std::floor(p.X() / tol)),
			static_cast<int64_t>(std::floor(p.Y() / tol)),
			static_cast<int64_t>(std::floor(p.Z() / tol))
		};
		for (int64_t x = -1; x <= 1; x++) {
			for (int64_t y = -1; y <= 1; y++) {
				for (int64_t z = -1; z <= 1; z++) {
					const auto it = grid.find({ cell[0] + x, cell[1] + y, cell[2] + z });
					if (it == grid.end())
						continue;
					for (int32_t i : it->second) {
						if ((mesh.points[i] - p).Modulus() <= tol)
							return i;
					}
				}
			}
		}
		const int32_t index = static_cast<int32_t>(mesh.points.size());
		mesh.points.push_back(p);
		grid[cell].push_back(index);
		return index;
	};

	std::vector<int32_t> nodes;
	for (size_t f = 0; f < faces.size(); f++) {
		const RawFace& face = faces[f];
		nodes.clear();
		for (int n = 1; n <= face.tri->NbNodes(); n++)
			nodes.push_back(weld(face.tri->Node(n).Transformed(face.trsf).XYZ()));

		for (int t = 1; t <= face.tri->NbTriangles(); t++) {
			int a, b, c;
			face.tri->Triangle(t).Get(a, b, c);
			if (face.reversed)
				std::swap(b, c);
			const std::array<int32_t, 3> v = { nodes[a - 1], nodes[b - 1], nodes[c - 1] };
			if (v[0] == v[1] || v[1] == v[2] || v[2] == v[0])
				continue;
			mesh.triangles.push_back(v);
			mesh.triangleFaces.push_back(static_cast<int32_t>(f));
		}
	}
	return !mesh.triangles.empty();
}

// Bevels of the convex edges between two faces meeting at minAngle or more
std::vector<Bevel>
makeBevels(CutMesh& mesh, double distance, double minAngle)
{
	// center of the volume: unlike the average of the points it doesn't jump when
	// the triangulation changes
	gp_XYZ center(0, 0, 0);
	{
		gp_XYZ origin(0, 0, 0);
		for (const gp_XYZ& p : mesh.points)
			origin += p;
		origin /= static_cast<double>(mesh.points.size());
		gp_XYZ moment(0, 0, 0);
		double volume = 0.0;
		for (const std::array<int32_t, 3>& v : mesh.triangles) {
			const gp_XYZ p0 = mesh.points[v[0]] - origin;
			const gp_XYZ p1 = mesh.points[v[1]] - origin;
			const gp_XYZ p2 = mesh.points[v[2]] - origin;
			const double tetra = p0.Dot(p1.Crossed(p2)) / 6.0;
			volume += tetra;
			moment += (p0 + p1 + p2) * (tetra / 4.0);
		}
		center = std::abs(volume) > 1e-12 ? origin + moment / volume : origin;
	}

	mesh.locked.assign(mesh.points.size(), 0);
	std::vector<std::vector<int32_t>> facePoints(mesh.faceNormals.size());
	struct EdgeUse
	{
		int32_t	face;
		int32_t	from;	// the direction of the edge in this triangle
	};
	std::map<std::pair<int32_t, int32_t>, std::vector<EdgeUse>> edges;
	for (size_t t = 0; t < mesh.triangles.size(); t++) {
		const std::array<int32_t, 3>& v = mesh.triangles[t];
		const int32_t face = mesh.triangleFaces[t];
		for (int k = 0; k < 3; k++) {
			facePoints[face].push_back(v[k]);
			const int32_t from = v[k], to = v[(k + 1) % 3];
			edges[{ std::min(from, to), std::max(from, to) }].push_back({ face, from });
		}
	}

	auto dihedral = [&](int32_t f1, int32_t f2) {
		const gp_XYZ& n1 = mesh.faceNormals[f1];
		const gp_XYZ& n2 = mesh.faceNormals[f2];
		return std::atan2(n1.Crossed(n2).Modulus(), n1.Dot(n2));
	};

	// Faces that meet at a nearly flat edge are one surface. The input can have
	// thin slivers next to a large face (e.g. Voronoi cells near a degenerate
	// vertex), that appear and disappear from frame to frame. Alone, a sliver
	// would limit the bevels of its edges to almost zero.
	PointSets surfaces(static_cast<int32_t>(mesh.faceNormals.size()));
	for (const auto& [edge, uses] : edges) {
		if (uses.size() == 2 && uses[0].face != uses[1].face && dihedral(uses[0].face, uses[1].face) < theFlatAngle)
			surfaces.join(uses[0].face, uses[1].face);
	}
	std::vector<std::vector<int32_t>> surfacePoints(mesh.faceNormals.size());
	for (size_t f = 0; f < facePoints.size(); f++) {
		std::vector<int32_t>& points = surfacePoints[surfaces.find(static_cast<int32_t>(f))];
		points.insert(points.end(), facePoints[f].begin(), facePoints[f].end());
	}

	// length of the edge between two surfaces, summed over its mesh segments
	auto surfacePair = [&](int32_t f1, int32_t f2) {
		const int32_t s1 = surfaces.find(f1), s2 = surfaces.find(f2);
		return std::make_pair(std::min(s1, s2), std::max(s1, s2));
	};
	std::map<std::pair<int32_t, int32_t>, double> edgeLengths;
	for (const auto& [edge, uses] : edges) {
		if (uses.size() == 2 && surfaces.find(uses[0].face) != surfaces.find(uses[1].face))
			edgeLengths[surfacePair(uses[0].face, uses[1].face)] += (mesh.points[edge.second] - mesh.points[edge.first]).Modulus();
	}

	// distance from the edge into the surface of the face, to its farthest point
	auto faceWidth = [&](int32_t face, const gp_XYZ& p, const gp_XYZ& inward) {
		double width = 0.0;
		for (int32_t i : surfacePoints[surfaces.find(face)])
			width = std::max(width, inward.Dot(mesh.points[i] - p));
		return width;
	};

	std::vector<Bevel> bevels;
	for (const auto& [edge, uses] : edges) {
		if (uses.size() != 2 || uses[0].face == uses[1].face)
			continue;

		const gp_XYZ& n1 = mesh.faceNormals[uses[0].face];
		const gp_XYZ& n2 = mesh.faceNormals[uses[1].face];
		const double angle = dihedral(uses[0].face, uses[1].face);
		const gp_XYZ bisector = n1 + n2;
		// nearly flat edges are neither cut nor locked: the float noise of the input
		// decides if they are convex or concave, which changes from frame to frame
		if (angle < std::max(minAngle, theFlatAngle) || bisector.Modulus() <= 1e-9)
			continue;

		const gp_XYZ& a = mesh.points[uses[0].from];
		const gp_XYZ& b = mesh.points[uses[0].from == edge.first ? edge.second : edge.first];
		gp_XYZ dir = b - a;
		if (dir.Modulus() <= 0.0)
			continue;
		dir.Normalize();

		// faces are counter-clockwise, so their inside is left of the edge direction
		const gp_XYZ inward1 = n1.Crossed(dir).Normalized();
		const gp_XYZ inward2 = n2.Crossed(dir.Reversed()).Normalized();

		// going into the first face rises above the second one: the edge is concave
		if (inward1.Dot(n2) > 0.0) {
			mesh.locked[edge.first] = 1;
			mesh.locked[edge.second] = 1;
			continue;
		}

		const gp_XYZ n = bisector.Normalized();
		const double halfWidth = 0.5 * std::min(faceWidth(uses[0].face, a, inward1), faceWidth(uses[1].face, a, inward2));
		// the bevel goes sin(angle / 2) deep per unit of distance
		const double halfToCenter = 0.5 * std::max(n.Dot(a - center), 0.0) / std::sin(0.5 * angle);
		// a short edge gets a narrow bevel: when the edge shrinks to nothing (e.g. a
		// Voronoi cell changes its topology) its bevel fades out instead of popping
		const double halfLength = 0.5 * edgeLengths[surfacePair(uses[0].face, uses[1].face)];
		// edges just over the Min Edge Angle get a narrow bevel, so it fades in
		// while their angle grows instead of popping up
		const double fade = std::min(1.0, (angle - minAngle) / theAngleFade);
		const double d = fade * std::min({ distance, halfWidth, halfToCenter, halfLength });
		if (d > 0.0)
			bevels.push_back({ n, n.Dot(a + inward1 * d), a, b, d });
	}
	return bevels;
}

double
distanceToSegment(const gp_XYZ& p, const gp_XYZ& a, const gp_XYZ& b)
{
	const gp_XYZ ab = b - a;
	const double t = std::max(0.0, std::min(1.0, (p - a).Dot(ab) / ab.SquareModulus()));
	return (p - (a + ab * t)).Modulus();
}

// Adds the triangles of a closed loop of points, counter-clockwise around normal.
// All loop edges must get a triangle, or the mesh is no longer closed. The plane
// crosses the diagonals of faces, so loops often have many points on one line.
void
addCap(CutMesh& mesh, const std::vector<int32_t>& loop, const gp_XYZ& normal, int32_t face)
{
	const gp_XYZ helper = std::abs(normal.X()) < 0.9 ? gp_XYZ(1, 0, 0) : gp_XYZ(0, 1, 0);
	const gp_XYZ u = normal.Crossed(helper).Normalized();
	const gp_XYZ v = normal.Crossed(u);
	struct Point2
	{
		double x, y;
	};
	auto project = [&](const gp_XYZ& p) { return Point2{ p.Dot(u), p.Dot(v) }; };
	auto cross = [](const Point2& a, const Point2& b, const Point2& c) {
		return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
	};

	std::map<int32_t, Point2> uv;
	gp_XYZ center(0, 0, 0);
	for (int32_t i : loop) {
		uv[i] = project(mesh.points[i]);
		center += mesh.points[i];
	}
	center /= static_cast<double>(loop.size());
	const Point2 c2 = project(center);

	// a fan from the center works on every loop that is star-shaped around it,
	// that is every loop of a convex part
	double scale = 0.0;
	for (int32_t i : loop)
		scale = std::max(scale, (mesh.points[i] - center).SquareModulus());
	bool star = true;
	for (size_t k = 0; k < loop.size(); k++)
		star = star && cross(c2, uv[loop[k]], uv[loop[(k + 1) % loop.size()]]) >= -1e-9 * scale;
	if (star) {
		const int32_t centerIndex = static_cast<int32_t>(mesh.points.size());
		mesh.points.push_back(center);
		for (size_t k = 0; k < loop.size(); k++) {
			mesh.triangles.push_back({ centerIndex, loop[k], loop[(k + 1) % loop.size()] });
			mesh.triangleFaces.push_back(face);
		}
		return;
	}

	// ear clipping for the other loops
	std::vector<int32_t> open = loop;
	while (open.size() > 3) {
		bool clipped = false;
		for (size_t k = 0; k < open.size() && !clipped; k++) {
			const int32_t a = open[(k + open.size() - 1) % open.size()];
			const int32_t b = open[k];
			const int32_t c = open[(k + 1) % open.size()];
			if (cross(uv[a], uv[b], uv[c]) <= 0.0)
				continue;
			bool empty = true;
			for (int32_t p : open) {
				if (p != a && p != b && p != c && cross(uv[a], uv[b], uv[p]) > 0.0 &&
					cross(uv[b], uv[c], uv[p]) > 0.0 && cross(uv[c], uv[a], uv[p]) > 0.0)
					empty = false;
			}
			if (!empty)
				continue;
			mesh.triangles.push_back({ a, b, c });
			mesh.triangleFaces.push_back(face);
			open.erase(open.begin() + k);
			clipped = true;
		}
		if (!clipped)
			break;
	}
	// what is left is one triangle or points on one line: close it anyway
	for (size_t k = 1; k + 1 < open.size(); k++) {
		mesh.triangles.push_back({ open[0], open[k], open[k + 1] });
		mesh.triangleFaces.push_back(face);
	}
}

// Removes the piece above the bevel plane nearest to the edge and closes the hole
void
clipCutMesh(CutMesh& mesh, const Bevel& bevel, double eps)
{
	const size_t numPoints = mesh.points.size();
	std::vector<double> heights(numPoints);
	for (size_t i = 0; i < numPoints; i++)
		heights[i] = bevel.n.Dot(mesh.points[i]) - bevel.c;

	auto above = [&](int32_t i) {
		return heights[i] > eps && !(static_cast<size_t>(i) < mesh.locked.size() && mesh.locked[i]);
	};

	// connected pieces of the points above the plane
	PointSets sets(static_cast<int32_t>(numPoints));
	std::vector<char> used(numPoints, 0);
	for (const std::array<int32_t, 3>& v : mesh.triangles) {
		for (int k = 0; k < 3; k++) {
			used[v[k]] = 1;
			const int32_t other = v[(k + 1) % 3];
			if (above(v[k]) && above(other))
				sets.join(v[k], other);
		}
	}

	int32_t nearest = -1;
	double nearestDistance = 2.0 * bevel.distance + eps;
	for (size_t i = 0; i < numPoints; i++) {
		if (!used[i] || !above(static_cast<int32_t>(i)))
			continue;
		const double d = distanceToSegment(mesh.points[i], bevel.a, bevel.b);
		if (d <= nearestDistance) {
			nearestDistance = d;
			nearest = sets.find(static_cast<int32_t>(i));
		}
	}
	if (nearest < 0)
		return;

	std::vector<char> removed(numPoints, 0);
	for (size_t i = 0; i < numPoints; i++)
		removed[i] = used[i] && above(static_cast<int32_t>(i)) && sets.find(static_cast<int32_t>(i)) == nearest;

	// the point where an edge crosses the plane, shared by the triangles of the edge
	std::map<std::pair<int32_t, int32_t>, int32_t> crossings;
	auto crossing = [&](int32_t gone, int32_t kept) {
		if (heights[kept] >= -eps)
			return kept;	// the kept point is on the plane, or locked above it
		const auto it = crossings.find({ std::min(gone, kept), std::max(gone, kept) });
		if (it != crossings.end())
			return it->second;
		const int32_t index = static_cast<int32_t>(mesh.points.size());
		const double t = heights[gone] / (heights[gone] - heights[kept]);
		const gp_XYZ p = mesh.points[gone] + (mesh.points[kept] - mesh.points[gone]) * t;
		mesh.points.push_back(p);
		crossings[{ std::min(gone, kept), std::max(gone, kept) }] = index;
		return index;
	};

	std::vector<std::array<int32_t, 3>> triangles;
	std::vector<int32_t> triangleFaces;
	std::map<int32_t, std::vector<int32_t>> capEdges;
	std::vector<int32_t> poly;
	for (size_t t = 0; t < mesh.triangles.size(); t++) {
		const std::array<int32_t, 3>& v = mesh.triangles[t];
		if (!removed[v[0]] && !removed[v[1]] && !removed[v[2]]) {
			triangles.push_back(v);
			triangleFaces.push_back(mesh.triangleFaces[t]);
			continue;
		}

		poly.clear();
		int32_t exit = -1, entry = -1;
		auto add = [&](int32_t i) {
			if (poly.empty() || poly.back() != i)
				poly.push_back(i);
		};
		for (int k = 0; k < 3; k++) {
			const int32_t a = v[k], b = v[(k + 1) % 3];
			if (!removed[a]) {
				add(a);
				if (removed[b]) {
					exit = crossing(b, a);
					add(exit);
				}
			} else if (!removed[b]) {
				entry = crossing(a, b);
				add(entry);
			}
		}
		while (poly.size() > 1 && poly.front() == poly.back())
			poly.pop_back();
		for (size_t k = 1; k + 1 < poly.size(); k++) {
			triangles.push_back({ poly[0], poly[k], poly[k + 1] });
			triangleFaces.push_back(mesh.triangleFaces[t]);
		}

		// the cap runs against the edge of the kept polygon
		if (exit >= 0 && entry >= 0 && exit != entry)
			capEdges[entry].push_back(exit);
	}
	mesh.triangles = std::move(triangles);
	mesh.triangleFaces = std::move(triangleFaces);

	// close every loop of the new edges with its own part of the cap
	const int32_t capFace = static_cast<int32_t>(mesh.faceNormals.size());
	mesh.faceNormals.push_back(bevel.n);
	while (!capEdges.empty()) {
		const int32_t start = capEdges.begin()->first;
		std::vector<int32_t> loop;
		int32_t current = start;
		do {
			const auto it = capEdges.find(current);
			if (it == capEdges.end()) {
				loop.clear();	// the loop doesn't close
				break;
			}
			loop.push_back(current);
			current = it->second.back();
			it->second.pop_back();
			if (it->second.empty())
				capEdges.erase(it);
		} while (current != start);
		if (loop.size() >= 3)
			addCap(mesh, loop, bevel.n, capFace);
	}
}

// Appends the triangles to the output. Every face gets its own points, so
// normals stay sharp across the edges between faces (like extractMesh). All
// faces are flat, so Smooth and Surface normals are both the face normal.
// Thin triangles of the cuts can't disturb them this way.
void
appendCutMesh(const CutMesh& mesh, NormalsMenuItems normals, std::vector<TD::Position>& points,
	std::vector<TD::Vector>& pointNormals, std::vector<int32_t>& indices)
{
	std::vector<std::vector<size_t>> faceTriangles(mesh.faceNormals.size());
	for (size_t t = 0; t < mesh.triangles.size(); t++)
		faceTriangles[mesh.triangleFaces[t]].push_back(t);

	std::map<int32_t, int32_t> remap;
	for (size_t f = 0; f < faceTriangles.size(); f++) {
		remap.clear();
		const gp_XYZ& n = mesh.faceNormals[f];
		for (size_t t : faceTriangles[f]) {
			for (int32_t i : mesh.triangles[t]) {
				const auto [it, added] = remap.emplace(i, static_cast<int32_t>(points.size()));
				if (added) {
					const gp_XYZ& p = mesh.points[i];
					points.emplace_back(static_cast<float>(p.X()), static_cast<float>(p.Y()), static_cast<float>(p.Z()));
					if (normals != NormalsMenuItems::Off)
						pointNormals.emplace_back(static_cast<float>(n.X()), static_cast<float>(n.Y()), static_cast<float>(n.Z()));
				}
				indices.push_back(it->second);
			}
		}
	}
}

// Cuts the part into mesh, the part then becomes null. Anything that is not
// one closed solid, or has no edges to cut, stays in part, unmodified.
// Returns false if the part can't be triangulated.
bool
planeCutChamfer(TopoDS_Shape& part, const OcctSettings& settings, CutMesh& mesh)
{
	TopoDS_Shape solid;
	int numSolids = 0;
	for (TopExp_Explorer e(part, TopAbs_SOLID); e.More(); e.Next(), numSolids++)
		solid = e.Current();
	if (numSolids != 1)
		return true;

	double size = 0.0;
	if (!buildCutMesh(solid, settings.deflection, settings.angularDeflection, mesh, size))
		return false;

	const std::vector<Bevel> bevels = makeBevels(mesh, settings.radius, settings.edgeAngle);
	if (bevels.empty()) {
		mesh = CutMesh();
		return true;
	}

	const double eps = 1e-9 * size;
	for (const Bevel& bevel : bevels)
		clipCutMesh(mesh, bevel, eps);

	part.Nullify();
	return true;
}

// Point normals from the triangles: every point gets the average of the normals
// of its triangles, weighted by the triangle angle at that point (like the
// Attribute Create SOP). They always match the triangles that are drawn.
void
computeMeshNormals(const std::vector<TD::Position>& points, const std::vector<int32_t>& indices,
	std::vector<TD::Vector>& pointNormals)
{
	std::vector<gp_XYZ> sums(points.size(), gp_XYZ(0, 0, 0));
	auto xyz = [&](int32_t i) { return gp_XYZ(points[i].x, points[i].y, points[i].z); };

	for (size_t t = 0; t + 2 < indices.size(); t += 3) {
		const int32_t v[3] = { indices[t], indices[t + 1], indices[t + 2] };
		const gp_XYZ p[3] = { xyz(v[0]), xyz(v[1]), xyz(v[2]) };
		gp_XYZ n = (p[1] - p[0]).Crossed(p[2] - p[0]);
		const double len = n.Modulus();
		if (len <= 0.0)
			continue;
		n /= len;

		for (int k = 0; k < 3; k++) {
			gp_XYZ e1 = p[(k + 1) % 3] - p[k];
			gp_XYZ e2 = p[(k + 2) % 3] - p[k];
			const double l1 = e1.Modulus(), l2 = e2.Modulus();
			if (l1 <= 0.0 || l2 <= 0.0)
				continue;
			const double cosA = std::max(-1.0, std::min(1.0, e1.Dot(e2) / (l1 * l2)));
			sums[v[k]] += n * std::acos(cosA);
		}
	}

	pointNormals.resize(points.size());
	for (size_t i = 0; i < sums.size(); i++) {
		const double len = sums[i].Modulus();
		const gp_XYZ n = len > 0.0 ? sums[i] / len : gp_XYZ(0, 0, 1);
		pointNormals[i] = TD::Vector(static_cast<float>(n.X()), static_cast<float>(n.Y()), static_cast<float>(n.Z()));
	}
}

// Triangulates the shape and appends its triangles. Every face gets its own
// points, so normals stay sharp across the edges between faces.
void
extractMesh(const TopoDS_Shape& shape, double deflection, double angularDeflection,
	NormalsMenuItems normals, std::vector<TD::Position>& points, std::vector<TD::Vector>& pointNormals,
	std::vector<int32_t>& indices)
{
	const bool surfaceNormals = normals == NormalsMenuItems::Surface;

	BRepMesh_IncrementalMesh mesher(shape, deflection, Standard_False, angularDeflection, Standard_True);

	for (TopExp_Explorer e(shape, TopAbs_FACE); e.More(); e.Next()) {
		const TopoDS_Face& face = TopoDS::Face(e.Current());
		TopLoc_Location loc;
		Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull())
			continue;

		if (surfaceNormals && !tri->HasNormals())
			BRepLib_ToolTriangulatedShape::ComputeNormals(face, tri);

		const gp_Trsf trsf = loc.Transformation();
		// triangles and normals follow the surface, reversed faces point the other way
		const bool reversed = face.Orientation() == TopAbs_REVERSED;
		const int32_t base = static_cast<int32_t>(points.size());

		for (int n = 1; n <= tri->NbNodes(); n++) {
			const gp_Pnt p = tri->Node(n).Transformed(trsf);
			points.emplace_back(static_cast<float>(p.X()), static_cast<float>(p.Y()), static_cast<float>(p.Z()));

			if (surfaceNormals) {
				gp_Dir d = tri->HasNormals() ? tri->Normal(n) : gp_Dir(0, 0, 1);
				d.Transform(trsf);
				if (reversed)
					d.Reverse();
				pointNormals.emplace_back(static_cast<float>(d.X()), static_cast<float>(d.Y()), static_cast<float>(d.Z()));
			}
		}

		for (int t = 1; t <= tri->NbTriangles(); t++) {
			int a, b, c;
			tri->Triangle(t).Get(a, b, c);
			if (reversed)
				std::swap(b, c);
			indices.push_back(base + a - 1);	// OpenCascade indices start at 1
			indices.push_back(base + b - 1);
			indices.push_back(base + c - 1);
		}
	}

	// exact surface normals don't always match the triangles: on thin faces the
	// float noise of the input tilts the triangles off the surface
	if (normals == NormalsMenuItems::Smooth)
		computeMeshNormals(points, indices, pointNormals);
}

}


void
occtProcess(const TD::Position* points, int32_t numPoints, const std::vector<int32_t>& polygons,
	const OcctSettings& settings, OcctResult& result)
{
	result = OcctResult();

	try {
		std::vector<TopoDS_Shape> pieces;
		const int skipped = buildSewedPieces(points, numPoints, polygons, pieces);
		if (pieces.empty()) {
			result.error = "Input has no polygons";
			return;
		}

		std::vector<TopoDS_Shape> parts;
		int numSolids = 0;
		for (const TopoDS_Shape& piece : pieces) {
			std::vector<TopoDS_Shape> pieceParts = makeParts(piece, numSolids);
			parts.insert(parts.end(), pieceParts.begin(), pieceParts.end());
		}

		// failed parts, split by whether the radius / distance is too large for them
		int tooThin = 0;
		double tooThinEdge = std::numeric_limits<double>::infinity();
		int failed = 0;
		std::string failure;
		int cutFailed = 0;	// Plane Cut only
		std::vector<CutMesh> cutMeshes;
		const bool planeCut = settings.modifier == ModifierMenuItems::Planecut;
		if (settings.radius > 0.0) {
			for (TopoDS_Shape& part : parts) {
				if (planeCut) {
					cutMeshes.emplace_back();
					if (!planeCutChamfer(part, settings, cutMeshes.back()))
						cutFailed++;
					continue;
				}

				std::string partFailure;
				double shortestEdge = 0.0;
				if (applyModifier(settings.modifier, settings.radius, settings.edgeAngle, part, partFailure, shortestEdge))
					continue;
				// broken input geometry is reported as it is, whatever the size
				const bool invalidInput = partFailure == theInvalidInput;
				if (!invalidInput && 2.0 * settings.radius >= shortestEdge * (1.0 - 1e-6)) {
					tooThin++;
					tooThinEdge = std::min(tooThinEdge, shortestEdge);
				} else {
					failed++;
					failure = partFailure;
				}
			}
		}

		BRep_Builder builder;
		TopoDS_Compound compound;
		builder.MakeCompound(compound);
		for (const TopoDS_Shape& part : parts) {
			if (!part.IsNull())
				builder.Add(compound, part);
		}

		extractMesh(compound, settings.deflection, settings.angularDeflection, settings.normals,
			result.points, result.normals, result.indices);
		for (const CutMesh& mesh : cutMeshes)
			appendCutMesh(mesh, settings.normals, result.points, result.normals, result.indices);

		std::stringstream warning;
		if (settings.radius > 0.0 && numSolids == 0)
			warning << "Input has no closed shells, nothing to modify (the input must be a closed, watertight surface). ";
		const bool fillet = settings.modifier == ModifierMenuItems::Fillet;
		if (tooThin > 0) {
			warning << (fillet ? "Fillet" : "Chamfer") << " skipped on " << tooThin << " of " << parts.size()
				<< " parts: " << (fillet ? "radius " : "distance ") << settings.radius
				<< " is too large, those parts have edges as short as " << tooThinEdge
				<< " (" << (fillet ? "radius" : "distance") << " must be under half the edge length). ";
		}
		if (cutFailed > 0)
			warning << "Plane cut left " << cutFailed << " of " << parts.size() << " parts unmodified: they could not be triangulated. ";
		if (failed > 0)
			warning << failed << " of " << parts.size() << " parts left unmodified: " << failure << ". ";
		if (skipped > 0)
			warning << skipped << " primitives skipped (not closed polygons or degenerate). ";
		result.warning = warning.str();
	} catch (const Standard_Failure& e) {
		result.error = std::string("OpenCascade error: ") + e.GetMessageString();
	} catch (const std::exception& e) {
		result.error = std::string("OpenCascade error: ") + e.what();
	}

	if (!result.error.empty()) {
		result.points.clear();
		result.normals.clear();
		result.indices.clear();
	}
}
