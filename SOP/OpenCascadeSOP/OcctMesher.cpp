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
#include <TopTools_IndexedMapOfShape.hxx>
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
std::vector<TopoDS_Edge>
sharpEdges(const TopoDS_Shape& part, double minAngle)
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

// Applies the modifier to all sharp edges of the part.
// Returns false (and leaves the part untouched) if OpenCascade fails.
// shortestEdge receives the length of the shortest modified edge: a fillet or
// chamfer fails once the radius / distance reaches half of it, because the
// modified strips on both ends of that edge would overlap.
bool
applyModifier(ModifierMenuItems modifier, double radius, double minAngle,
	TopoDS_Shape& part, std::string& failure, double& shortestEdge)
{
	const std::vector<TopoDS_Edge> edges = sharpEdges(part, minAngle);
	shortestEdge = std::numeric_limits<double>::infinity();
	if (edges.empty())
		return true;
	for (const TopoDS_Edge& edge : edges)
		shortestEdge = std::min(shortestEdge, edgeLength(edge));

	// the fillet algorithms don't check their input and can crash on broken shapes
	if (!BRepCheck_Analyzer(part).IsValid()) {
		failure = theInvalidInput;
		return false;
	}

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
			part = fillet.Shape();
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
			part = chamfer.Shape();
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

// ---------------------------------------------------------------------------
// Plane cut chamfer for convex parts.
// A chamfer of a convex solid is the intersection of the solid with one extra
// half-space per edge: the bevel plane, tilted halfway between the two faces
// and set back so it cuts the given distance into both faces. Unlike
// BRepFilletAPI_MakeChamfer this never fails: faces that are cut away completely
// just disappear, so thin parts get bevels that meet in a ridge or a point.

// keeps the side n.x <= c
struct CutPlane
{
	gp_XYZ	n;
	double	c;
};

// Convex polyhedron as a list of convex polygons, each counter-clockwise
// around its outward normal
using CutPolygon = std::vector<gp_XYZ>;

std::vector<CutPolygon>
makeCutBox(const gp_XYZ& lo, const gp_XYZ& hi)
{
	auto corner = [&](int i) {
		return gp_XYZ((i & 1) ? hi.X() : lo.X(), (i & 2) ? hi.Y() : lo.Y(), (i & 4) ? hi.Z() : lo.Z());
	};
	static const int faces[6][4] = {
		{ 0, 4, 6, 2 }, { 1, 3, 7, 5 }, { 0, 1, 5, 4 }, { 2, 6, 7, 3 }, { 0, 2, 3, 1 }, { 4, 5, 7, 6 }
	};
	std::vector<CutPolygon> polys;
	for (const auto& f : faces)
		polys.push_back({ corner(f[0]), corner(f[1]), corner(f[2]), corner(f[3]) });
	return polys;
}

// Clips the polyhedron with a half-space and closes the hole with a cap polygon
void
clipPolyhedron(std::vector<CutPolygon>& polys, const CutPlane& plane, double eps)
{
	bool cut = false;
	for (const CutPolygon& poly : polys) {
		for (const gp_XYZ& p : poly)
			cut = cut || plane.n.Dot(p) - plane.c > eps;
	}
	if (!cut)
		return;

	std::vector<CutPolygon> out;
	std::vector<gp_XYZ> capPoints;
	for (const CutPolygon& poly : polys) {
		CutPolygon clipped;
		for (size_t i = 0; i < poly.size(); i++) {
			const gp_XYZ& a = poly[i];
			const gp_XYZ& b = poly[(i + 1) % poly.size()];
			const double da = plane.n.Dot(a) - plane.c;
			const double db = plane.n.Dot(b) - plane.c;
			if (da <= eps) {
				clipped.push_back(a);
				if (da >= -eps)
					capPoints.push_back(a);
			}
			if ((da < -eps && db > eps) || (da > eps && db < -eps)) {
				const gp_XYZ p = a + (b - a) * (da / (da - db));
				clipped.push_back(p);
				capPoints.push_back(p);
			}
		}
		if (clipped.size() >= 3)
			out.push_back(std::move(clipped));
	}

	// cap: the points on the plane, sorted counter-clockwise around its normal
	std::vector<gp_XYZ> cap;
	for (const gp_XYZ& p : capPoints) {
		bool dup = false;
		for (const gp_XYZ& q : cap)
			dup = dup || (p - q).Modulus() <= eps;
		if (!dup)
			cap.push_back(p);
	}
	if (cap.size() >= 3) {
		gp_XYZ center(0, 0, 0);
		for (const gp_XYZ& p : cap)
			center += p;
		center /= static_cast<double>(cap.size());

		const gp_XYZ helper = std::abs(plane.n.X()) < 0.9 ? gp_XYZ(1, 0, 0) : gp_XYZ(0, 1, 0);
		const gp_XYZ u = plane.n.Crossed(helper).Normalized();
		const gp_XYZ v = plane.n.Crossed(u);
		std::sort(cap.begin(), cap.end(), [&](const gp_XYZ& a, const gp_XYZ& b) {
			return std::atan2((a - center).Dot(v), (a - center).Dot(u)) <
				std::atan2((b - center).Dot(v), (b - center).Dot(u));
		});
		out.push_back(std::move(cap));
	}
	polys = std::move(out);
}

enum class PlaneCutResult
{
	Done,
	NotConvex,	// or not a single planar solid: use the OpenCascade chamfer instead
	Vanished,	// the bevels cut the whole part away
	Failed		// the result could not be turned into a valid solid
};

PlaneCutResult
planeCutChamfer(TopoDS_Shape& part, double distance, double minAngle)
{
	TopoDS_Shape solid;
	int numSolids = 0;
	for (TopExp_Explorer e(part, TopAbs_SOLID); e.More(); e.Next(), numSolids++)
		solid = e.Current();
	if (numSolids != 1)
		return PlaneCutResult::NotConvex;

	TopTools_IndexedMapOfShape vertices;
	TopExp::MapShapes(solid, TopAbs_VERTEX, vertices);
	if (vertices.Extent() < 4)
		return PlaneCutResult::NotConvex;

	gp_XYZ lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
	std::vector<gp_XYZ> points;
	for (int i = 1; i <= vertices.Extent(); i++) {
		const gp_XYZ p = BRep_Tool::Pnt(TopoDS::Vertex(vertices(i))).XYZ();
		points.push_back(p);
		lo.SetCoord(std::min(lo.X(), p.X()), std::min(lo.Y(), p.Y()), std::min(lo.Z(), p.Z()));
		hi.SetCoord(std::max(hi.X(), p.X()), std::max(hi.Y(), p.Y()), std::max(hi.Z(), p.Z()));
	}
	const double size = (hi - lo).Modulus();
	// the input comes as floats and merged faces keep that noise
	const double convexTol = theUnifyLinearTolerance + 1e-6 * size;
	const double eps = 1e-9 * size;

	// face planes, the part must lie on the inner side of all of them
	std::vector<CutPlane> planes;
	for (TopExp_Explorer e(solid, TopAbs_FACE); e.More(); e.Next()) {
		const TopoDS_Face& face = TopoDS::Face(e.Current());
		gp_Dir n;
		if (!planeNormal(face, n))
			return PlaneCutResult::NotConvex;
		CutPlane plane{ n.XYZ(), -1e300 };
		for (TopExp_Explorer v(face, TopAbs_VERTEX); v.More(); v.Next())
			plane.c = std::max(plane.c, plane.n.Dot(BRep_Tool::Pnt(TopoDS::Vertex(v.Current())).XYZ()));
		for (const gp_XYZ& p : points) {
			if (plane.n.Dot(p) > plane.c + convexTol)
				return PlaneCutResult::NotConvex;
		}
		planes.push_back(plane);
	}

	// bevel planes of the sharp edges
	const size_t numFacePlanes = planes.size();
	TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
	TopExp::MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
	for (int i = 1; i <= edgeFaces.Extent(); i++) {
		const TopoDS_Edge& edge = TopoDS::Edge(edgeFaces.FindKey(i));
		const TopTools_ListOfShape& faces = edgeFaces.FindFromIndex(i);
		if (faces.Extent() != 2 || BRep_Tool::Degenerated(edge))
			continue;

		gp_Dir d1, d2;
		if (!planeNormal(TopoDS::Face(faces.First()), d1) || !planeNormal(TopoDS::Face(faces.Last()), d2))
			return PlaneCutResult::NotConvex;
		if (d1.Angle(d2) < minAngle)
			continue;

		TopoDS_Vertex v0, v1;
		TopExp::Vertices(edge, v0, v1);
		if (v0.IsNull() || v1.IsNull())
			continue;
		const gp_XYZ p0 = BRep_Tool::Pnt(v0).XYZ();
		const gp_XYZ dir = BRep_Tool::Pnt(v1).XYZ() - p0;
		const gp_XYZ bisector = d1.XYZ() + d2.XYZ();
		if (dir.Modulus() <= eps || bisector.Modulus() <= 1e-9)
			continue;

		// direction from the edge into the first face, away from the second one
		gp_XYZ inward = dir.Crossed(d1.XYZ()).Normalized();
		if (inward.Dot(d2.XYZ()) > 0.0)
			inward.Reverse();

		const gp_XYZ n = bisector.Normalized();
		planes.push_back({ n, n.Dot(p0 + inward * distance) });
	}
	if (planes.size() == numFacePlanes)
		return PlaneCutResult::Done;	// no sharp edges

	// intersect all half-spaces, starting from a box around the part
	const gp_XYZ margin(0.1 * size + 1e-6, 0.1 * size + 1e-6, 0.1 * size + 1e-6);
	std::vector<CutPolygon> polys = makeCutBox(lo - margin, hi + margin);
	for (const CutPlane& plane : planes) {
		clipPolyhedron(polys, plane, eps);
		if (polys.size() < 4)
			return PlaneCutResult::Vanished;
	}

	// weld the corners computed separately by neighbouring polygons, drop
	// the edges and polygons that collapse
	const double weldTol = std::max(10.0 * theSewingTolerance, 1e-7 * size);
	std::vector<gp_XYZ> corners;
	for (CutPolygon& poly : polys) {
		CutPolygon welded;
		for (const gp_XYZ& p : poly) {
			gp_XYZ q = p;
			bool found = false;
			for (const gp_XYZ& c : corners) {
				if ((c - p).Modulus() <= weldTol) {
					q = c;
					found = true;
					break;
				}
			}
			if (!found)
				corners.push_back(p);
			if (welded.empty() || !welded.back().IsEqual(q, 0.0))
				welded.push_back(q);
		}
		while (welded.size() > 1 && welded.front().IsEqual(welded.back(), 0.0))
			welded.pop_back();
		poly = welded.size() >= 3 ? std::move(welded) : CutPolygon();
	}
	polys.erase(std::remove_if(polys.begin(), polys.end(), [](const CutPolygon& p) { return p.empty(); }), polys.end());
	if (polys.size() < 4)
		return PlaneCutResult::Vanished;

	BRepBuilderAPI_Sewing sewing(theSewingTolerance);
	std::vector<gp_Pnt> pts;
	for (const CutPolygon& poly : polys) {
		pts.clear();
		for (const gp_XYZ& p : poly)
			pts.emplace_back(p);
		const TopoDS_Face face = makePolygonFace(pts);
		if (face.IsNull())
			return PlaneCutResult::Failed;
		sewing.Add(face);
	}
	sewing.Perform();

	TopoDS_Shape result;
	for (TopExp_Explorer e(sewing.SewedShape(), TopAbs_SHELL); e.More(); e.Next()) {
		if (!result.IsNull() || !BRep_Tool::IsClosed(e.Current()))
			return PlaneCutResult::Failed;
		BRepBuilderAPI_MakeSolid makeSolid(TopoDS::Shell(e.Current()));
		if (!makeSolid.IsDone())
			return PlaneCutResult::Failed;
		ShapeFix_Solid fix(makeSolid.Solid());
		fix.Perform();
		result = fix.Solid();
	}
	if (result.IsNull() || !BRepCheck_Analyzer(result).IsValid())
		return PlaneCutResult::Failed;

	part = result;
	return PlaneCutResult::Done;
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
		// Plane Cut only
		int notConvex = 0;
		int cutFailed = 0;
		int vanished = 0;
		const bool planeCut = settings.modifier == ModifierMenuItems::Planecut;
		if (settings.radius > 0.0) {
			for (TopoDS_Shape& part : parts) {
				if (planeCut) {
					const PlaneCutResult cut = planeCutChamfer(part, settings.radius, settings.edgeAngle);
					if (cut == PlaneCutResult::Done)
						continue;
					if (cut == PlaneCutResult::Vanished) {
						vanished++;
						part.Nullify();
						continue;
					}
					// not convex or the cut failed: the regular chamfer below
					if (cut == PlaneCutResult::NotConvex)
						notConvex++;
					else
						cutFailed++;
				}

				const ModifierMenuItems modifier = planeCut ? ModifierMenuItems::Chamfer : settings.modifier;
				std::string partFailure;
				double shortestEdge = 0.0;
				if (applyModifier(modifier, settings.radius, settings.edgeAngle, part, partFailure, shortestEdge))
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
		if (notConvex > 0)
			warning << notConvex << " of " << parts.size() << " parts are not convex, the regular chamfer was used on them. ";
		if (cutFailed > 0)
			warning << "Plane cut failed on " << cutFailed << " of " << parts.size() << " parts, the regular chamfer was used on them. ";
		if (vanished > 0)
			warning << vanished << " of " << parts.size() << " parts were cut away completely, distance " << settings.radius
				<< " is larger than the parts. ";
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
