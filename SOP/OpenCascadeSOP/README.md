# OpenCascade SOP
Applies an [OpenCascade (OCCT)](https://dev.opencascade.org) modifier (Chamfer, Chamfer plane cut and Fillet) to the sharp edges of the input geometry: the input polygons are sewn into solids, coplanar faces are merged, the modifier is applied and the result is triangulated back to TouchDesigner.

Works best with flat-faced input (boxes, low-poly models). Smooth meshes (e.g. Sphere SOP) have no real edges, every triangle edge is treated as one.

OCCT libraries are linked dynamically.

MacOS: *brew install opencascade*. The plugin loads the Homebrew OCCT libraries at runtime.

Windows: use [vcpkg](https://vcpkg.io/) (classic mode with `vcpkg integrate install`) and install the dynamic triplet: `vcpkg install opencascade:x64-windows`. Visual Studio copies the OCCT DLLs next to the plugin DLL, and the Release build copies all of them to the TouchDesigner Plugins folder. With CMake, point `OpenCASCADE_DIR` to the folder with `OpenCASCADEConfig.cmake` (CMake 3.21+ also copies the OCCT DLLs).

## Parameters
* **Modifier** - select the OCCT modifier.
  * **Fillet** - rounds the edges (BRepFilletAPI_MakeFillet).
  * **Chamfer** - bevels the edges (BRepFilletAPI_MakeChamfer).
  * **Chamfer (Plane Cut)** - bevels convex parts by cutting them with one plane per edge, halfway between its two faces. Never fails on thin parts: once the distance is over half a face's width the bevels meet in a ridge or a point and the face disappears; a part smaller than the distance is cut away completely. Faster than Chamfer for animated input. Where three bevels meet the corner is a point, not a small triangle like in Chamfer. Parts that are not convex use the regular Chamfer.
* **Radius / Distance** - fillet radius or chamfer distance. 0 outputs the input unmodified (triangulated).
* **Min Edge Angle** - only edges whose faces meet at a larger angle (degrees) are modified.
* **Mesh Deflection** - max distance between the output triangles and the real surface, smaller is smoother.
* **Mesh Angular Deflection** - max angle (degrees) between neighbouring triangles on curved surfaces.
* **Normals** - how the point normals are computed.
  * **Off** - no normals.
  * **Smooth** (default) - every point gets the average of the normals of its triangles, weighted by the triangle angle at that point (like Attribute Create SOP > Compute Normals). Always matches the drawn triangles. Every OpenCascade face has its own points, so normals stay sharp across the edges between faces.
  * **Surface** - exact normals of the OpenCascade surfaces. Smoothest on fillets, but on thin faces the triangles can be tilted against them (float precision of the input), which shows as shading artifacts.

## Notes
* Polygons are grouped into shells by shared points. Groups that are not closed on their own (e.g. Box SOP, where every face has its own corner points) are welded by point position, so they don't need a Fuse/Facet SOP. Closed cells that only touch each other stay separate.
* Input without any closed shell produces a warning and is output unmodified.
* Every closed shell becomes its own solid and is modified separately. A part the modifier fails on is output unmodified and reported in the warning.
* The Radius / Distance must be under half the length of the shortest modified edge of a part (for a box: half its thinnest side), otherwise the fillets / chamfers of both ends of that edge overlap and the part is skipped. The warning reports the shortest edge of the skipped parts.
* The result is cached, the node recomputes only when the input or parameters change. Fillets can take a while on complex input.
