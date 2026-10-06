# Alpha Shapes SOP
Use Alpha Shapes algorithm calculation for input SOP points and output resulting mesh with triangle faces. Using CGAL 6.2.1, Computational Geometry Algorithms Library, https://www.cgal.org. To be able to build this sop you have to install CGAL library.

To build on Windows: Use [vcpkg](https://vcpkg.io/), more info on [CGAL installation manual](https://doc.cgal.org/latest/Manual/windows.html).

CGAL must be installed for the `x64-windows-static-md` triplet (`vcpkg install cgal:x64-windows-static-md`), not the default `x64-windows`. This statically links CGAL's dependencies (GMP, MPFR, etc.) directly into `AlphaShapesSOP.dll` so no extra DLLs (e.g. `gmp-10.dll`) need to be shipped alongside it.

To build on MacOS (ARM only): *brew install cgal*


## Parameters
* **Mode** - select between Regularized (default) and General mode.
  * **Regularized** - outputs only the surface triangles of the solid part of the shape.
  * **General** - additionally outputs dangling triangles, dangling edges (as line primitives) and isolated points that belong to the alpha complex but are not part of any solid.
* **Use optimal Alpha** - Use optimal Alpha value that is automatically calculated.
* **Alpha** - Set manual alpha (α) value for calculation.
* **Skip interior points** - Skip points that are mesh interior and not used for triangle faces.
* **Normals** - select how point normals (`N`) are output.
  * **Off** - no normals are output.
  * **Smooth** (default) - triangles share points, and each point gets the average of the normals of its triangles, weighted by the triangle angle at that point (like Attribute Create SOP > Compute Normals). Points without triangles (e.g. isolated points or line-only points in General mode) get (0, 0, 1).
  * **Flat** - every triangle gets its own 3 points, so all points of a triangle share the face normal and the mesh is shaded faceted. This increases the output point count.
