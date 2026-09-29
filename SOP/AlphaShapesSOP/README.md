# Alpha Shapes SOP
Use Alpha Shapes algorithm calculation for input SOP points and output resulting mesh with triangle faces. Using CGAL 6.2.1, Computational Geometry Algorithms Library, https://www.cgal.org. To be able to build this sop you have to install CGAL library. The easiest way is to use [vcpkg](https://vcpkg.io/), more info on [CGAL installation manual](https://doc.cgal.org/latest/Manual/windows.html).

CGAL must be installed for the `x64-windows-static-md` triplet (`vcpkg install cgal:x64-windows-static-md`), not the default `x64-windows`. This statically links CGAL's dependencies (GMP, MPFR, etc.) directly into `AlphaShapesSOP.dll` so no extra DLLs (e.g. `gmp-10.dll`) need to be shipped alongside it.

[CGAL 6.2 - 3D Alpha Shapes User Manual](https://doc.cgal.org/6.2/Alpha_shapes_3/index.html)



## Parameters
* **Mode** - select between Regularized (default) and General mode.
* **Use optimal Alpha** - Use optimal Alpha value that is automatically calculated.
* **Alpha** - Set manual alpha (α) value for calculation.
* **Skip interior points** - Skip points that are mesh interior and not used for triangle faces.
