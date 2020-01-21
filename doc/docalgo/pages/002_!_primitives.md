Primitives {#primitives}
========================

In Mobius, all primitives are defined as C++ classes. The class names are prefixed with the corresponding
package name, e.g., geom_BSplineSurface or core_XYZ. There are also short names prefixed with "t_", e.g.,
t_xyz, t_bsurf, etc. We use "t_" prefix instead of a more common C-like "_t" suffix in order to avoid any
collisions with standard C/C++ typenames and POSIX structures.

-# \subpage bsurf
-# \subpage smesh
