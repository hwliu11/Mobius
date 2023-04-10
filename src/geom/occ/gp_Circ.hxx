// Copyright (c) 1991-1999 Matra Datavision
// Copyright (c) 1999-2014 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#ifndef _gp_Circ_HeaderFile
#define _gp_Circ_HeaderFile

#include <mobius/gp_Ax2.hxx>
#include <mobius/Standard_Real.hxx>
#include <mobius/Standard_Boolean.hxx>

namespace mobius {
namespace occ {

class gp_Ax1;
class gp_Pnt;
class gp_Trsf;
class gp_Vec;

//! Describes a circle in 3D space.
//! A circle is defined by its radius and positioned in space
//! with a coordinate system (a gp_Ax2 object) as follows:
//! -   the origin of the coordinate system is the center of the circle, and
//! -   the origin, "X Direction" and "Y Direction" of the
//! coordinate system define the plane of the circle.
//! This positioning coordinate system is the "local
//! coordinate system" of the circle. Its "main Direction"
//! gives the normal vector to the plane of the circle. The
//! "main Axis" of the coordinate system is referred to as
//! the "Axis" of the circle.
//! Note: when a gp_Circ circle is converted into a
//! Geom_Circle circle, some implicit properties of the
//! circle are used explicitly:
//! -   the "main Direction" of the local coordinate system
//! gives an implicit orientation to the circle (and defines
//! its trigonometric sense),
//! -   this orientation corresponds to the direction in
//! which parameter values increase,
//! -   the starting point for parameterization is that of the
//! "X Axis" of the local coordinate system (i.e. the "X Axis" of the circle).
//! See Also
//! gce_MakeCirc which provides functions for more complex circle constructions
//! Geom_Circle which provides additional functions for
//! constructing circles and works, in particular, with the
//! parametric equations of circles
class gp_Circ 
{
public:

  //! Creates an indefinite circle.
  gp_Circ() : radius(RealLast()) {}

  //! A2 locates the circle and gives its orientation in 3D space.
  gp_Circ(const gp_Ax2& A2,
          const Standard_Real R)
    : pos(A2), radius(R) {}

  //! Changes the main axis of the circle. It is the axis
  //! perpendicular to the plane of the circle.
  void SetAxis(const gp_Ax1& A1)
  {
    pos.SetAxis(A1);
  }

  //! Changes the "Location" point (center) of the circle.
  void SetLocation(const gp_Pnt& P)
  {
    pos.SetLocation(P);
  }

  //! Changes the position of the circle.
  void SetPosition(const gp_Ax2& A2)
  {
    pos = A2;
  }

  //! Modifies the radius of this circle.
  void SetRadius(const Standard_Real R)
  {
    radius = R;
  }

  //! Computes the area of the circle.
  Standard_Real Area() const
  {
    return M_PI * radius * radius;
  }

  //! Returns the main axis of the circle.
  //! It is the axis perpendicular to the plane of the circle,
  //! passing through the "Location" point (center) of the circle.
  const gp_Ax1& Axis() const
  {
    return pos.Axis();
  }

  //! Computes the circumference of the circle.
  Standard_Real Length() const
  {
    return 2. * M_PI * radius;
  }

  //! Returns the center of the circle. It is the
  //! "Location" point of the local coordinate system
  //! of the circle
  const gp_Pnt& Location() const
  {
    return pos.Location();
  }

  //! Returns the position of the circle.
  //! It is the local coordinate system of the circle.
  const gp_Ax2& Position() const
  {
    return pos;
  }

  //! Returns the radius of this circle.
  Standard_Real Radius() const
  {
    return radius;
  }

  //! Returns the "XAxis" of the circle.
  //! This axis is perpendicular to the axis of the conic.
  //! This axis and the "Yaxis" define the plane of the conic.
  gp_Ax1 XAxis() const
  {
    return gp_Ax1(pos.Location(), pos.XDirection());
  }

  //! Returns the "YAxis" of the circle.
  //! This axis and the "Xaxis" define the plane of the conic.
  //! The "YAxis" is perpendicular to the "Xaxis".
  gp_Ax1 YAxis() const
  {
    return gp_Ax1(pos.Location(), pos.YDirection());
  }

  //! Computes the minimum of distance between the point P and
  //! any point on the circumference of the circle.
  Standard_Real Distance(const gp_Pnt& P) const
  {
    return sqrt(SquareDistance(P));
  }

  //! Computes the square distance between <me> and the point P.
  mobiusGeom_EXPORT Standard_Real SquareDistance(const gp_Pnt& P) const;

  //! Returns True if the point P is on the circumference.
  //! The distance between <me> and <P> must be lower or
  //! equal to LinearTolerance.
  Standard_Boolean Contains (const gp_Pnt& P,
                             const Standard_Real LinearTolerance) const
  {
    return Distance(P) <= LinearTolerance;
  }

  void Rotate(const gp_Ax1& A1,
              const Standard_Real Ang)
  {
    pos.Rotate(A1, Ang);
  }

  //! Rotates a circle. A1 is the axis of the rotation.
  //! Ang is the angular value of the rotation in radians.
  gp_Circ Rotated(const gp_Ax1& A1,
                  const Standard_Real Ang) const
  {
    gp_Circ C = *this;
    C.pos.Rotate(A1, Ang);
    return C;
  }

  void Scale(const gp_Pnt& P,
             const Standard_Real S)
  {
    radius *= S;
    if (radius < 0) radius = -radius;
    pos.Scale(P, S);
  }

  //! Scales a circle. S is the scaling value.
  //! Warnings :
  //! If S is negative the radius stay positive but
  //! the "XAxis" and the "YAxis" are  reversed as for
  //! an ellipse.
  gp_Circ Scaled(const gp_Pnt& P,
                 const Standard_Real S) const
  {
    gp_Circ C = *this;
    C.radius *= S;
    if (C.radius < 0) C.radius = -C.radius;
    C.pos.Scale(P, S);
    return C;
  }

  void Transform(const gp_Trsf& T)
  {
    radius *= T.ScaleFactor();
    if (radius < 0) radius = -radius;
    pos.Transform(T);
  }

  //! Transforms a circle with the transformation T from class Trsf.
  gp_Circ Transformed(const gp_Trsf& T) const
  {
    gp_Circ C = *this;
    C.radius *= T.ScaleFactor();
    if (C.radius < 0) C.radius = -C.radius;
    C.pos.Transform(T);
    return C;
  }

  void Translate(const gp_Vec& V)
  {
    pos.Translate(V);
  }

  gp_Circ Translated(const gp_Vec& V) const
  {
    gp_Circ C = *this;
    C.pos.Translate(V);
    return C;
  }

  //! Translates a circle in the direction of the vector V.
  //! The magnitude of the translation is the vector's magnitude.
  void Translate(const gp_Pnt& P1,
                 const gp_Pnt& P2)
  {
    pos.Translate(P1, P2);
  }

  //! Translates a circle in the direction of the vector V.
  //! The magnitude of the translation is the vector's magnitude.
  gp_Circ Translated(const gp_Pnt& P1,
                     const gp_Pnt& P2) const
  {
    gp_Circ C = *this;
    C.pos.Translate(P1, P2);
    return C;
  }

  mobiusGeom_EXPORT void Mirror (const gp_Pnt& P);  

  //! Performs the symmetrical transformation of a circle
  //! with respect to the point P which is the center of the
  //! symmetry.
  Standard_NODISCARD mobiusGeom_EXPORT gp_Circ Mirrored (const gp_Pnt& P) const;
  
  mobiusGeom_EXPORT void Mirror (const gp_Ax1& A1);  

  //! Performs the symmetrical transformation of a circle with
  //! respect to an axis placement which is the axis of the
  //! symmetry.
  Standard_NODISCARD mobiusGeom_EXPORT gp_Circ Mirrored (const gp_Ax1& A1) const;
  
  mobiusGeom_EXPORT void Mirror (const gp_Ax2& A2);  

  //! Performs the symmetrical transformation of a circle with respect
  //! to a plane. The axis placement A2 locates the plane of the
  //! of the symmetry : (Location, XDirection, YDirection).
  Standard_NODISCARD mobiusGeom_EXPORT gp_Circ Mirrored (const gp_Ax2& A2) const;

private:

  gp_Ax2 pos;
  Standard_Real radius;
};

}
}

#endif // _gp_Circ_HeaderFile
