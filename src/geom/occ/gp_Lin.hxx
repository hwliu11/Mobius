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

#ifndef _gp_Lin_HeaderFile
#define _gp_Lin_HeaderFile

#include <mobius/gp_Ax1.hxx>

namespace mobius {
namespace occ {

class gp_Pnt;
class gp_Dir;
class gp_Ax2;
class gp_Trsf;
class gp_Vec;

//! Describes a line in 3D space.
//! A line is positioned in space with an axis (a gp_Ax1
//! object) which gives it an origin and a unit vector.
//! A line and an axis are similar objects, thus, we can
//! convert one into the other. A line provides direct access
//! to the majority of the edit and query functions available
//! on its positioning axis. In addition, however, a line has
//! specific functions for computing distances and positions.
//! See Also
//! gce_MakeLin which provides functions for more complex
//! line constructions
//! Geom_Line which provides additional functions for
//! constructing lines and works, in particular, with the
//! parametric equations of lines
class gp_Lin 
{
public:

  //! Creates a Line corresponding to Z axis of the
  //! reference coordinate system.
  gp_Lin() {}

  //! Creates a line defined by axis A1.
  gp_Lin(const gp_Ax1& A1)
    : pos(A1) {}

  //! Creates a line passing through point P and parallel to
  //! vector V (P and V are, respectively, the origin and
  //! the unit vector of the positioning axis of the line).
  gp_Lin(const gp_Pnt& P,
         const gp_Dir& V)
    : pos(P, V) {}

  void Reverse()
  {
    pos.Reverse();
  }

  //! Reverses the direction of the line.
  //! Note:
  //! -   Reverse assigns the result to this line, while
  //! -   Reversed creates a new one.
  gp_Lin Reversed() const
  {
    gp_Lin L = *this;
    L.pos.Reverse();
    return L;
  }

  //! Changes the direction of the line.
  void SetDirection(const gp_Dir& V)
  {
    pos.SetDirection(V);
  }

  //! Changes the location point (origin) of the line.
  void SetLocation(const gp_Pnt& P)
  {
    pos.SetLocation(P);
  }

  //! Complete redefinition of the line.
  //! The "Location" point of <A1> is the origin of the line.
  //! The "Direction" of <A1> is  the direction of the line.
  void SetPosition(const gp_Ax1& A1)
  {
    pos = A1;
  }

  //! Returns the direction of the line.
  const gp_Dir& Direction() const
  {
    return pos.Direction();
  }

  //! Returns the location point (origin) of the line.
  const gp_Pnt& Location()  const
  {
    return pos.Location();
  }

  //! Returns the axis placement one axis whith the same
  //! location and direction as <me>.
  const gp_Ax1& Position() const
  {
    return pos;
  }

  //! Computes the angle between two lines in radians.
  Standard_Real Angle(const gp_Lin& Other) const
  {
    return pos.Direction().Angle(Other.pos.Direction());
  }

  //! Returns true if this line contains the point P, that is, if the
  //! distance between point P and this line is less than or
  //! equal to LinearTolerance.
  Standard_Boolean Contains(const gp_Pnt& P,
                            const Standard_Real LinearTolerance) const
  {
    return Distance(P) <= LinearTolerance;
  }

  //! Computes the distance between <me> and the point P.
  Standard_Real Distance(const gp_Pnt& P) const
  {
    gp_XYZ Coord = P.XYZ();
    Coord.Subtract((pos.Location()).XYZ());
    Coord.Cross((pos.Direction()).XYZ());
    return Coord.Modulus();
  }

  //! Computes the square distance between <me> and the point P.
  mobiusGeom_EXPORT Standard_Real SquareDistance(const gp_Pnt& P) const;

  //! Computes the square distance between two lines.
  Standard_Real SquareDistance(const gp_Lin& Other) const
  {
    Standard_Real D = Distance(Other);
    return D * D;
  }

  //! Computes the line normal to the direction of <me>, passing
  //! through the point P.  Raises ConstructionError
  //! if the distance between <me> and the point P is lower
  //! or equal to Resolution from gp because there is an infinity of
  //! solutions in 3D space.
  gp_Lin Normal(const gp_Pnt& P) const
  {
    const gp_Pnt& Loc = pos.Location();
    gp_Dir V(P.X() - Loc.X(),
      P.Y() - Loc.Y(),
      P.Z() - Loc.Z());
    V = pos.Direction().CrossCrossed(V, pos.Direction());

    return gp_Lin(P, V);
  }

  void Rotate(const gp_Ax1& A1,
              const Standard_Real Ang)
  {
    pos.Rotate(A1, Ang);
  }

  //! Rotates a line. A1 is the axis of the rotation.
  //! Ang is the angular value of the rotation in radians.
  gp_Lin Rotated(const gp_Ax1& A1,
                 const Standard_Real Ang) const
  {
    gp_Lin L = *this;
    L.pos.Rotate(A1, Ang);
    return L;
  }

  void Scale(const gp_Pnt& P,
             const Standard_Real S)
  {
    pos.Scale(P, S);
  }

  //! Scales a line. S is the scaling value.
  //! The "Location" point (origin) of the line is modified.
  //! The "Direction" is reversed if the scale is negative
  gp_Lin Scaled(const gp_Pnt& P,
                const Standard_Real S) const
  {
    gp_Lin L = *this;
    L.pos.Scale(P, S);
    return L;
  }

  void Transform(const gp_Trsf& T)
  {
    pos.Transform(T);
  }

  //! Transforms a line with the transformation T from class Trsf.
  gp_Lin Transformed(const gp_Trsf& T) const
  {
    gp_Lin L = *this;
    L.pos.Transform(T);

    return L;
  }

  void Translate(const gp_Vec& V)
  {
    pos.Translate(V);
  }

  //! Translates a line in the direction of the vector V.
  //! The magnitude of the translation is the vector's magnitude.
  gp_Lin Translated(const gp_Vec& V) const
  {
    gp_Lin L = *this;
    L.pos.Translate(V);
    return L;
  }

  void Translate(const gp_Pnt& P1,
                 const gp_Pnt& P2)
  {
    pos.Translate(P1, P2);
  }

  //! Translates a line from the point P1 to the point P2.
  mobiusGeom_EXPORT gp_Lin Translated(const gp_Pnt& P1,
                            const gp_Pnt& P2) const;

  //! Computes the distance between two lines.
  mobiusGeom_EXPORT Standard_Real Distance (const gp_Lin& Other) const;

  mobiusGeom_EXPORT void Mirror (const gp_Pnt& P);

  //! Performs the symmetrical transformation of a line
  //! with respect to the point P which is the center of
  //! the symmetry.
  Standard_NODISCARD mobiusGeom_EXPORT gp_Lin Mirrored (const gp_Pnt& P) const;
  
  mobiusGeom_EXPORT void Mirror (const gp_Ax1& A1);
  
  //! Performs the symmetrical transformation of a line
  //! with respect to an axis placement which is the axis
  //! of the symmetry.
  Standard_NODISCARD mobiusGeom_EXPORT gp_Lin Mirrored (const gp_Ax1& A1) const;
  
  mobiusGeom_EXPORT void Mirror (const gp_Ax2& A2);
  
  //! Performs the symmetrical transformation of a line
  //! with respect to a plane. The axis placement  <A2>
  //! locates the plane of the symmetry :
  //! (Location, XDirection, YDirection).
  Standard_NODISCARD mobiusGeom_EXPORT gp_Lin Mirrored (const gp_Ax2& A2) const;

private:

  gp_Ax1 pos;
};

}
}

#endif // _gp_Lin_HeaderFile
