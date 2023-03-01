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

#ifndef _gp_Dir2d_HeaderFile
#define _gp_Dir2d_HeaderFile

#include <mobius/gp_XY.hxx>

namespace mobius {
namespace occ {

class gp_Vec2d;
class gp_XY;
class gp_Ax2d;
class gp_Trsf2d;

//! Describes a unit vector in the plane (2D space).
class gp_Dir2d 
{
public:

  //! Creates a direction corresponding to X axis.
  gp_Dir2d()
  : coord (1., 0.)
  {}

  //! Normalizes the vector theV and creates a Direction. Raises ConstructionError if theV.Magnitude() <= Resolution from gp.
  mobiusGeom_EXPORT gp_Dir2d (const gp_Vec2d& theV);

  //! Creates a Direction from a doublet of coordinates. Raises ConstructionError if theCoord.Modulus() <= Resolution from gp.
  mobiusGeom_EXPORT gp_Dir2d (const gp_XY& theCoord);

  //! Creates a Direction with its 2 cartesian coordinates. Raises ConstructionError if Sqrt(theXv*theXv + theYv*theYv) <= Resolution from gp.
  mobiusGeom_EXPORT gp_Dir2d (const double theXv, const double theYv);

  //! For this unit vector, assigns:
  //! the value theXi to:
  //! -   the X coordinate if theIndex is 1, or
  //! -   the Y coordinate if theIndex is 2, and then normalizes it.
  //! Warning
  //! Remember that all the coordinates of a unit vector are
  //! implicitly modified when any single one is changed directly.
  //! Exceptions
  //! Standard_OutOfRange if theIndex is not 1 or 2.
  //! Standard_ConstructionError if either of the following
  //! is less than or equal to gp::Resolution():
  //! -   Sqrt(theXv*theXv + theYv*theYv), or
  //! -   the modulus of the number pair formed by the new
  //! value theXi and the other coordinate of this vector that
  //! was not directly modified.
  //! Raises OutOfRange if theIndex != {1, 2}.
  mobiusGeom_EXPORT void SetCoord (const int theIndex, const double theXi);

  //! For this unit vector, assigns:
  //! -   the values theXv and theYv to its two coordinates,
  //! Warning
  //! Remember that all the coordinates of a unit vector are
  //! implicitly modified when any single one is changed directly.
  //! Exceptions
  //! Standard_OutOfRange if theIndex is not 1 or 2.
  //! Standard_ConstructionError if either of the following
  //! is less than or equal to gp::Resolution():
  //! -   Sqrt(theXv*theXv + theYv*theYv), or
  //! -   the modulus of the number pair formed by the new
  //! value Xi and the other coordinate of this vector that
  //! was not directly modified.
  //! Raises OutOfRange if theIndex != {1, 2}.
  mobiusGeom_EXPORT void SetCoord (const double theXv, const double theYv);

  //! Assigns the given value to the X coordinate of this unit   vector,
  //! and then normalizes it.
  //! Warning
  //! Remember that all the coordinates of a unit vector are
  //! implicitly modified when any single one is changed directly.
  //! Exceptions
  //! Standard_ConstructionError if either of the following
  //! is less than or equal to gp::Resolution():
  //! -   the modulus of Coord, or
  //! -   the modulus of the number pair formed from the new
  //! X or Y coordinate and the other coordinate of this
  //! vector that was not directly modified.
  mobiusGeom_EXPORT void SetX (const double theX);

  //! Assigns  the given value to the Y coordinate of this unit   vector,
  //! and then normalizes it.
  //! Warning
  //! Remember that all the coordinates of a unit vector are
  //! implicitly modified when any single one is changed directly.
  //! Exceptions
  //! Standard_ConstructionError if either of the following
  //! is less than or equal to gp::Resolution():
  //! -   the modulus of Coord, or
  //! -   the modulus of the number pair formed from the new
  //! X or Y coordinate and the other coordinate of this
  //! vector that was not directly modified.
  mobiusGeom_EXPORT void SetY (const double theY);

  //! Assigns:
  //! -   the two coordinates of theCoord to this unit vector,
  //! and then normalizes it.
  //! Warning
  //! Remember that all the coordinates of a unit vector are
  //! implicitly modified when any single one is changed directly.
  //! Exceptions
  //! Standard_ConstructionError if either of the following
  //! is less than or equal to gp::Resolution():
  //! -   the modulus of theCoord, or
  //! -   the modulus of the number pair formed from the new
  //! X or Y coordinate and the other coordinate of this
  //! vector that was not directly modified.
  mobiusGeom_EXPORT void SetXY (const gp_XY& theCoord);

  //! For this unit vector returns the coordinate of range theIndex :
  //! theIndex = 1 => X is returned
  //! theIndex = 2 => Y is returned
  //! Raises OutOfRange if theIndex != {1, 2}.
  double Coord (const int theIndex) const  { return coord.Coord (theIndex); }

  //! For this unit vector returns its two coordinates theXv and theYv.
  //! Raises OutOfRange if theIndex != {1, 2}.
  void Coord (double& theXv, double& theYv) const  { coord.Coord (theXv, theYv); }

  //! For this unit vector, returns its X coordinate.
  double X() const { return coord.X(); }

  //! For this unit vector, returns its Y coordinate.
  double Y() const { return coord.Y(); }

  //! For this unit vector, returns its two coordinates as a number pair.
  //! Comparison between Directions
  //! The precision value is an input data.
  const gp_XY& XY() const { return coord; }

  //! Returns True if the two vectors have the same direction
  //! i.e. the angle between this unit vector and the
  //! unit vector theOther is less than or equal to theAngularTolerance.
  mobiusGeom_EXPORT bool IsEqual (const gp_Dir2d& theOther, const double theAngularTolerance) const;

  //! Returns True if the angle between this unit vector and the
  //! unit vector theOther is equal to Pi/2 or -Pi/2 (normal)
  //! i.e. Abs(Abs(<me>.Angle(theOther)) - PI/2.) <= theAngularTolerance
  mobiusGeom_EXPORT bool IsNormal (const gp_Dir2d& theOther, const double theAngularTolerance) const;

  //! Returns True if the angle between this unit vector and the
  //! unit vector theOther is equal to Pi or -Pi (opposite).
  //! i.e.  PI - Abs(<me>.Angle(theOther)) <= theAngularTolerance
  mobiusGeom_EXPORT bool IsOpposite (const gp_Dir2d& theOther, const double theAngularTolerance) const;

  //! returns true if the angle between this unit vector and unit
  //! vector theOther is equal to 0, Pi or -Pi.
  //! i.e.  Abs(Angle(<me>, theOther)) <= theAngularTolerance or
  //! PI - Abs(Angle(<me>, theOther)) <= theAngularTolerance
  mobiusGeom_EXPORT bool IsParallel (const gp_Dir2d& theOther, const double theAngularTolerance) const;

  //! Computes the angular value in radians between <me> and
  //! <theOther>. Returns the angle in the range [-PI, PI].
  mobiusGeom_EXPORT double Angle (const gp_Dir2d& theOther) const;

  //! Computes the cross product between two directions.
  mobiusCore_NODISCARD double Crossed (const gp_Dir2d& theRight) const { return coord.Crossed (theRight.coord); }

  mobiusCore_NODISCARD double operator ^ (const gp_Dir2d& theRight) const { return Crossed (theRight); }

  //! Computes the scalar product
  double Dot (const gp_Dir2d& theOther) const { return coord.Dot (theOther.coord); }

  double operator * (const gp_Dir2d& theOther) const { return Dot (theOther); }

  void Reverse() { coord.Reverse(); }

  //! Reverses the orientation of a direction
  mobiusCore_NODISCARD gp_Dir2d Reversed() const
  {
    gp_Dir2d aV = *this;
    aV.coord.Reverse();
    return aV;
  }

  mobiusCore_NODISCARD gp_Dir2d operator -() const { return Reversed(); }

  mobiusGeom_EXPORT void Mirror (const gp_Dir2d& theV);

  //! Performs the symmetrical transformation of a direction
  //! with respect to the direction theV which is the center of
  //! the  symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Dir2d Mirrored (const gp_Dir2d& theV) const;

  mobiusGeom_EXPORT void Mirror (const gp_Ax2d& theA);

  //! Performs the symmetrical transformation of a direction
  //! with respect to an axis placement which is the axis
  //! of the symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Dir2d Mirrored (const gp_Ax2d& theA) const;

  mobiusGeom_EXPORT void Rotate (const double Ang);

  //! Rotates a direction.  theAng is the angular value of
  //! the rotation in radians.
  mobiusCore_NODISCARD gp_Dir2d Rotated (const double theAng) const
  {
    gp_Dir2d aV = *this;
    aV.Rotate (theAng);
    return aV;
  }

  mobiusGeom_EXPORT void Transform (const gp_Trsf2d& theT);

  //! Transforms a direction with the "Trsf" theT.
  //! Warnings :
  //! If the scale factor of the "Trsf" theT is negative then the
  //! direction <me> is reversed.
  mobiusCore_NODISCARD gp_Dir2d Transformed (const gp_Trsf2d& theT) const
  {
    gp_Dir2d aV = *this;
    aV.Transform (theT);
    return aV;
  }

  //! Dumps the content of me into the stream
  mobiusGeom_EXPORT void DumpJson (std::ostream& theOStream, int theDepth = -1) const;

private:

  gp_XY coord;

};


}
}

#endif // _gp_Dir2d_HeaderFile
