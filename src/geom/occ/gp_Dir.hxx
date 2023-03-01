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

#ifndef _gp_Dir_HeaderFile
#define _gp_Dir_HeaderFile

#include <mobius/gp_XYZ.hxx>
#include <mobius/gp_Trsf.hxx>
#include <mobius/occMathDefs.hxx>

namespace mobius {
namespace occ {

class gp_Vec;
class gp_Ax1;
class gp_Ax2;
class gp_Trsf;

//! Describes a unit vector in 3D space.
class gp_Dir
{
public:

  //! Creates a direction corresponding to X axis.
  gp_Dir()
  : coord (1., 0., 0.)
  {}

  //! Normalizes the vector theV and creates a direction. Raises ConstructionError if theV.Magnitude() <= Resolution.
  mobiusGeom_EXPORT gp_Dir (const gp_Vec& theV);

  //! Creates a direction from a triplet of coordinates. Raises ConstructionError if theCoord.Modulus() <= Resolution from gp.
  mobiusGeom_EXPORT gp_Dir (const gp_XYZ& theCoord);

  //! Creates a direction with its 3 cartesian coordinates. Raises ConstructionError if Sqrt(theXv*theXv + theYv*theYv + theZv*theZv) <= Resolution
  //! Modification of the direction's coordinates
  //! If Sqrt (theXv*theXv + theYv*theYv + theZv*theZv) <= Resolution from gp where
  //! theXv, theYv ,theZv are the new coordinates it is not possible to
  //! construct the direction and the method raises the
  //! exception ConstructionError.
  mobiusGeom_EXPORT gp_Dir (const double theXv, const double theYv, const double theZv);

  //! For this unit vector,  assigns the value Xi to:
  //! -   the X coordinate if theIndex is 1, or
  //! -   the Y coordinate if theIndex is 2, or
  //! -   the Z coordinate if theIndex is 3,
  //! and then normalizes it.
  //! Warning
  //! Remember that all the coordinates of a unit vector are
  //! implicitly modified when any single one is changed directly.
  //! Exceptions
  //! Standard_OutOfRange if theIndex is not 1, 2, or 3.
  //! Standard_ConstructionError if either of the following
  //! is less than or equal to gp::Resolution():
  //! -   Sqrt(Xv*Xv + Yv*Yv + Zv*Zv), or
  //! -   the modulus of the number triple formed by the new
  //! value theXi and the two other coordinates of this vector
  //! that were not directly modified.
  void SetCoord (const int theIndex, const double theXi);

  //! For this unit vector,  assigns the values theXv, theYv and theZv to its three coordinates.
  //! Remember that all the coordinates of a unit vector are
  //! implicitly modified when any single one is changed directly.
  void SetCoord (const double theXv, const double theYv, const double theZv);

  //! Assigns the given value to the X coordinate of this   unit vector.
  void SetX (const double theX);

  //! Assigns the given value to the Y coordinate of this   unit vector.
  void SetY (const double theY);

  //! Assigns the given value to the Z  coordinate of this   unit vector.
  void SetZ (const double theZ);

  //! Assigns the three coordinates of theCoord to this unit vector.
  void SetXYZ (const gp_XYZ& theCoord);

  //! Returns the coordinate of range theIndex :
  //! theIndex = 1 => X is returned
  //! Ithendex = 2 => Y is returned
  //! theIndex = 3 => Z is returned
  //! Exceptions
  //! Standard_OutOfRange if theIndex is not 1, 2, or 3.
  double Coord (const int theIndex) const  { return coord.Coord (theIndex); }

  //! Returns for the  unit vector  its three coordinates theXv, theYv, and theZv.
  void Coord (double& theXv, double& theYv, double& theZv) const  { coord.Coord (theXv, theYv, theZv); }

  //! Returns the X coordinate for a  unit vector.
  double X() const { return coord.X(); }

  //! Returns the Y coordinate for a  unit vector.
  double Y() const { return coord.Y(); }

  //! Returns the Z coordinate for a  unit vector.
  double Z() const { return coord.Z(); }

  //! for this unit vector, returns  its three coordinates as a number triplea.
  const gp_XYZ& XYZ() const { return coord; }

  //! Returns True if the angle between the two directions is
  //! lower or equal to theAngularTolerance.
  bool IsEqual (const gp_Dir& theOther, const double theAngularTolerance) const
  {
    return Angle (theOther) <= theAngularTolerance;
  }

  //! Returns True if  the angle between this unit vector and the unit vector theOther is equal to Pi/2 (normal).
  bool IsNormal (const gp_Dir& theOther, const double theAngularTolerance) const
  {
    double anAng = M_PI / 2.0 - Angle (theOther);
    if (anAng < 0)
    {
      anAng = -anAng;
    }
    return anAng <= theAngularTolerance;
  }

  //! Returns True if  the angle between this unit vector and the unit vector theOther is equal to  Pi (opposite).
  bool IsOpposite (const gp_Dir& theOther, const double theAngularTolerance) const
  {
    return M_PI - Angle (theOther) <= theAngularTolerance;
  }

  //! Returns true if the angle between this unit vector and the
  //! unit vector theOther is equal to 0 or to Pi.
  //! Note: the tolerance criterion is given by theAngularTolerance.
  bool IsParallel (const gp_Dir& theOther, const double theAngularTolerance) const
  {
    double anAng = Angle (theOther);
    return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
  }

  //! Computes the angular value in radians between <me> and
  //! <theOther>. This value is always positive in 3D space.
  //! Returns the angle in the range [0, PI]
  mobiusGeom_EXPORT double Angle (const gp_Dir& theOther) const;

  //! Computes the angular value between <me> and <theOther>.
  //! <theVRef> is the direction of reference normal to <me> and <theOther>
  //! and its orientation gives the positive sense of rotation.
  //! If the cross product <me> ^ <theOther> has the same orientation
  //! as <theVRef> the angular value is positive else negative.
  //! Returns the angular value in the range -PI and PI (in radians). Raises  DomainError if <me> and <theOther> are not parallel this exception is raised
  //! when <theVRef> is in the same plane as <me> and <theOther>
  //! The tolerance criterion is Resolution from package gp.
  mobiusGeom_EXPORT double AngleWithRef (const gp_Dir& theOther, const gp_Dir& theVRef) const;

  //! Computes the cross product between two directions
  //! Raises the exception ConstructionError if the two directions
  //! are parallel because the computed vector cannot be normalized
  //! to create a direction.
  void Cross (const gp_Dir& theRight);

  void operator ^= (const gp_Dir& theRight) { Cross (theRight); }

  //! Computes the triple vector product.
  //! <me> ^ (V1 ^ V2)
  //! Raises the exception ConstructionError if V1 and V2 are parallel
  //! or <me> and (V1^V2) are parallel because the computed vector
  //! can't be normalized to create a direction.
  mobiusCore_NODISCARD gp_Dir Crossed (const gp_Dir& theRight) const;

  mobiusCore_NODISCARD gp_Dir operator ^ (const gp_Dir& theRight) const { return Crossed (theRight); }

  void CrossCross (const gp_Dir& theV1, const gp_Dir& theV2);

  //! Computes the double vector product this ^ (theV1 ^ theV2).
  //! -   CrossCrossed creates a new unit vector.
  //! Exceptions
  //! Standard_ConstructionError if:
  //! -   theV1 and theV2 are parallel, or
  //! -   this unit vector and (theV1 ^ theV2) are parallel.
  //! This is because, in these conditions, the computed vector
  //! is null and cannot be normalized.
  mobiusCore_NODISCARD gp_Dir CrossCrossed (const gp_Dir& theV1, const gp_Dir& theV2) const;

  //! Computes the scalar product
  double Dot (const gp_Dir& theOther) const { return coord.Dot (theOther.coord); }

  double operator * (const gp_Dir& theOther) const { return Dot (theOther); }

  //! Computes the triple scalar product <me> * (theV1 ^ theV2).
  //! Warnings :
  //! The computed vector theV1' = theV1 ^ theV2 is not normalized
  //! to create a unitary vector. So this method never
  //! raises an exception even if theV1 and theV2 are parallel.
  double DotCross (const gp_Dir& theV1, const gp_Dir& theV2) const
  {
    return coord.Dot (theV1.coord.Crossed (theV2.coord));
  }

  void Reverse() { coord.Reverse(); }

  //! Reverses the orientation of a direction
  //! geometric transformations
  //! Performs the symmetrical transformation of a direction
  //! with respect to the direction V which is the center of
  //! the  symmetry.]
  mobiusCore_NODISCARD gp_Dir Reversed() const
  {
    gp_Dir aV = *this;
    aV.coord.Reverse();
    return aV;
  }

  mobiusCore_NODISCARD gp_Dir operator -() const { return Reversed(); }

  mobiusGeom_EXPORT void Mirror (const gp_Dir& theV);

  //! Performs the symmetrical transformation of a direction
  //! with respect to the direction theV which is the center of
  //! the  symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Dir Mirrored (const gp_Dir& theV) const;

  mobiusGeom_EXPORT void Mirror (const gp_Ax1& theA1);

  //! Performs the symmetrical transformation of a direction
  //! with respect to an axis placement which is the axis
  //! of the symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Dir Mirrored (const gp_Ax1& theA1) const;

  mobiusGeom_EXPORT void Mirror (const gp_Ax2& theA2);

  //! Performs the symmetrical transformation of a direction
  //! with respect to a plane. The axis placement theA2 locates
  //! the plane of the symmetry : (Location, XDirection, YDirection).
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Dir Mirrored (const gp_Ax2& theA2) const;

  void Rotate(const gp_Ax1& theA1, const double theAng);

  //! Rotates a direction. theA1 is the axis of the rotation.
  //! theAng is the angular value of the rotation in radians.
  mobiusCore_NODISCARD gp_Dir Rotated (const gp_Ax1& theA1, const double theAng) const
  {
    gp_Dir aV = *this;
    aV.Rotate (theA1, theAng);
    return aV;
  }

  mobiusGeom_EXPORT void Transform (const gp_Trsf& theT);

  //! Transforms a direction with a "Trsf" from gp.
  //! Warnings :
  //! If the scale factor of the "Trsf" theT is negative then the
  //! direction <me> is reversed.
  mobiusCore_NODISCARD gp_Dir Transformed (const gp_Trsf& theT) const
  {
    gp_Dir aV = *this;
    aV.Transform (theT);
    return aV;
  }

private:

  gp_XYZ coord;

};

}
}

#endif // _gp_Dir_HeaderFile
