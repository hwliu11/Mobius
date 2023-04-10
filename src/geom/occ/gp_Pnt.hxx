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

#ifndef _gp_Pnt_HeaderFile
#define _gp_Pnt_HeaderFile

#include <mobius/geom.h>

#include <mobius/gp_XYZ.hxx>
#include <mobius/gp_Trsf.hxx>
#include <mobius/gp_XYZ.hxx>

namespace mobius {
namespace occ {

class gp_XYZ;
class Standard_OutOfRange;
class gp_Ax1;
class gp_Ax2;
class gp_Trsf;
class gp_Vec;

//! Defines a 3D Cartesian point.
class gp_Pnt
{
public:

  //! Creates a point with zero coordinates.
  gp_Pnt() {}

  //! Creates a point from a XYZ object.
  gp_Pnt (const gp_XYZ& theCoord)
  : coord (theCoord)
  {}

  //! Creates a  point with its 3 cartesian's coordinates : theXp, theYp, theZp.
  gp_Pnt (const double theXp, const double theYp, const double theZp)
  : coord (theXp, theYp, theZp)
  {}

  //! Changes the coordinate of range theIndex :
  //! theIndex = 1 => X is modified
  //! theIndex = 2 => Y is modified
  //! theIndex = 3 => Z is modified
  //! Raised if theIndex != {1, 2, 3}.
  void SetCoord (const int theIndex, const double theXi)
  {
    coord.SetCoord (theIndex, theXi);
  }

  //! For this point, assigns  the values theXp, theYp and theZp to its three coordinates.
  void SetCoord (const double theXp, const double theYp, const double theZp)
  {
    coord.SetCoord (theXp, theYp, theZp);
  }

  //! Assigns the given value to the X coordinate of this point.
  void SetX (const double theX) { coord.SetX (theX); }

  //! Assigns the given value to the Y coordinate of this point.
  void SetY (const double theY) { coord.SetY (theY); }

  //! Assigns the given value to the Z coordinate of this point.
  void SetZ (const double theZ) { coord.SetZ (theZ); }

  //! Assigns the three coordinates of theCoord to this point.
  void SetXYZ (const gp_XYZ& theCoord) { coord = theCoord; }

  //! Returns the coordinate of corresponding to the value of theIndex :
  //! theIndex = 1 => X is returned
  //! theIndex = 2 => Y is returned
  //! theIndex = 3 => Z is returned
  //! Raises OutOfRange if theIndex != {1, 2, 3}.
  //! Raised if theIndex != {1, 2, 3}.
  double Coord (const int theIndex) const { return coord.Coord (theIndex); }

  //! For this point gives its three coordinates theXp, theYp and theZp.
  void Coord (double& theXp, double& theYp, double& theZp) const
  {
    coord.Coord (theXp, theYp, theZp);
  }

  //! For this point, returns its X coordinate.
  double X() const { return coord.X(); }

  //! For this point, returns its Y coordinate.
  double Y() const { return coord.Y(); }

  //! For this point, returns its Z coordinate.
  double Z() const { return coord.Z(); }

  //! For this point, returns its three coordinates as a XYZ object.
  const gp_XYZ& XYZ() const { return coord; }

  //! For this point, returns its three coordinates as a XYZ object.
  const gp_XYZ& Coord() const { return coord; }

  //! Returns the coordinates of this point.
  //! Note: This syntax allows direct modification of the returned value.
  gp_XYZ& ChangeCoord() { return coord; }

  //! Assigns the result of the following expression to this point
  //! (theAlpha*this + theBeta*theP) / (theAlpha + theBeta)
  void BaryCenter (const double theAlpha, const gp_Pnt& theP, const double theBeta)
  {
    coord.SetLinearForm (theAlpha, coord, theBeta, theP.coord);
    coord.Divide (theAlpha + theBeta);
  }

  //! Comparison
  //! Returns True if the distance between the two points is
  //! lower or equal to theLinearTolerance.
  bool IsEqual (const gp_Pnt& theOther, const double theLinearTolerance) const
  {
    return Distance (theOther) <= theLinearTolerance;
  }

  //! Computes the distance between two points.
  mobiusGeom_EXPORT double Distance (const gp_Pnt& theOther) const;

  //! Computes the square distance between two points.
  mobiusGeom_EXPORT double SquareDistance (const gp_Pnt& theOther) const;

  //! Performs the symmetrical transformation of a point
  //! with respect to the point theP which is the center of
  //! the  symmetry.
  mobiusGeom_EXPORT void Mirror (const gp_Pnt& theP);

  //! Performs the symmetrical transformation of a point
  //! with respect to an axis placement which is the axis
  //! of the symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Pnt Mirrored (const gp_Pnt& theP) const;

  mobiusGeom_EXPORT void Mirror (const gp_Ax1& theA1);

  //! Performs the symmetrical transformation of a point
  //! with respect to a plane. The axis placement theA2 locates
  //! the plane of the symmetry : (Location, XDirection, YDirection).
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Pnt Mirrored (const gp_Ax1& theA1) const;

  mobiusGeom_EXPORT void Mirror (const gp_Ax2& theA2);

  //! Rotates a point. theA1 is the axis of the rotation.
  //! theAng is the angular value of the rotation in radians.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Pnt Mirrored (const gp_Ax2& theA2) const;

  mobiusGeom_EXPORT void Rotate (const gp_Ax1& theA1, const double theAng);

  mobiusCore_NODISCARD gp_Pnt Rotated (const gp_Ax1& theA1, const double theAng) const
  {
    gp_Pnt aP = *this;
    aP.Rotate (theA1, theAng);
    return aP;
  }

  //! Scales a point. theS is the scaling value.
  mobiusGeom_EXPORT void Scale (const gp_Pnt& theP, const double theS);

  mobiusCore_NODISCARD gp_Pnt Scaled (const gp_Pnt& theP, const double theS) const
  {
    gp_Pnt aPres = *this;
    aPres.Scale (theP, theS);
    return aPres;
  }

  //! Transforms a point with the transformation T.
  mobiusGeom_EXPORT void Transform (const gp_Trsf& theT);

  mobiusCore_NODISCARD gp_Pnt Transformed (const gp_Trsf& theT) const
  {
    gp_Pnt aP = *this;
    aP.Transform (theT);
    return aP;
  }

  //! Translates a point in the direction of the vector theV.
  //! The magnitude of the translation is the vector's magnitude.
  mobiusGeom_EXPORT void Translate (const gp_Vec& theV);

  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Pnt Translated (const gp_Vec& theV) const;

  //! Translates a point from the point theP1 to the point theP2.
  void Translate (const gp_Pnt& theP1, const gp_Pnt& theP2)
  {
    coord.Add (theP2.coord);
    coord.Subtract (theP1.coord);
  }

  mobiusCore_NODISCARD gp_Pnt Translated (const gp_Pnt& theP1, const gp_Pnt& theP2) const
  {
    gp_Pnt aP = *this;
    aP.Translate (theP1, theP2);
    return aP;
  }

private:

  gp_XYZ coord;

};

}
}

#endif // _gp_Pnt_HeaderFile
