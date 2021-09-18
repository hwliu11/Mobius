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

#ifndef _gp_Vec2d_HeaderFile
#define _gp_Vec2d_HeaderFile

#include <mobius/gp_XY.hxx>

#include <mobius/occMathDefs.hxx>

class gp_Dir2d;
class gp_Pnt2d;
class gp_Ax2d;
class gp_Trsf2d;

//! Defines a non-persistent vector in 2D space.
class gp_Vec2d 
{
public:

  //! Creates a zero vector.
  gp_Vec2d() {}

  //! Creates a unitary vector from a direction theV.
  gp_Vec2d (const gp_Dir2d& theV);

  //! Creates a vector with a doublet of coordinates.
  gp_Vec2d (const gp_XY& theCoord)
    : coord(theCoord)
  {}

  //! Creates a point with its two Cartesian coordinates.
  gp_Vec2d (const double theXv, const double theYv)
  : coord (theXv, theYv)
  {}

  //! Creates a vector from two points. The length of the vector
  //! is the distance between theP1 and theP2
  gp_Vec2d (const gp_Pnt2d& theP1, const gp_Pnt2d& theP2);

  //! Changes the coordinate of range theIndex
  //! theIndex = 1 => X is modified
  //! theIndex = 2 => Y is modified
  //! Raises OutOfRange if theIndex != {1, 2}.
  void SetCoord (const int theIndex, const double theXi) { coord.SetCoord (theIndex, theXi); }

  //! For this vector, assigns
  //! the values theXv and theYv to its two coordinates
  void SetCoord (const double theXv, const double theYv) { coord.SetCoord (theXv, theYv); }

  //! Assigns the given value to the X coordinate of this vector.
  void SetX (const double theX) { coord.SetX (theX); }

  //! Assigns the given value to the Y coordinate of this vector.
  void SetY (const double theY) { coord.SetY (theY); }

  //! Assigns the two coordinates of theCoord to this vector.
  void SetXY (const gp_XY& theCoord) { coord = theCoord; }

  //! Returns the coordinate of range theIndex :
  //! theIndex = 1 => X is returned
  //! theIndex = 2 => Y is returned
  //! Raised if theIndex != {1, 2}.
  double Coord (const int theIndex) const { return coord.Coord (theIndex); }

  //! For this vector, returns  its two coordinates theXv and theYv
  void Coord (double& theXv, double& theYv) const { coord.Coord (theXv, theYv); }

  //! For this vector, returns its X  coordinate.
  double X() const { return coord.X(); }

  //! For this vector, returns its Y  coordinate.
  double Y() const { return coord.Y(); }

  //! For this vector, returns its two coordinates as a number pair
  const gp_XY& XY() const { return coord; }

  //! Returns True if the two vectors have the same magnitude value
  //! and the same direction. The precision values are theLinearTolerance
  //! for the magnitude and theAngularTolerance for the direction.
  mobiusGeom_EXPORT bool IsEqual (const gp_Vec2d& theOther, const double theLinearTolerance, const double theAngularTolerance) const;

  //! Returns True if abs(Abs(<me>.Angle(theOther)) - PI/2.)
  //! <= theAngularTolerance
  //! Raises VectorWithNullMagnitude if <me>.Magnitude() <= Resolution or
  //! theOther.Magnitude() <= Resolution from gp.
  bool IsNormal (const gp_Vec2d& theOther, const double theAngularTolerance) const
  {
    const double anAng = Abs (M_PI_2 - Abs (Angle (theOther)));
    return !(anAng > theAngularTolerance);
  }

  //! Returns True if PI - Abs(<me>.Angle(theOther)) <= theAngularTolerance
  //! Raises VectorWithNullMagnitude if <me>.Magnitude() <= Resolution or
  //! theOther.Magnitude() <= Resolution from gp.
  bool IsOpposite (const gp_Vec2d& theOther, const double theAngularTolerance) const;

  //! Returns true if Abs(Angle(<me>, theOther)) <= theAngularTolerance or
  //! PI - Abs(Angle(<me>, theOther)) <= theAngularTolerance
  //! Two vectors with opposite directions are considered as parallel.
  //! Raises VectorWithNullMagnitude if <me>.Magnitude() <= Resolution or
  //! theOther.Magnitude() <= Resolution from gp
  bool IsParallel (const gp_Vec2d& theOther, const double theAngularTolerance) const;

  //! Computes the angular value between <me> and <theOther>
  //! returns the angle value between -PI and PI in radian.
  //! The orientation is from <me> to theOther. The positive sense is the
  //! trigonometric sense.
  //! Raises VectorWithNullMagnitude if <me>.Magnitude() <= Resolution from gp or
  //! theOther.Magnitude() <= Resolution because the angular value is
  //! indefinite if one of the vectors has a null magnitude.
  mobiusGeom_EXPORT double Angle (const gp_Vec2d& theOther) const;

  //! Computes the magnitude of this vector.
  double Magnitude() const { return coord.Modulus(); }

  //! Computes the square magnitude of this vector.
  double SquareMagnitude() const { return coord.SquareModulus(); }

  void Add (const gp_Vec2d& theOther) { coord.Add (theOther.coord); }

  void operator += (const gp_Vec2d& theOther) { Add (theOther); }

  //! Adds two vectors
  mobiusCore_NODISCARD gp_Vec2d Added (const gp_Vec2d& theOther) const
  {
    gp_Vec2d aV = *this;
    aV.coord.Add (theOther.coord);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec2d operator + (const gp_Vec2d& theOther) const { return Added (theOther); }

  //! Computes the crossing product between two vectors
  mobiusCore_NODISCARD double Crossed (const gp_Vec2d& theRight) const
  {
    return coord.Crossed (theRight.coord);
  }

  mobiusCore_NODISCARD double operator ^ (const gp_Vec2d& theRight) const { return Crossed (theRight); }

  //! Computes the magnitude of the cross product between <me> and
  //! theRight. Returns || <me> ^ theRight ||
  double CrossMagnitude (const gp_Vec2d& theRight) const
  {
    return coord.CrossMagnitude (theRight.coord);
  }

  //! Computes the square magnitude of the cross product between <me> and
  //! theRight. Returns || <me> ^ theRight ||**2
  double CrossSquareMagnitude (const gp_Vec2d& theRight) const
  {
    return coord.CrossSquareMagnitude (theRight.coord);
  }

  void Divide (const double theScalar) { coord.Divide (theScalar); }

  void operator /= (const double theScalar) { Divide (theScalar); }

  //! divides a vector by a scalar
  mobiusCore_NODISCARD gp_Vec2d Divided (const double theScalar) const
  {
    gp_Vec2d aV = *this;
    aV.coord.Divide (theScalar);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec2d operator / (const double theScalar) const { return Divided (theScalar); }

  //! Computes the scalar product
  double Dot (const gp_Vec2d& theOther) const { return coord.Dot (theOther.coord); }

  double operator * (const gp_Vec2d& theOther) const { return Dot (theOther); }

  gp_Vec2d GetNormal() const { return gp_Vec2d (this->Y(), (-1)*this->X()); }

  void Multiply (const double theScalar) { coord.Multiply (theScalar); }

  void operator *= (const double theScalar) { Multiply (theScalar); }

  //! Normalizes a vector
  //! Raises an exception if the magnitude of the vector is
  //! lower or equal to Resolution from package gp.
  mobiusCore_NODISCARD gp_Vec2d Multiplied (const double theScalar) const
  {
    gp_Vec2d aV = *this;
    aV.coord.Multiply (theScalar);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec2d operator * (const double theScalar) const { return Multiplied (theScalar); }

  void Normalize()
  {
    double aD = coord.Modulus();
    coord.Divide (aD);
  }

  //! Normalizes a vector
  //! Raises an exception if the magnitude of the vector is
  //! lower or equal to Resolution from package gp.
  //! Reverses the direction of a vector
  mobiusCore_NODISCARD gp_Vec2d Normalized() const;

  void Reverse() { coord.Reverse(); }

  //! Reverses the direction of a vector
  mobiusCore_NODISCARD gp_Vec2d Reversed() const
  {
    gp_Vec2d aV = *this;
    aV.coord.Reverse();
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec2d operator -() const { return Reversed(); }

  //! Subtracts two vectors
  void Subtract (const gp_Vec2d& theRight)
  {
    coord.Subtract (theRight.coord);
  }

  void operator -= (const gp_Vec2d& theRight) { Subtract (theRight); }

  //! Subtracts two vectors
  mobiusCore_NODISCARD gp_Vec2d Subtracted (const gp_Vec2d& theRight) const
  {
    gp_Vec2d aV = *this;
    aV.coord.Subtract (theRight.coord);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec2d operator - (const gp_Vec2d& theRight) const { return Subtracted (theRight); }

  //! <me> is set to the following linear form :
  //! theA1 * theV1 + theA2 * theV2 + theV3
  void SetLinearForm (const double theA1, const gp_Vec2d& theV1,
                      const double theA2, const gp_Vec2d& theV2, const gp_Vec2d& theV3)
  {
    coord.SetLinearForm (theA1, theV1.coord, theA2, theV2.coord, theV3.coord);
  }

  //! <me> is set to the following linear form : theA1 * theV1 + theA2 * theV2
  void SetLinearForm (const double theA1, const gp_Vec2d& theV1,
                      const double theA2, const gp_Vec2d& theV2)
  {
    coord.SetLinearForm (theA1, theV1.coord, theA2, theV2.coord);
  }

  //! <me> is set to the following linear form : theA1 * theV1 + theV2
  void SetLinearForm (const double theA1, const gp_Vec2d& theV1, const gp_Vec2d& theV2)
  {
    coord.SetLinearForm (theA1, theV1.coord, theV2.coord);
  }

  //! <me> is set to the following linear form : theV1 + theV2
  void SetLinearForm (const gp_Vec2d& theV1, const gp_Vec2d& theV2)
  {
    coord.SetLinearForm (theV1.coord, theV2.coord);
  }

  //! Performs the symmetrical transformation of a vector
  //! with respect to the vector theV which is the center of
  //! the  symmetry.
  mobiusGeom_EXPORT void Mirror (const gp_Vec2d& theV);
 
  //! Performs the symmetrical transformation of a vector
  //! with respect to the vector theV which is the center of
  //! the  symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Vec2d Mirrored (const gp_Vec2d& theV) const;

  //! Performs the symmetrical transformation of a vector
  //! with respect to an axis placement which is the axis
  //! of the symmetry.
  mobiusGeom_EXPORT void Mirror (const gp_Ax2d& theA1);

  //! Performs the symmetrical transformation of a vector
  //! with respect to an axis placement which is the axis
  //! of the symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Vec2d Mirrored (const gp_Ax2d& theA1) const;

  void Rotate (const double theAng);

  //! Rotates a vector. theAng is the angular value of the
  //! rotation in radians.
  mobiusCore_NODISCARD gp_Vec2d Rotated (const double theAng) const
  {
    gp_Vec2d aV = *this;
    aV.Rotate (theAng);
    return aV;
  }

  void Scale (const double theS) { coord.Multiply (theS); }

  //! Scales a vector. theS is the scaling value.
  mobiusCore_NODISCARD gp_Vec2d Scaled (const double theS) const
  {
    gp_Vec2d aV = *this;
    aV.coord.Multiply (theS);
    return aV;
  }

  mobiusGeom_EXPORT void Transform (const gp_Trsf2d& theT);

  //! Transforms a vector with a Trsf from gp.
  mobiusCore_NODISCARD gp_Vec2d Transformed (const gp_Trsf2d& theT) const
  {
    gp_Vec2d aV = *this;
    aV.Transform (theT);
    return aV;
  }

private:

  gp_XY coord;

};

#include <mobius/gp_Dir2d.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_Pnt2d.hxx>

//=======================================================================
//function :  gp_Vec2d
// purpose :
//=======================================================================
inline gp_Vec2d::gp_Vec2d (const gp_Dir2d& theV)
{
  coord = theV.XY();
}

//=======================================================================
//function :  gp_Vec2d
// purpose :
//=======================================================================
inline gp_Vec2d::gp_Vec2d (const gp_Pnt2d& theP1, const gp_Pnt2d& theP2)
{
  coord = theP2.XY().Subtracted (theP1.XY());
}

//=======================================================================
//function :  IsOpposite
// purpose :
//=======================================================================
inline bool gp_Vec2d::IsOpposite (const gp_Vec2d& theOther, const double theAngularTolerance) const
{
  double anAng = Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return M_PI - anAng <= theAngularTolerance;
}

//=======================================================================
//function :  IsParallel
// purpose :
//=======================================================================
inline bool gp_Vec2d::IsParallel (const gp_Vec2d& theOther, const double theAngularTolerance) const
{
  double anAng = Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
}

//=======================================================================
//function :  Normalized
// purpose :
//=======================================================================
inline gp_Vec2d gp_Vec2d::Normalized() const
{
  double aD = coord.Modulus();
  gp_Vec2d aV = *this;
  aV.coord.Divide (aD);
  return aV;
}

//=======================================================================
//function :  Rotate
// purpose :
//=======================================================================
inline void gp_Vec2d::Rotate (const double theAng)
{
  gp_Trsf2d aT;
  aT.SetRotation (gp_Pnt2d(0.0, 0.0), theAng);
  coord.Multiply (aT.VectorialPart());
}

//=======================================================================
//function :  operator*
// purpose :
//=======================================================================
inline gp_Vec2d operator* (const double theScalar,
                           const gp_Vec2d& theV)
{
  return theV.Multiplied (theScalar);
}

#endif // _gp_Vec2d_HeaderFile
