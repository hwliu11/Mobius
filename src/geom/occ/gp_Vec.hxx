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

#ifndef _gp_Vec_HeaderFile
#define _gp_Vec_HeaderFile

#include <mobius/gp_XYZ.hxx>
#include <mobius/occMathDefs.hxx>

class gp_Dir;
class gp_Pnt;
class gp_Ax1;
class gp_Ax2;
class gp_Trsf;

//! Defines a non-persistent vector in 3D space.
class gp_Vec 
{
public:

  //! Creates a zero vector.
  gp_Vec() {}

  //! Creates a unitary vector from a direction theV.
  gp_Vec (const gp_Dir& theV);

  //! Creates a vector with a triplet of coordinates.
  gp_Vec (const gp_XYZ& theCoord)
  : coord (theCoord)
  {}

  //! Creates a point with its three cartesian coordinates.
  gp_Vec (const double theXv, const double theYv, const double theZv)
  : coord (theXv, theYv, theZv)
  {}

  //! Creates a vector from two points. The length of the vector
  //! is the distance between theP1 and theP2
  gp_Vec (const gp_Pnt& theP1, const gp_Pnt& theP2);

  //! Changes the coordinate of range theIndex
  //! theIndex = 1 => X is modified
  //! theIndex = 2 => Y is modified
  //! theIndex = 3 => Z is modified
  //! Raised if theIndex != {1, 2, 3}.
  void SetCoord (const int theIndex, const double theXi) { coord.SetCoord (theIndex, theXi); }

  //! For this vector, assigns
  //! -   the values theXv, theYv and theZv to its three coordinates.
  void SetCoord (const double theXv, const double theYv, const double theZv)
  {
    coord.SetX (theXv);
    coord.SetY (theYv);
    coord.SetZ (theZv);
  }

  //! Assigns the given value to the X coordinate of this vector.
  void SetX (const double theX) { coord.SetX(theX); }

  //! Assigns the given value to the X coordinate of this vector.
  void SetY (const double theY) { coord.SetY (theY); }

  //! Assigns the given value to the X coordinate of this vector.
  void SetZ (const double theZ) { coord.SetZ (theZ); }

  //! Assigns the three coordinates of theCoord to this vector.
  void SetXYZ (const gp_XYZ& theCoord) { coord = theCoord; }

  //! Returns the coordinate of range theIndex :
  //! theIndex = 1 => X is returned
  //! theIndex = 2 => Y is returned
  //! theIndex = 3 => Z is returned
  //! Raised if theIndex != {1, 2, 3}.
  double Coord (const int theIndex) const { return coord.Coord (theIndex); }

  //! For this vector returns its three coordinates theXv, theYv, and theZv inline
  void Coord (double& theXv, double& theYv, double& theZv) const
  {
    theXv = coord.X();
    theYv = coord.Y();
    theZv = coord.Z();
  }

  //! For this vector, returns its X coordinate.
  double X() const { return coord.X(); }

  //! For this vector, returns its Y coordinate.
  double Y() const { return coord.Y(); }

  //! For this vector, returns its Z  coordinate.
  double Z() const { return coord.Z(); }

  //! For this vector, returns
  //! -   its three coordinates as a number triple
  const gp_XYZ& XYZ() const { return coord; }

  //! Returns True if the two vectors have the same magnitude value
  //! and the same direction. The precision values are theLinearTolerance
  //! for the magnitude and theAngularTolerance for the direction.
  mobiusGeom_EXPORT bool IsEqual (const gp_Vec& theOther, const double theLinearTolerance, const double theAngularTolerance) const;

  //! Returns True if abs(<me>.Angle(theOther) - PI/2.) <= theAngularTolerance
  //! Raises VectorWithNullMagnitude if <me>.Magnitude() <= Resolution or
  //! theOther.Magnitude() <= Resolution from gp
  bool IsNormal (const gp_Vec& theOther, const double theAngularTolerance) const;

  //! Returns True if PI - <me>.Angle(theOther) <= theAngularTolerance
  //! Raises VectorWithNullMagnitude if <me>.Magnitude() <= Resolution or
  //! Other.Magnitude() <= Resolution from gp
  bool IsOpposite (const gp_Vec& theOther, const double theAngularTolerance) const
  {
    double anAng = M_PI - Angle (theOther);
    return anAng <= theAngularTolerance;
  }

  //! Returns True if Angle(<me>, theOther) <= theAngularTolerance or
  //! PI - Angle(<me>, theOther) <= theAngularTolerance
  //! This definition means that two parallel vectors cannot define
  //! a plane but two vectors with opposite directions are considered
  //! as parallel. Raises VectorWithNullMagnitude if <me>.Magnitude() <= Resolution or
  //! Other.Magnitude() <= Resolution from gp
  bool IsParallel (const gp_Vec& theOther, const double theAngularTolerance) const
  {
    double anAng = Angle (theOther);
    return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
  }

  //! Computes the angular value between <me> and <theOther>
  //! Returns the angle value between 0 and PI in radian.
  //! Raises VectorWithNullMagnitude if <me>.Magnitude() <= Resolution from gp or
  //! theOther.Magnitude() <= Resolution because the angular value is
  //! indefinite if one of the vectors has a null magnitude.
  double Angle (const gp_Vec& theOther) const;

  //! Computes the angle, in radians, between this vector and
  //! vector theOther. The result is a value between -Pi and Pi.
  //! For this, theVRef defines the positive sense of rotation: the
  //! angular value is positive, if the cross product this ^ theOther
  //! has the same orientation as theVRef relative to the plane
  //! defined by the vectors this and theOther. Otherwise, the
  //! angular value is negative.
  //! Exceptions
  //! gp_VectorWithNullMagnitude if the magnitude of this
  //! vector, the vector theOther, or the vector theVRef is less than or
  //! equal to gp::Resolution().
  //! Standard_DomainError if this vector, the vector theOther,
  //! and the vector theVRef are coplanar, unless this vector and
  //! the vector theOther are parallel.
  double AngleWithRef (const gp_Vec& theOther, const gp_Vec& theVRef) const;

  //! Computes the magnitude of this vector.
  double Magnitude() const { return coord.Modulus(); }

  //! Computes the square magnitude of this vector.
  double SquareMagnitude() const { return coord.SquareModulus(); }

  //! Adds two vectors
  void Add (const gp_Vec& theOther) { coord.Add (theOther.coord); }

  void operator += (const gp_Vec& theOther) { Add (theOther); }

  //! Adds two vectors
  mobiusCore_NODISCARD gp_Vec Added (const gp_Vec& theOther) const
  {
    gp_Vec aV = *this;
    aV.coord.Add (theOther.coord);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec operator + (const gp_Vec& theOther) const { return Added (theOther); }

  //! Subtracts two vectors
  void Subtract (const gp_Vec& theRight) { coord.Subtract (theRight.coord); }

  void operator -= (const gp_Vec& theRight) { Subtract (theRight); }

  //! Subtracts two vectors
  mobiusCore_NODISCARD gp_Vec Subtracted (const gp_Vec& theRight) const
  {
    gp_Vec aV = *this;
    aV.coord.Subtract (theRight.coord);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec operator - (const gp_Vec& theRight) const { return Subtracted (theRight); }

  //! Multiplies a vector by a scalar
  void Multiply (const double theScalar) { coord.Multiply (theScalar); }

  void operator *= (const double theScalar) { Multiply (theScalar); }

  //! Multiplies a vector by a scalar
  mobiusCore_NODISCARD gp_Vec Multiplied (const double theScalar) const
  {
    gp_Vec aV = *this;
    aV.coord.Multiply (theScalar);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec operator * (const double theScalar) const { return Multiplied (theScalar); }

  //! Divides a vector by a scalar
  void Divide (const double theScalar) { coord.Divide (theScalar); }

  void operator /= (const double theScalar) { Divide (theScalar); }

  //! Divides a vector by a scalar
  mobiusCore_NODISCARD gp_Vec Divided (const double theScalar) const
  {
    gp_Vec aV = *this;
    aV.coord.Divide (theScalar);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec operator / (const double theScalar) const { return Divided (theScalar); }

  //! computes the cross product between two vectors
  void Cross (const gp_Vec& theRight) { coord.Cross (theRight.coord); }

  void operator ^= (const gp_Vec& theRight) { Cross (theRight); }

  //! computes the cross product between two vectors
  mobiusCore_NODISCARD gp_Vec Crossed (const gp_Vec& theRight) const
  {
    gp_Vec aV = *this;
    aV.coord.Cross (theRight.coord);
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec operator ^ (const gp_Vec& theRight) const { return Crossed (theRight); }

  //! Computes the magnitude of the cross
  //! product between <me> and theRight.
  //! Returns || <me> ^ theRight ||
  double CrossMagnitude (const gp_Vec& theRight) const { return coord.CrossMagnitude (theRight.coord); }

  //! Computes the square magnitude of
  //! the cross product between <me> and theRight.
  //! Returns || <me> ^ theRight ||**2
  double CrossSquareMagnitude (const gp_Vec& theRight) const
  {
    return coord.CrossSquareMagnitude (theRight.coord);
  }

  //! Computes the triple vector product.
  //! <me> ^= (theV1 ^ theV2)
  void CrossCross (const gp_Vec& theV1, const gp_Vec& theV2)
  {
    coord.CrossCross (theV1.coord, theV2.coord);
  }

  //! Computes the triple vector product.
  //! <me> ^ (theV1 ^ theV2)
  mobiusCore_NODISCARD gp_Vec CrossCrossed (const gp_Vec& theV1, const gp_Vec& theV2) const
  {
    gp_Vec aV = *this;
    aV.coord.CrossCross (theV1.coord, theV2.coord);
    return aV;
  }

  //! computes the scalar product
  double Dot (const gp_Vec& theOther) const { return coord.Dot (theOther.coord); }

  double operator * (const gp_Vec& theOther) const { return Dot (theOther); }

  //! Computes the triple scalar product <me> * (theV1 ^ theV2).
  double DotCross (const gp_Vec& theV1, const gp_Vec& theV2) const
  {
    return coord.DotCross (theV1.coord, theV2.coord);
  }

  //! normalizes a vector
  //! Raises an exception if the magnitude of the vector is
  //! lower or equal to Resolution from gp.
  void Normalize()
  {
    double aD = coord.Modulus();
    coord.Divide (aD);
  }

  //! normalizes a vector
  //! Raises an exception if the magnitude of the vector is
  //! lower or equal to Resolution from gp.
  mobiusCore_NODISCARD gp_Vec Normalized() const;

  //! Reverses the direction of a vector
  void Reverse() { coord.Reverse(); }

  //! Reverses the direction of a vector
  mobiusCore_NODISCARD gp_Vec Reversed() const
  {
    gp_Vec aV = *this;
    aV.coord.Reverse();
    return aV;
  }

  mobiusCore_NODISCARD gp_Vec operator -() const { return Reversed(); }

  //! <me> is set to the following linear form :
  //! theA1 * theV1 + theA2 * theV2 + theA3 * theV3 + theV4
  void SetLinearForm (const double theA1, const gp_Vec& theV1,
                      const double theA2, const gp_Vec& theV2,
                      const double theA3, const gp_Vec& theV3, const gp_Vec& theV4)
  {
    coord.SetLinearForm (theA1, theV1.coord, theA2, theV2.coord, theA3, theV3.coord, theV4.coord);
  }

  //! <me> is set to the following linear form :
  //! theA1 * theV1 + theA2 * theV2 + theA3 * theV3
  void SetLinearForm (const double theA1, const gp_Vec& theV1,
                      const double theA2, const gp_Vec& theV2,
                      const double theA3, const gp_Vec& theV3)
  {
    coord.SetLinearForm (theA1, theV1.coord, theA2, theV2.coord, theA3, theV3.coord);
  }

  //! <me> is set to the following linear form :
  //! theA1 * theV1 + theA2 * theV2 + theV3
  void SetLinearForm (const double theA1, const gp_Vec& theV1,
                      const double theA2, const gp_Vec& theV2, const gp_Vec& theV3)
  {
    coord.SetLinearForm (theA1, theV1.coord, theA2, theV2.coord, theV3.coord);
  }

  //! <me> is set to the following linear form :
  //! theA1 * theV1 + theA2 * theV2
  void SetLinearForm (const double theA1, const gp_Vec& theV1,
                      const double theA2, const gp_Vec& theV2)
  {
    coord.SetLinearForm (theA1, theV1.coord, theA2, theV2.coord);
  }

  //! <me> is set to the following linear form : theA1 * theV1 + theV2
  void SetLinearForm (const double theA1, const gp_Vec& theV1, const gp_Vec& theV2)
  {
    coord.SetLinearForm (theA1, theV1.coord, theV2.coord);
  }

  //! <me> is set to the following linear form : theV1 + theV2
  void SetLinearForm (const gp_Vec& theV1, const gp_Vec& theV2)
  {
    coord.SetLinearForm (theV1.coord, theV2.coord);
  }

  mobiusGeom_EXPORT void Mirror (const gp_Vec& theV);

  //! Performs the symmetrical transformation of a vector
  //! with respect to the vector theV which is the center of
  //! the  symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Vec Mirrored (const gp_Vec& theV) const;

  mobiusGeom_EXPORT void Mirror (const gp_Ax1& theA1);

  //! Performs the symmetrical transformation of a vector
  //! with respect to an axis placement which is the axis
  //! of the symmetry.
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Vec Mirrored (const gp_Ax1& theA1) const;

  mobiusGeom_EXPORT void Mirror (const gp_Ax2& theA2);

  //! Performs the symmetrical transformation of a vector
  //! with respect to a plane. The axis placement theA2 locates
  //! the plane of the symmetry : (Location, XDirection, YDirection).
  mobiusCore_NODISCARD mobiusGeom_EXPORT gp_Vec Mirrored (const gp_Ax2& theA2) const;

  void Rotate (const gp_Ax1& theA1, const double theAng);

  //! Rotates a vector. theA1 is the axis of the rotation.
  //! theAng is the angular value of the rotation in radians.
  mobiusCore_NODISCARD gp_Vec Rotated (const gp_Ax1& theA1, const double theAng) const
  {
    gp_Vec aVres = *this;
    aVres.Rotate (theA1, theAng);
    return aVres;
  }

  void Scale (const double theS) { coord.Multiply (theS); }

  //! Scales a vector. theS is the scaling value.
  mobiusCore_NODISCARD gp_Vec Scaled (const double theS) const
  {
    gp_Vec aV = *this;
    aV.coord.Multiply (theS);
    return aV;
  }

  //! Transforms a vector with the transformation theT.
  mobiusGeom_EXPORT void Transform (const gp_Trsf& theT);

  //! Transforms a vector with the transformation theT.
  mobiusCore_NODISCARD gp_Vec Transformed (const gp_Trsf& theT) const
  {
    gp_Vec aV = *this;
    aV.Transform (theT);
    return aV;
  }

  //! Dumps the content of me into the stream
  mobiusGeom_EXPORT void DumpJson (std::ostream& theOStream, int theDepth = -1) const;

private:

  gp_XYZ coord;

};


#include <mobius/gp.hxx>
#include <mobius/gp_Dir.hxx>
#include <mobius/gp_Pnt.hxx>
#include <mobius/gp_Trsf.hxx>

//=======================================================================
//function :  gp_Vec
// purpose :
//=======================================================================
inline gp_Vec::gp_Vec (const gp_Dir& theV)
{
  coord = theV.XYZ();
}

//=======================================================================
//function :  gp_Vec
// purpose :
//=======================================================================
inline gp_Vec::gp_Vec (const gp_Pnt& theP1, const gp_Pnt& theP2)
{
  coord = theP2.XYZ().Subtracted (theP1.XYZ());
}

//=======================================================================
//function :  IsNormal
// purpose :
//=======================================================================
inline bool gp_Vec::IsNormal (const gp_Vec& theOther, const double theAngularTolerance) const
{
  double anAng = M_PI / 2.0 - Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return  anAng <= theAngularTolerance;
}

//=======================================================================
//function :  Angle
// purpose :
//=======================================================================
inline double gp_Vec::Angle (const gp_Vec& theOther) const
{
  return (gp_Dir (coord)).Angle (theOther);
}

//=======================================================================
//function :  AngleWithRef
// purpose :
//=======================================================================
inline double gp_Vec::AngleWithRef (const gp_Vec& theOther, const gp_Vec& theVRef) const
{
  return (gp_Dir (coord)).AngleWithRef (theOther, theVRef);
}

//=======================================================================
//function :  Normalized
// purpose :
//=======================================================================
inline gp_Vec gp_Vec::Normalized() const
{
  double aD = coord.Modulus();
  gp_Vec aV = *this;
  aV.coord.Divide (aD);
  return aV;
}

//=======================================================================
//function :  Rotate
// purpose :
//=======================================================================
inline void gp_Vec::Rotate (const gp_Ax1& theA1, const double theAng)
{
  gp_Trsf aT;
  aT.SetRotation (theA1, theAng);
  coord.Multiply (aT.VectorialPart());
}

//=======================================================================
//function :  operator*
// purpose :
//=======================================================================
inline gp_Vec operator* (const double theScalar, const gp_Vec& theV)
{
  return theV.Multiplied(theScalar);
}

#endif // _gp_Vec_HeaderFile
