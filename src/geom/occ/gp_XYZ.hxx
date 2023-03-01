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

#ifndef _gp_XYZ_HeaderFile
#define _gp_XYZ_HeaderFile

#include <mobius/geom.h>

#include <mobius/gp.hxx>
#include <mobius/gp_Mat.hxx>

namespace mobius {
namespace occ {

//! This class describes a cartesian coordinate entity in
//! 3D space {X,Y,Z}. This entity is used for algebraic
//! calculation.
//! It is used in vectorial computations or for holding this type
//! of information in data structures.
class gp_XYZ
{
public:

  //! Creates an XYZ object with zero coordinates (0,0,0)
  gp_XYZ()
  : x (0.),
    y (0.),
    z (0.)
  {}

  //! creates an XYZ with given coordinates
  gp_XYZ (const double theX, const double theY, const double theZ)
  : x (theX),
    y (theY),
    z (theZ)
  {}

  //! For this XYZ object, assigns
  //! the values theX, theY and theZ to its three coordinates
  void SetCoord (const double theX, const double theY, const double theZ)
  {
    x = theX;
    y = theY;
    z = theZ;
  }

  //! modifies the coordinate of range theIndex
  //! theIndex = 1 => X is modified
  //! theIndex = 2 => Y is modified
  //! theIndex = 3 => Z is modified
  //! Raises OutOfRange if theIndex != {1, 2, 3}.
  void SetCoord (const int theIndex, const double theXi)
  {
    (&x)[theIndex - 1] = theXi;
  }

  //! Assigns the given value to the X coordinate
  void SetX (const double theX) { x = theX; }

  //! Assigns the given value to the Y coordinate
  void SetY (const double theY) { y = theY; }

  //! Assigns the given value to the Z coordinate
  void SetZ (const double theZ) { z = theZ; }

  //! returns the coordinate of range theIndex :
  //! theIndex = 1 => X is returned
  //! theIndex = 2 => Y is returned
  //! theIndex = 3 => Z is returned
  //!
  //! Raises OutOfRange if theIndex != {1, 2, 3}.
  double Coord (const int theIndex) const
  {
    return (&x)[theIndex - 1];
  }

  double& ChangeCoord (const int theIndex)
  {
    return (&x)[theIndex - 1];
  }

  void Coord (double& theX, double& theY, double& theZ) const
  {
    theX = x;
    theY = y;
    theZ = z;
  }

  //! Returns a const ptr to coordinates location.
  //! Is useful for algorithms, but DOES NOT PERFORM
  //! ANY CHECKS!
  const double* GetData() const { return (&x); }

  //! Returns a ptr to coordinates location.
  //! Is useful for algorithms, but DOES NOT PERFORM
  //! ANY CHECKS!
  double* ChangeData() { return (&x); }

  //! Returns the X coordinate
  double X() const { return x; }

  //! Returns the Y coordinate
  double Y() const { return y; }

  //! Returns the Z coordinate
  double Z() const { return z; }

  //! computes Sqrt (X*X + Y*Y + Z*Z) where X, Y and Z are the three coordinates of this XYZ object.
  double Modulus() const { return sqrt (x * x + y * y + z * z); }

  //! Computes X*X + Y*Y + Z*Z where X, Y and Z are the three coordinates of this XYZ object.
  double SquareModulus() const { return (x * x + y * y + z * z); }

  //! Returns True if he coordinates of this XYZ object are
  //! equal to the respective coordinates Other,
  //! within the specified tolerance theTolerance. I.e.:
  //! abs(<me>.X() - theOther.X()) <= theTolerance and
  //! abs(<me>.Y() - theOther.Y()) <= theTolerance and
  //! abs(<me>.Z() - theOther.Z()) <= theTolerance.
  mobiusGeom_EXPORT bool IsEqual (const gp_XYZ& theOther, const double theTolerance) const;

  //! @code
  //! <me>.X() = <me>.X() + theOther.X()
  //! <me>.Y() = <me>.Y() + theOther.Y()
  //! <me>.Z() = <me>.Z() + theOther.Z()
  void Add (const gp_XYZ& theOther)
  {
    x += theOther.x;
    y += theOther.y;
    z += theOther.z;
  }

  void operator+= (const gp_XYZ& theOther) { Add (theOther); }

  //! @code
  //! new.X() = <me>.X() + theOther.X()
  //! new.Y() = <me>.Y() + theOther.Y()
  //! new.Z() = <me>.Z() + theOther.Z()
  //! @endcode
  mobiusCore_NODISCARD gp_XYZ Added (const gp_XYZ& theOther) const
  {
    return gp_XYZ (x + theOther.x, y + theOther.y, z + theOther.z);
  }

  mobiusCore_NODISCARD gp_XYZ operator + (const gp_XYZ& theOther) const { return Added (theOther); }

  //! @code
  //! <me>.X() = <me>.Y() * theOther.Z() - <me>.Z() * theOther.Y()
  //! <me>.Y() = <me>.Z() * theOther.X() - <me>.X() * theOther.Z()
  //! <me>.Z() = <me>.X() * theOther.Y() - <me>.Y() * theOther.X()
  void Cross (const gp_XYZ& theOther);

  void operator^= (const gp_XYZ& theOther) { Cross (theOther); }

  //! @code
  //! new.X() = <me>.Y() * theOther.Z() - <me>.Z() * theOther.Y()
  //! new.Y() = <me>.Z() * theOther.X() - <me>.X() * theOther.Z()
  //! new.Z() = <me>.X() * theOther.Y() - <me>.Y() * theOther.X()
  //! @endcode
  mobiusCore_NODISCARD gp_XYZ Crossed (const gp_XYZ& theOther) const
  {
    return gp_XYZ (y * theOther.z - z * theOther.y,
                   z * theOther.x - x * theOther.z,
                   x * theOther.y - y * theOther.x);
  }

  mobiusCore_NODISCARD gp_XYZ operator ^ (const gp_XYZ& theOther) const { return Crossed (theOther); }

  //! Computes the magnitude of the cross product between <me> and
  //! theRight. Returns || <me> ^ theRight ||
  double CrossMagnitude (const gp_XYZ& theRight) const;

  //! Computes the square magnitude of the cross product between <me> and
  //! theRight. Returns || <me> ^ theRight ||**2
  double CrossSquareMagnitude (const gp_XYZ& theRight) const;

  //! Triple vector product
  //! Computes <me> = <me>.Cross(theCoord1.Cross(theCoord2))
  void CrossCross (const gp_XYZ& theCoord1, const gp_XYZ& theCoord2);

  //! Triple vector product
  //! computes New = <me>.Cross(theCoord1.Cross(theCoord2))
  mobiusCore_NODISCARD gp_XYZ CrossCrossed (const gp_XYZ& theCoord1, const gp_XYZ& theCoord2) const
  {
    gp_XYZ aCoord0 = *this;
    aCoord0.CrossCross (theCoord1, theCoord2);
    return aCoord0;
  }

  //! divides <me> by a real.
  void Divide (const double theScalar)
  {
    x /= theScalar;
    y /= theScalar;
    z /= theScalar;
  }

  void operator/= (const double theScalar) { Divide (theScalar); }

  //! divides <me> by a real.
  mobiusCore_NODISCARD gp_XYZ Divided (const double theScalar) const { return gp_XYZ (x / theScalar, y / theScalar, z / theScalar); }

  mobiusCore_NODISCARD gp_XYZ operator/ (const double theScalar) const { return Divided (theScalar); }

  //! computes the scalar product between <me> and theOther
  double Dot (const gp_XYZ& theOther) const { return(x * theOther.x + y * theOther.y + z * theOther.z); }

  double operator* (const gp_XYZ& theOther) const { return Dot (theOther); }

  //! computes the triple scalar product
  double DotCross (const gp_XYZ& theCoord1, const gp_XYZ& theCoord2) const;

  //! @code
  //! <me>.X() = <me>.X() * theScalar;
  //! <me>.Y() = <me>.Y() * theScalar;
  //! <me>.Z() = <me>.Z() * theScalar;
  void Multiply (const double theScalar)
  {
    x *= theScalar;
    y *= theScalar;
    z *= theScalar;
  }

  void operator*= (const double theScalar) { Multiply (theScalar); }

  //! @code
  //! <me>.X() = <me>.X() * theOther.X();
  //! <me>.Y() = <me>.Y() * theOther.Y();
  //! <me>.Z() = <me>.Z() * theOther.Z();
  void Multiply (const gp_XYZ& theOther)
  {
    x *= theOther.x;
    y *= theOther.y;
    z *= theOther.z;
  }

  void operator*= (const gp_XYZ& theOther) { Multiply (theOther); }

  //! <me> = theMatrix * <me>
  void Multiply (const gp_Mat& theMatrix);

  void operator*= (const gp_Mat& theMatrix) { Multiply (theMatrix); }

  //! @code
  //! New.X() = <me>.X() * theScalar;
  //! New.Y() = <me>.Y() * theScalar;
  //! New.Z() = <me>.Z() * theScalar;
  mobiusCore_NODISCARD gp_XYZ Multiplied (const double theScalar) const { return gp_XYZ (x * theScalar, y * theScalar, z * theScalar); }

  mobiusCore_NODISCARD gp_XYZ operator* (const double theScalar) const { return Multiplied (theScalar); }

  //! @code
  //! new.X() = <me>.X() * theOther.X();
  //! new.Y() = <me>.Y() * theOther.Y();
  //! new.Z() = <me>.Z() * theOther.Z();
  mobiusCore_NODISCARD gp_XYZ Multiplied (const gp_XYZ& theOther) const { return gp_XYZ (x * theOther.x, y * theOther.y, z * theOther.z); }

  //! New = theMatrix * <me>
  //! @endcode
  mobiusCore_NODISCARD gp_XYZ Multiplied (const gp_Mat& theMatrix) const
  {
    return gp_XYZ (theMatrix.Value (1, 1) * x + theMatrix.Value (1, 2) * y + theMatrix.Value (1, 3) * z,
                   theMatrix.Value (2, 1) * x + theMatrix.Value (2, 2) * y + theMatrix.Value (2, 3) * z,
                   theMatrix.Value (3, 1) * x + theMatrix.Value (3, 2) * y + theMatrix.Value (3, 3) * z);
  }

  mobiusCore_NODISCARD gp_XYZ operator* (const gp_Mat& theMatrix) const { return Multiplied (theMatrix); }

  //! @code
  //! <me>.X() = <me>.X()/ <me>.Modulus()
  //! <me>.Y() = <me>.Y()/ <me>.Modulus()
  //! <me>.Z() = <me>.Z()/ <me>.Modulus()
  //! @endcode
  //! Raised if <me>.Modulus() <= Resolution from gp
  void Normalize();

  //! @code
  //! New.X() = <me>.X()/ <me>.Modulus()
  //! New.Y() = <me>.Y()/ <me>.Modulus()
  //! New.Z() = <me>.Z()/ <me>.Modulus()
  //! @endcode
  //! Raised if <me>.Modulus() <= Resolution from gp
  mobiusCore_NODISCARD gp_XYZ Normalized() const
  {
    double aD = Modulus();
    return gp_XYZ(x / aD, y / aD, z / aD);
  }

  //! @code
  //! <me>.X() = -<me>.X()
  //! <me>.Y() = -<me>.Y()
  //! <me>.Z() = -<me>.Z()
  void Reverse()
  {
    x = -x;
    y = -y;
    z = -z;
  }

  //! @code
  //! New.X() = -<me>.X()
  //! New.Y() = -<me>.Y()
  //! New.Z() = -<me>.Z()
  mobiusCore_NODISCARD gp_XYZ Reversed() const { return gp_XYZ (-x, -y, -z); }

  //! @code
  //! <me>.X() = <me>.X() - theOther.X()
  //! <me>.Y() = <me>.Y() - theOther.Y()
  //! <me>.Z() = <me>.Z() - theOther.Z()
  void Subtract (const gp_XYZ& theOther)
  {
    x -= theOther.x;
    y -= theOther.y;
    z -= theOther.z;
  }

  void operator-= (const gp_XYZ& theOther) { Subtract (theOther); }

  //! @code
  //! new.X() = <me>.X() - theOther.X()
  //! new.Y() = <me>.Y() - theOther.Y()
  //! new.Z() = <me>.Z() - theOther.Z()
  mobiusCore_NODISCARD gp_XYZ Subtracted (const gp_XYZ& theOther) const { return gp_XYZ (x - theOther.x, y - theOther.y, z - theOther.z); }

  mobiusCore_NODISCARD gp_XYZ operator-  (const gp_XYZ& theOther) const { return Subtracted (theOther); }

  //! <me> is set to the following linear form :
  //! @code
  //! theA1 * theXYZ1 + theA2 * theXYZ2 + theA3 * theXYZ3 + theXYZ4
  void SetLinearForm (const double theA1, const gp_XYZ& theXYZ1,
                      const double theA2, const gp_XYZ& theXYZ2,
                      const double theA3, const gp_XYZ& theXYZ3,
                      const gp_XYZ& theXYZ4)
  {
    x = theA1 * theXYZ1.x + theA2 * theXYZ2.x + theA3 * theXYZ3.x + theXYZ4.x;
    y = theA1 * theXYZ1.y + theA2 * theXYZ2.y + theA3 * theXYZ3.y + theXYZ4.y;
    z = theA1 * theXYZ1.z + theA2 * theXYZ2.z + theA3 * theXYZ3.z + theXYZ4.z;
  }

  //! <me> is set to the following linear form :
  //! @code
  //! theA1 * theXYZ1 + theA2 * theXYZ2 + theA3 * theXYZ3
  void SetLinearForm (const double theA1, const gp_XYZ& theXYZ1,
                      const double theA2, const gp_XYZ& theXYZ2,
                      const double theA3, const gp_XYZ& theXYZ3)
  {
    x = theA1 * theXYZ1.x + theA2 * theXYZ2.x + theA3 * theXYZ3.x;
    y = theA1 * theXYZ1.y + theA2 * theXYZ2.y + theA3 * theXYZ3.y;
    z = theA1 * theXYZ1.z + theA2 * theXYZ2.z + theA3 * theXYZ3.z;
  }

  //! <me> is set to the following linear form :
  //! @code
  //! theA1 * theXYZ1 + theA2 * theXYZ2 + theXYZ3
  void SetLinearForm (const double theA1, const gp_XYZ& theXYZ1,
                      const double theA2, const gp_XYZ& theXYZ2,
                      const gp_XYZ& theXYZ3)
  {
    x = theA1 * theXYZ1.x + theA2 * theXYZ2.x + theXYZ3.x;
    y = theA1 * theXYZ1.y + theA2 * theXYZ2.y + theXYZ3.y;
    z = theA1 * theXYZ1.z + theA2 * theXYZ2.z + theXYZ3.z;
  }

  //! <me> is set to the following linear form :
  //! @code
  //! theA1 * theXYZ1 + theA2 * theXYZ2
  void SetLinearForm (const double theA1, const gp_XYZ& theXYZ1,
                      const double theA2, const gp_XYZ& theXYZ2)
  {
    x = theA1 * theXYZ1.x + theA2 * theXYZ2.x;
    y = theA1 * theXYZ1.y + theA2 * theXYZ2.y;
    z = theA1 * theXYZ1.z + theA2 * theXYZ2.z;
  }

  //! <me> is set to the following linear form :
  //! @code
  //! theA1 * theXYZ1 + theXYZ2
  void SetLinearForm (const double theA1, const gp_XYZ& theXYZ1,
                      const gp_XYZ& theXYZ2)
  {
    x = theA1 * theXYZ1.x + theXYZ2.x;
    y = theA1 * theXYZ1.y + theXYZ2.y;
    z = theA1 * theXYZ1.z + theXYZ2.z;
  }

  //! <me> is set to the following linear form :
  //! @code
  //! theXYZ1 + theXYZ2
  void SetLinearForm (const gp_XYZ& theXYZ1, const gp_XYZ& theXYZ2)
  {
    x = theXYZ1.x + theXYZ2.x;
    y = theXYZ1.y + theXYZ2.y;
    z = theXYZ1.z + theXYZ2.z;
  }

private:

  double x;
  double y;
  double z;

};

//=======================================================================
//function : Cross
// purpose :
//=======================================================================
inline void gp_XYZ::Cross (const gp_XYZ& theRight)
{
  double aXresult = y * theRight.z - z * theRight.y;
  double aYresult = z * theRight.x - x * theRight.z;
  z = x * theRight.y - y * theRight.x;
  x = aXresult;
  y = aYresult;
}

//=======================================================================
//function : CrossMagnitude
// purpose :
//=======================================================================
inline double gp_XYZ::CrossMagnitude (const gp_XYZ& theRight) const
{
  double aXresult = y * theRight.z - z * theRight.y;
  double aYresult = z * theRight.x - x * theRight.z;
  double aZresult = x * theRight.y - y * theRight.x;
  return sqrt (aXresult * aXresult + aYresult * aYresult + aZresult * aZresult);
}

//=======================================================================
//function : CrossSquareMagnitude
// purpose :
//=======================================================================
inline double gp_XYZ::CrossSquareMagnitude (const gp_XYZ& theRight) const
{
  double aXresult = y * theRight.z - z * theRight.y;
  double aYresult = z * theRight.x - x * theRight.z;
  double aZresult = x * theRight.y - y * theRight.x;
  return aXresult * aXresult + aYresult * aYresult + aZresult * aZresult;
}

//=======================================================================
//function : CrossCross
// purpose :
//=======================================================================
inline void gp_XYZ::CrossCross (const gp_XYZ& theCoord1, const gp_XYZ& theCoord2)
{
  double aXresult = y * (theCoord1.x * theCoord2.y - theCoord1.y * theCoord2.x) -
                           z * (theCoord1.z * theCoord2.x - theCoord1.x * theCoord2.z);
  double anYresult = z * (theCoord1.y * theCoord2.z - theCoord1.z * theCoord2.y) -
                            x * (theCoord1.x * theCoord2.y - theCoord1.y * theCoord2.x);
  z = x * (theCoord1.z * theCoord2.x - theCoord1.x * theCoord2.z) -
      y * (theCoord1.y * theCoord2.z - theCoord1.z * theCoord2.y);
  x = aXresult;
  y = anYresult;
}

//=======================================================================
//function : DotCross
// purpose :
//=======================================================================
inline double gp_XYZ::DotCross (const gp_XYZ& theCoord1, const gp_XYZ& theCoord2) const
{
  double aXresult = theCoord1.y * theCoord2.z - theCoord1.z * theCoord2.y;
  double anYresult = theCoord1.z * theCoord2.x - theCoord1.x * theCoord2.z;
  double aZresult = theCoord1.x * theCoord2.y - theCoord1.y * theCoord2.x;
  return (x * aXresult + y * anYresult + z * aZresult);
}

//=======================================================================
//function : Multiply
// purpose :
//=======================================================================
inline void gp_XYZ::Multiply (const gp_Mat& theMatrix)
{
  double aXresult = theMatrix.Value (1, 1) * x + theMatrix.Value (1, 2) * y + theMatrix.Value (1, 3) * z;
  double anYresult = theMatrix.Value (2, 1) * x + theMatrix.Value (2, 2) * y + theMatrix.Value (2, 3) * z;
  z = theMatrix.Value (3, 1) * x + theMatrix.Value (3, 2) * y + theMatrix.Value (3, 3) * z;
  x = aXresult;
  y = anYresult;
}

//=======================================================================
//function : Normalize
// purpose :
//=======================================================================
inline void gp_XYZ::Normalize()
{
  double aD = Modulus();
  x = x / aD;
  y = y / aD;
  z = z / aD;
}

//=======================================================================
//function : operator*
// purpose :
//=======================================================================
inline gp_XYZ operator* (const gp_Mat& theMatrix, const gp_XYZ& theCoord1)
{
  return theCoord1.Multiplied (theMatrix);
}

//=======================================================================
//function : operator*
// purpose :
//=======================================================================
inline gp_XYZ operator* (const double theScalar, const gp_XYZ& theCoord1)
{
  return theCoord1.Multiplied (theScalar);
}

}
}

#endif // _gp_XYZ_HeaderFile
