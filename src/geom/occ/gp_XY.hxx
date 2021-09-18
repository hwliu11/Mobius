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

#ifndef _gp_XY_HeaderFile
#define _gp_XY_HeaderFile

#include <mobius/gp.hxx>
#include <mobius/gp_Mat2d.hxx>

//! This class describes a cartesian coordinate entity in 2D
//! space {X,Y}. This class is non persistent. This entity used
//! for algebraic calculation.
//! It is used in vectorial computations or for holding this type
//! of information in data structures.
class gp_XY 
{
public:

  //! Creates XY object with zero coordinates (0,0).
  gp_XY()
  : x (0.),
    y (0.)
  {}

  //! a number pair defined by the XY coordinates
  gp_XY (const double theX, const double theY)
  : x (theX),
    y (theY)
  {}

  //! modifies the coordinate of range theIndex
  //! theIndex = 1 => X is modified
  //! theIndex = 2 => Y is modified
  //! Raises OutOfRange if theIndex != {1, 2}.
  inline void SetCoord (const int theIndex, const double theXi)
  {
    (&x)[theIndex - 1] = theXi;
  }

  //! For this number pair, assigns
  //! the values theX and theY to its coordinates
  inline void SetCoord (const double theX, const double theY)
  {
    x = theX;
    y = theY;
  }

  //! Assigns the given value to the X coordinate of this number pair.
  void SetX (const double theX) { x = theX; }

  //! Assigns the given value to the Y  coordinate of this number pair.
  void SetY (const double theY) { y = theY; }

  //! returns the coordinate of range theIndex :
  //! theIndex = 1 => X is returned
  //! theIndex = 2 => Y is returned
  //! Raises OutOfRange if theIndex != {1, 2}.
  inline double Coord (const int theIndex) const
  {
    return (&x)[theIndex - 1];
  }

  inline double& ChangeCoord (const int theIndex)
  {
    return (&x)[theIndex - 1];
  }

  //! For this number pair, returns its coordinates X and Y.
  inline void Coord (double& theX, double& theY) const
  {
    theX = x;
    theY = y;
  }

  //! Returns the X coordinate of this number pair.
  double X() const { return x; }

  //! Returns the Y coordinate of this number pair.
  double Y() const { return y; }

  //! Computes Sqrt (X*X + Y*Y) where X and Y are the two coordinates of this number pair.
  double Modulus() const { return sqrt (x * x + y * y); }

  //! Computes X*X + Y*Y where X and Y are the two coordinates of this number pair.
  double SquareModulus() const { return x * x + y * y; }

  //! Returns true if the coordinates of this number pair are
  //! equal to the respective coordinates of the number pair
  //! theOther, within the specified tolerance theTolerance. I.e.:
  //! abs(<me>.X() - theOther.X()) <= theTolerance and
  //! abs(<me>.Y() - theOther.Y()) <= theTolerance and
  //! computations
  mobiusGeom_EXPORT bool IsEqual (const gp_XY& theOther, const double theTolerance) const;

  //! Computes the sum of this number pair and number pair theOther
  //! @code
  //! <me>.X() = <me>.X() + theOther.X()
  //! <me>.Y() = <me>.Y() + theOther.Y()
  inline void Add (const gp_XY& theOther)
  {
    x += theOther.x;
    y += theOther.y;
  }

  void operator+= (const gp_XY& theOther) { Add (theOther); }

  //! Computes the sum of this number pair and number pair theOther
  //! @code
  //! new.X() = <me>.X() + theOther.X()
  //! new.Y() = <me>.Y() + theOther.Y()
  //! @endcode
  mobiusCore_NODISCARD gp_XY Added (const gp_XY& theOther) const
  {
    return gp_XY (x + theOther.X(), y + theOther.Y());
  }

  mobiusCore_NODISCARD gp_XY operator+ (const gp_XY& theOther) const { return Added (theOther); }

  //! @code
  //! double D = <me>.X() * theOther.Y() - <me>.Y() * theOther.X()
  //! @endcode
  mobiusCore_NODISCARD double Crossed (const gp_XY& theOther) const { return x * theOther.y - y * theOther.x; }

  mobiusCore_NODISCARD double operator^ (const gp_XY& theOther) const { return Crossed (theOther); }

  //! computes the magnitude of the cross product between <me> and
  //! theRight. Returns || <me> ^ theRight ||
  inline double CrossMagnitude (const gp_XY& theRight) const
  {
    double aVal = x * theRight.y - y * theRight.x;
    return aVal < 0 ? -aVal : aVal;
  }

  //! computes the square magnitude of the cross product between <me> and
  //! theRight. Returns || <me> ^ theRight ||**2
  inline double CrossSquareMagnitude (const gp_XY& theRight) const
  {
    double aZresult = x * theRight.y - y * theRight.x;
    return aZresult * aZresult;
  }

  //! divides <me> by a real.
  void Divide (const double theScalar)
  {
    x /= theScalar;
    y /= theScalar;
  }

  void operator /= (const double theScalar) { Divide (theScalar); }

  //! Divides <me> by a real.
  mobiusCore_NODISCARD gp_XY Divided (const double theScalar) const
  {
    return gp_XY (x / theScalar, y / theScalar);
  }

  mobiusCore_NODISCARD gp_XY operator/ (const double theScalar) const { return Divided (theScalar); }

  //! Computes the scalar product between <me> and theOther
  double Dot (const gp_XY& theOther) const { return x * theOther.x + y * theOther.y; }

  double operator* (const gp_XY& theOther) const { return Dot (theOther); }

  //! @code
  //! <me>.X() = <me>.X() * theScalar;
  //! <me>.Y() = <me>.Y() * theScalar;
  void Multiply (const double theScalar)
  {
    x *= theScalar;
    y *= theScalar;
  }

  void operator*= (const double theScalar) { Multiply (theScalar); }

  //! @code
  //! <me>.X() = <me>.X() * theOther.X();
  //! <me>.Y() = <me>.Y() * theOther.Y();
  void Multiply (const gp_XY& theOther)
  {
    x *= theOther.x;
    y *= theOther.y;
  }

  void operator*= (const gp_XY& theOther) { Multiply (theOther); }

  //! <me> = theMatrix * <me>
  void Multiply (const gp_Mat2d& theMatrix);

  void operator*= (const gp_Mat2d& theMatrix) { Multiply (theMatrix); }

  //! @code
  //! New.X() = <me>.X() * theScalar;
  //! New.Y() = <me>.Y() * theScalar;
  mobiusCore_NODISCARD gp_XY Multiplied (const double theScalar) const { return gp_XY (x * theScalar, y * theScalar); }

  mobiusCore_NODISCARD gp_XY operator*  (const double theScalar) const { return Multiplied (theScalar); }
  //! @code
  //! new.X() = <me>.X() * theOther.X();
  //! new.Y() = <me>.Y() * theOther.Y();
  mobiusCore_NODISCARD gp_XY Multiplied (const gp_XY& theOther) const { return gp_XY (x * theOther.X(), y * theOther.Y()); }

  //! New = theMatrix * <me>
  //! @endcode
  mobiusCore_NODISCARD gp_XY Multiplied (const gp_Mat2d& theMatrix) const
  {
    return gp_XY (theMatrix.Value (1, 1) * x + theMatrix.Value (1, 2) * y,
                  theMatrix.Value (2, 1) * x + theMatrix.Value (2, 2) * y);
  }

  mobiusCore_NODISCARD gp_XY operator*  (const gp_Mat2d& theMatrix) const { return Multiplied (theMatrix); }
  //! @code
  //! <me>.X() = <me>.X()/ <me>.Modulus()
  //! <me>.Y() = <me>.Y()/ <me>.Modulus()
  //! @endcode
  //! Raises ConstructionError if <me>.Modulus() <= Resolution from gp
  void Normalize();

  //! @code
  //! New.X() = <me>.X()/ <me>.Modulus()
  //! New.Y() = <me>.Y()/ <me>.Modulus()
  //! @endcode
  //! Raises ConstructionError if <me>.Modulus() <= Resolution from gp
  mobiusCore_NODISCARD gp_XY Normalized() const
  {
    double aD = Modulus();
    return gp_XY (x / aD, y / aD);
  }

  //! @code
  //! <me>.X() = -<me>.X()
  //! <me>.Y() = -<me>.Y()
  inline void Reverse()
  {
    x = -x;
    y = -y;
  }

  //! @code
  //! New.X() = -<me>.X()
  //! New.Y() = -<me>.Y()
  mobiusCore_NODISCARD gp_XY Reversed() const
  {
    gp_XY aCoord2D = *this;
    aCoord2D.Reverse();
    return aCoord2D;
  }

  mobiusCore_NODISCARD gp_XY operator-() const { return Reversed(); }

  //! Computes  the following linear combination and
  //! assigns the result to this number pair:
  //! @code
  //! theA1 * theXY1 + theA2 * theXY2
  inline void SetLinearForm (const double theA1, const gp_XY& theXY1,
                             const double theA2, const gp_XY& theXY2)
  {
    x = theA1 * theXY1.x + theA2 * theXY2.x;
    y = theA1 * theXY1.y + theA2 * theXY2.y;
  }

  //! --  Computes  the following linear combination and
  //! assigns the result to this number pair:
  //! @code
  //! theA1 * theXY1 + theA2 * theXY2 + theXY3
  inline void SetLinearForm (const double theA1, const gp_XY& theXY1,
                             const double theA2, const gp_XY& theXY2,
                             const gp_XY& theXY3)
  {
    x = theA1 * theXY1.x + theA2 * theXY2.x + theXY3.x;
    y = theA1 * theXY1.y + theA2 * theXY2.y + theXY3.y;
  }

  //! Computes  the following linear combination and
  //! assigns the result to this number pair:
  //! @code
  //! theA1 * theXY1 + theXY2
  inline void SetLinearForm (const double theA1, const gp_XY& theXY1,
                             const gp_XY& theXY2)
  {
    x = theA1 * theXY1.x + theXY2.x;
    y = theA1 * theXY1.y + theXY2.y;
  }

  //! Computes  the following linear combination and
  //! assigns the result to this number pair:
  //! @code
  //! theXY1 + theXY2
  inline void SetLinearForm (const gp_XY& theXY1,
                             const gp_XY& theXY2)
  {
    x = theXY1.x + theXY2.x;
    y = theXY1.y + theXY2.y;
  }

  //! @code
  //! <me>.X() = <me>.X() - theOther.X()
  //! <me>.Y() = <me>.Y() - theOther.Y()
  inline void Subtract (const gp_XY& theOther)
  {
    x -= theOther.x;
    y -= theOther.y;
  }

  void operator-= (const gp_XY& theOther) { Subtract (theOther); }

  //! @code
  //! new.X() = <me>.X() - theOther.X()
  //! new.Y() = <me>.Y() - theOther.Y()
  //! @endcode
  mobiusCore_NODISCARD gp_XY Subtracted (const gp_XY& theOther) const
  {
    gp_XY aCoord2D = *this;
    aCoord2D.Subtract (theOther);
    return aCoord2D;
  }

  mobiusCore_NODISCARD gp_XY operator-  (const gp_XY& theOther) const { return Subtracted (theOther); }

private:

  double x;
  double y;

};

//=======================================================================
//function :  Multiply
// purpose :
//=======================================================================
inline void gp_XY::Multiply (const gp_Mat2d& theMatrix)
{
  double aXresult = theMatrix.Value (1, 1) * x + theMatrix.Value (1, 2) * y;
  y = theMatrix.Value (2, 1) * x + theMatrix.Value (2, 2) * y;
  x = aXresult;
}

//=======================================================================
//function :  Normalize
// purpose :
//=======================================================================
inline void gp_XY::Normalize()
{
  double aD = Modulus();
  x = x / aD;
  y = y / aD;
}

//=======================================================================
//function :  operator*
// purpose :
//=======================================================================
inline gp_XY operator* (const gp_Mat2d& theMatrix,
                        const gp_XY&    theCoord1)
{
  return theCoord1.Multiplied (theMatrix);
}

//=======================================================================
//function :  operator*
// purpose :
//=======================================================================
inline gp_XY operator* (const double theScalar,
                        const gp_XY&        theCoord1)
{
  return theCoord1.Multiplied (theScalar);
}

#endif // _gp_XY_HeaderFile
