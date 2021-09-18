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

#ifndef _gp_Mat2d_HeaderFile
#define _gp_Mat2d_HeaderFile

#include <mobius/gp.hxx>

class gp_Trsf2d;
class gp_GTrsf2d;
class gp_XY;

//! Describes a two-column, two-row matrix.
//! This sort of object is used in various vectorial or matrix computations.
class gp_Mat2d 
{
public:

  //! Creates  a matrix with null coefficients.
  gp_Mat2d()
  {
    myMat[0][0] = myMat[0][1] = myMat[1][0] = myMat[1][1] = 0.0;
  }

  //! theCol1, theCol2 are the 2 columns of the matrix.
  mobiusGeom_EXPORT gp_Mat2d (const gp_XY& theCol1, const gp_XY& theCol2);

  //! Assigns the two coordinates of theValue to the column of range
  //! theCol of this matrix
  //! Raises OutOfRange if theCol < 1 or theCol > 2.
  mobiusGeom_EXPORT void SetCol (const int theCol, const gp_XY& theValue);

  //! Assigns the number pairs theCol1, theCol2 to the two columns of   this matrix
  mobiusGeom_EXPORT void SetCols (const gp_XY& theCol1, const gp_XY& theCol2);

  //! Modifies the main diagonal of the matrix.
  //! @code
  //! <me>.Value (1, 1) = theX1
  //! <me>.Value (2, 2) = theX2
  //! @endcode
  //! The other coefficients of the matrix are not modified.
  void SetDiagonal (const double theX1, const double theX2)
  {
    myMat[0][0] = theX1;
    myMat[1][1] = theX2;
  }

  //! Modifies this matrix, so that it represents the Identity matrix.
  void SetIdentity()
  {
    myMat[0][0] = myMat[1][1] = 1.0;
    myMat[0][1] = myMat[1][0] = 0.0;
  }

  //! Modifies this matrix, so that it represents a rotation. theAng is the angular
  //! value in radian of the rotation.
  void SetRotation (const double theAng);

  //! Assigns the two coordinates of theValue to the row of index theRow of this matrix.
  //! Raises OutOfRange if theRow < 1 or theRow > 2.
  mobiusGeom_EXPORT void SetRow (const int theRow, const gp_XY& theValue);

  //! Assigns the number pairs theRow1, theRow2 to the two rows of this matrix.
  mobiusGeom_EXPORT void SetRows (const gp_XY& theRow1, const gp_XY& theRow2);

  //! Modifies the matrix such that it
  //! represents a scaling transformation, where theS is the scale   factor :
  //! @code
  //!         | theS    0.0 |
  //! <me> =  | 0.0   theS  |
  //! @endcode
  void SetScale (const double theS)
  {
    myMat[0][0] = myMat[1][1] = theS;
    myMat[0][1] = myMat[1][0] = 0.0;
  }

  //! Assigns <theValue> to the coefficient of row theRow, column theCol of this matrix.
  //! Raises OutOfRange if theRow < 1 or theRow > 2 or theCol < 1 or theCol > 2
  void SetValue (const int theRow, const int theCol, const double theValue)
  {
    myMat[theRow - 1][theCol - 1] = theValue;
  }

  //! Returns the column of theCol index.
  //! Raises OutOfRange if theCol < 1 or theCol > 2
  mobiusGeom_EXPORT gp_XY Column (const int theCol) const;

  //! Computes the determinant of the matrix.
  double Determinant() const
  {
    return myMat[0][0] * myMat[1][1] - myMat[1][0] * myMat[0][1];
  }

  //! Returns the main diagonal of the matrix.
  mobiusGeom_EXPORT gp_XY Diagonal() const;

  //! Returns the row of index theRow.
  //! Raised if theRow < 1 or theRow > 2
  mobiusGeom_EXPORT gp_XY Row (const int theRow) const;

  //! Returns the coefficient of range (ttheheRow, theCol)
  //! Raises OutOfRange
  //! if theRow < 1 or theRow > 2 or theCol < 1 or theCol > 2
  const double& Value (const int theRow, const int theCol) const
  {
    return myMat[theRow - 1][theCol - 1];
  }

  const double& operator() (const int theRow, const int theCol) const { return Value (theRow, theCol); }

  //! Returns the coefficient of range (theRow, theCol)
  //! Raises OutOfRange
  //! if theRow < 1 or theRow > 2 or theCol < 1 or theCol > 2
  double& ChangeValue (const int theRow, const int theCol)
  {
    return myMat[theRow - 1][theCol - 1];
  }

  double& operator() (const int theRow, const int theCol) { return ChangeValue (theRow, theCol); }

  //! Returns true if this matrix is singular (and therefore, cannot be inverted).
  //! The Gauss LU decomposition is used to invert the matrix
  //! so the matrix is considered as singular if the largest
  //! pivot found is lower or equal to Resolution from gp.
  bool IsSingular() const
  {
    double aDet = Determinant();
    if (aDet < 0)
    {
      aDet = -aDet;
    }
    return aDet <= gp::Resolution();
  }

  void Add (const gp_Mat2d& Other);

  void operator += (const gp_Mat2d& theOther) { Add (theOther); }

  //! Computes the sum of this matrix and the matrix
  //! theOther.for each coefficient of the matrix :
  //! @code
  //! <me>.Coef(i,j) + <theOther>.Coef(i,j)
  //! @endcode
  //! Note:
  //! -   operator += assigns the result to this matrix, while
  //! -   operator + creates a new one.
  mobiusCore_NODISCARD gp_Mat2d Added (const gp_Mat2d& theOther) const;

  mobiusCore_NODISCARD gp_Mat2d operator + (const gp_Mat2d& theOther) const { return Added (theOther); }

  void Divide (const double theScalar);

  void operator /= (const double theScalar) { Divide (theScalar); }

  //! Divides all the coefficients of the matrix by a scalar.
  mobiusCore_NODISCARD gp_Mat2d Divided (const double theScalar) const;

  mobiusCore_NODISCARD gp_Mat2d operator / (const double theScalar) const { return Divided (theScalar); }

  mobiusGeom_EXPORT void Invert();

  //! Inverses the matrix and raises exception if the matrix
  //! is singular.
  mobiusCore_NODISCARD gp_Mat2d Inverted() const
  {
    gp_Mat2d aNewMat = *this;
    aNewMat.Invert();
    return aNewMat;
  }

  mobiusCore_NODISCARD gp_Mat2d Multiplied (const gp_Mat2d& theOther) const
  {
    gp_Mat2d aNewMat2d = *this;
    aNewMat2d.Multiply (theOther);
    return aNewMat2d;
  }

  mobiusCore_NODISCARD gp_Mat2d operator * (const gp_Mat2d& theOther) const { return Multiplied (theOther); }

  //! Computes the product of two matrices <me> * <theOther>
  void Multiply (const gp_Mat2d& theOther);

  //! Modifies this matrix by premultiplying it by the matrix Other
  //! <me> = theOther * <me>.
  void PreMultiply (const gp_Mat2d& theOther);

  mobiusCore_NODISCARD gp_Mat2d Multiplied (const double theScalar) const;

  mobiusCore_NODISCARD gp_Mat2d operator * (const double theScalar) const { return Multiplied (theScalar); }

  //! Multiplies all the coefficients of the matrix by a scalar.
  void Multiply (const double theScalar);

  void operator *= (const double theScalar) { Multiply (theScalar); }

  mobiusGeom_EXPORT void Power (const int theN);

  //! computes <me> = <me> * <me> * .......* <me>, theN time.
  //! if theN = 0 <me> = Identity
  //! if theN < 0 <me> = <me>.Invert() *...........* <me>.Invert().
  //! If theN < 0 an exception can be raised if the matrix is not
  //! inversible
  mobiusCore_NODISCARD gp_Mat2d Powered (const int theN) const
  {
    gp_Mat2d aMat2dN = *this;
    aMat2dN.Power (theN);
    return aMat2dN;
  }

  void Subtract (const gp_Mat2d& theOther);

  void operator -= (const gp_Mat2d& theOther) { Subtract (theOther); }

  //! Computes for each coefficient of the matrix :
  //! @code
  //! <me>.Coef(i,j) - <theOther>.Coef(i,j)
  //! @endcode
  mobiusCore_NODISCARD gp_Mat2d Subtracted (const gp_Mat2d& theOther) const;

  mobiusCore_NODISCARD gp_Mat2d operator - (const gp_Mat2d& theOther) const { return Subtracted (theOther); }

  void Transpose();

  //! Transposes the matrix. A(j, i) -> A (i, j)
  mobiusCore_NODISCARD gp_Mat2d Transposed() const;

friend class gp_Trsf2d;
friend class gp_GTrsf2d;
friend class gp_XY;

private:

  double myMat[2][2];

};

//=======================================================================
//function : SetRotation
// purpose :
//=======================================================================
inline void gp_Mat2d::SetRotation (const double theAng)
{
  double aSinA = sin (theAng);
  double aCosA = cos (theAng);
  myMat[0][0] = myMat[1][1] = aCosA;
  myMat[0][1] = -aSinA;
  myMat[1][0] =  aSinA;
}

//=======================================================================
//function : Add
// purpose :
//=======================================================================
inline void gp_Mat2d::Add (const gp_Mat2d& theOther)
{
  myMat[0][0] += theOther.myMat[0][0];
  myMat[0][1] += theOther.myMat[0][1];
  myMat[1][0] += theOther.myMat[1][0];
  myMat[1][1] += theOther.myMat[1][1];
}

//=======================================================================
//function : Added
// purpose :
//=======================================================================
inline gp_Mat2d gp_Mat2d::Added (const gp_Mat2d& theOther) const
{
  gp_Mat2d aNewMat2d;
  aNewMat2d.myMat[0][0] = myMat[0][0] + theOther.myMat[0][0];
  aNewMat2d.myMat[0][1] = myMat[0][1] + theOther.myMat[0][1];
  aNewMat2d.myMat[1][0] = myMat[1][0] + theOther.myMat[1][0];
  aNewMat2d.myMat[1][1] = myMat[1][1] + theOther.myMat[1][1];
  return aNewMat2d;
}

//=======================================================================
//function : Divide
// purpose :
//=======================================================================
inline void gp_Mat2d::Divide (const double theScalar)
{
  myMat[0][0] /= theScalar;
  myMat[0][1] /= theScalar;
  myMat[1][0] /= theScalar;
  myMat[1][1] /= theScalar;
}

//=======================================================================
//function : Divided
// purpose :
//=======================================================================
inline gp_Mat2d gp_Mat2d::Divided (const double theScalar) const
{
  gp_Mat2d aNewMat2d;
  aNewMat2d.myMat[0][0] = myMat[0][0] / theScalar;
  aNewMat2d.myMat[0][1] = myMat[0][1] / theScalar;
  aNewMat2d.myMat[1][0] = myMat[1][0] / theScalar;
  aNewMat2d.myMat[1][1] = myMat[1][1] / theScalar;
  return aNewMat2d;
}

//=======================================================================
//function : Multiply
// purpose :
//=======================================================================
inline void gp_Mat2d::Multiply (const gp_Mat2d& theOther)
{
  const double aT00 = myMat[0][0] * theOther.myMat[0][0] + myMat[0][1] * theOther.myMat[1][0];
  const double aT10 = myMat[1][0] * theOther.myMat[0][0] + myMat[1][1] * theOther.myMat[1][0];
  myMat[0][1] = myMat[0][0] * theOther.myMat[0][1] + myMat[0][1] * theOther.myMat[1][1];
  myMat[1][1] = myMat[1][0] * theOther.myMat[0][1] + myMat[1][1] * theOther.myMat[1][1];
  myMat[0][0] = aT00;
  myMat[1][0] = aT10;
}

//=======================================================================
//function : PreMultiply
// purpose :
//=======================================================================
inline void gp_Mat2d::PreMultiply (const gp_Mat2d& theOther)
{
  const double aT00 = theOther.myMat[0][0] * myMat[0][0]
                           + theOther.myMat[0][1] * myMat[1][0];
  myMat[1][0] = theOther.myMat[1][0] * myMat[0][0]
              + theOther.myMat[1][1] * myMat[1][0];
  const double aT01 = theOther.myMat[0][0] * myMat[0][1]
                           + theOther.myMat[0][1] * myMat[1][1];
  myMat[1][1] = theOther.myMat[1][0] * myMat[0][1]
              + theOther.myMat[1][1] * myMat[1][1];
  myMat[0][0] = aT00;
  myMat[0][1] = aT01;
}

//=======================================================================
//function : Multiplied
// purpose :
//=======================================================================
inline gp_Mat2d gp_Mat2d::Multiplied (const double theScalar) const
{
  gp_Mat2d aNewMat2d;
  aNewMat2d.myMat[0][0] = myMat[0][0] * theScalar;
  aNewMat2d.myMat[0][1] = myMat[0][1] * theScalar;
  aNewMat2d.myMat[1][0] = myMat[1][0] * theScalar;
  aNewMat2d.myMat[1][1] = myMat[1][1] * theScalar;
  return aNewMat2d;
}

//=======================================================================
//function : Multiply
// purpose :
//=======================================================================
inline void gp_Mat2d::Multiply (const double theScalar)
{
  myMat[0][0] *= theScalar;
  myMat[0][1] *= theScalar;
  myMat[1][0] *= theScalar;
  myMat[1][1] *= theScalar;
}

//=======================================================================
//function : Subtract
// purpose :
//=======================================================================
inline void gp_Mat2d::Subtract (const gp_Mat2d& theOther)
{
  myMat[0][0] -= theOther.myMat[0][0];
  myMat[0][1] -= theOther.myMat[0][1];
  myMat[1][0] -= theOther.myMat[1][0];
  myMat[1][1] -= theOther.myMat[1][1];
}

//=======================================================================
//function : Subtracted
// purpose :
//=======================================================================
inline gp_Mat2d gp_Mat2d::Subtracted (const gp_Mat2d& theOther) const
{
  gp_Mat2d aNewMat2d;
  aNewMat2d.myMat[0][0] = myMat[0][0] - theOther.myMat[0][0];
  aNewMat2d.myMat[0][1] = myMat[0][1] - theOther.myMat[0][1];
  aNewMat2d.myMat[1][0] = myMat[1][0] - theOther.myMat[1][0];
  aNewMat2d.myMat[1][1] = myMat[1][1] - theOther.myMat[1][1];
  return aNewMat2d;
}

//=======================================================================
//function : Transpose
// purpose :
//=======================================================================
inline void gp_Mat2d::Transpose()
{
  const double aTemp = myMat[0][1];
  myMat[0][1] = myMat[1][0];
  myMat[1][0] = aTemp;
}

//=======================================================================
//function : Transposed
// purpose :
//=======================================================================
inline gp_Mat2d gp_Mat2d::Transposed() const
{
  gp_Mat2d aNewMat2d;
  aNewMat2d.myMat[1][0] = myMat[0][1];
  aNewMat2d.myMat[0][1] = myMat[1][0];
  aNewMat2d.myMat[0][0] = myMat[0][0];
  aNewMat2d.myMat[1][1] = myMat[1][1];
  return aNewMat2d; 
}

//=======================================================================
//function : operator*
// purpose :
//=======================================================================
inline gp_Mat2d operator* (const double theScalar, const gp_Mat2d& theMat2D)
{
  return theMat2D.Multiplied (theScalar);
}

#endif // _gp_Mat2d_HeaderFile
