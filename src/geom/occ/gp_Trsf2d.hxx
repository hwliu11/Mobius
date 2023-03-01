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

#ifndef _gp_Trsf2d_HeaderFile
#define _gp_Trsf2d_HeaderFile

#include <mobius/gp_TrsfForm.hxx>
#include <mobius/gp_Mat2d.hxx>
#include <mobius/gp_XY.hxx>

namespace mobius {
namespace occ {

class gp_GTrsf2d;
class gp_Trsf;
class gp_Pnt2d;
class gp_Ax2d;
class gp_Vec2d;
class gp_XY;
class gp_Mat2d;

//! Defines a non-persistent transformation in 2D space.
//! The following transformations are implemented :
//! - Translation, Rotation, Scale
//! - Symmetry with respect to a point and a line.
//! Complex transformations can be obtained by combining the
//! previous elementary transformations using the method Multiply.
//! The transformations can be represented as follow :
//! @code
//!    V1   V2   T       XY        XY
//! | a11  a12  a13 |   | x |     | x'|
//! | a21  a22  a23 |   | y |     | y'|
//! |  0    0    1  |   | 1 |     | 1 |
//! @endcode
//! where {V1, V2} defines the vectorial part of the transformation
//! and T defines the translation part of the transformation.
//! This transformation never change the nature of the objects.
class gp_Trsf2d 
{
public:

  //! Returns identity transformation.
  mobiusGeom_EXPORT gp_Trsf2d();

  //! Creates a 2d transformation in the XY plane from a
  //! 3d transformation .
  mobiusGeom_EXPORT gp_Trsf2d (const gp_Trsf& theT);

  //! Changes the transformation into a symmetrical transformation.
  //! theP is the center of the symmetry.
  mobiusGeom_EXPORT void SetMirror (const gp_Pnt2d& theP);

  //! Changes the transformation into a symmetrical transformation.
  //! theA is the center of the axial symmetry.
  mobiusGeom_EXPORT void SetMirror (const gp_Ax2d& theA);

  //! Changes the transformation into a rotation.
  //! theP is the rotation's center and theAng is the angular value of the
  //! rotation in radian.
  mobiusGeom_EXPORT void SetRotation (const gp_Pnt2d& theP, const double theAng);

  //! Changes the transformation into a scale.
  //! theP is the center of the scale and theS is the scaling value.
  mobiusGeom_EXPORT void SetScale (const gp_Pnt2d& theP, const double theS);

  //! Changes a transformation allowing passage from the coordinate
  //! system "theFromSystem1" to the coordinate system "theToSystem2".
  mobiusGeom_EXPORT void SetTransformation (const gp_Ax2d& theFromSystem1, const gp_Ax2d& theToSystem2);

  //! Changes the transformation allowing passage from the basic
  //! coordinate system
  //! {P(0.,0.,0.), VX (1.,0.,0.), VY (0.,1.,0.)}
  //! to the local coordinate system defined with the Ax2d theToSystem.
  mobiusGeom_EXPORT void SetTransformation (const gp_Ax2d& theToSystem);

  //! Changes the transformation into a translation.
  //! theV is the vector of the translation.
  mobiusGeom_EXPORT void SetTranslation (const gp_Vec2d& theV);

  //! Makes the transformation into a translation from
  //! the point theP1 to the point theP2.
  mobiusGeom_EXPORT void SetTranslation (const gp_Pnt2d& theP1, const gp_Pnt2d& theP2);

  //! Replaces the translation vector with theV.
  mobiusGeom_EXPORT void SetTranslationPart (const gp_Vec2d& theV);

  //! Modifies the scale factor.
  mobiusGeom_EXPORT void SetScaleFactor (const double theS);

  //! Returns true if the determinant of the vectorial part of
  //! this transformation is negative..
  bool IsNegative() const { return (matrix.Determinant() < 0.0); }

  //! Returns the nature of the transformation. It can be  an
  //! identity transformation, a rotation, a translation, a mirror
  //! (relative to a point or an axis), a scaling transformation,
  //! or a compound transformation.
  gp_TrsfForm Form() const { return shape; }

  //! Returns the scale factor.
  double ScaleFactor() const { return scale; }

  //! Returns the translation part of the transformation's matrix
  const gp_XY& TranslationPart() const { return loc; }

  //! Returns the vectorial part of the transformation. It is a
  //! 2*2 matrix which includes the scale factor.
  mobiusGeom_EXPORT gp_Mat2d VectorialPart() const;

  //! Returns the homogeneous vectorial part of the transformation.
  //! It is a 2*2 matrix which doesn't include the scale factor.
  //! The coefficients of this matrix must be multiplied by the
  //! scale factor to obtain the coefficients of the transformation.
  const gp_Mat2d& HVectorialPart() const { return matrix; }

  //! Returns the angle corresponding to the rotational component
  //! of the transformation matrix (operation opposite to SetRotation()).
  mobiusGeom_EXPORT double RotationPart() const;

  //! Returns the coefficients of the transformation's matrix.
  //! It is a 2 rows * 3 columns matrix.
  //! Raises OutOfRange if theRow < 1 or theRow > 2 or theCol < 1 or theCol > 3
  mobiusGeom_EXPORT double Value (const int theRow, const int theCol) const;

  mobiusGeom_EXPORT void Invert();

  //! Computes the reverse transformation.
  //! Raises an exception if the matrix of the transformation
  //! is not inversible, it means that the scale factor is lower
  //! or equal to Resolution from package gp.
  mobiusCore_NODISCARD gp_Trsf2d Inverted() const
  {
    gp_Trsf2d aT = *this;
    aT.Invert();
    return aT;
  }

  mobiusCore_NODISCARD gp_Trsf2d Multiplied (const gp_Trsf2d& theT) const
  {
    gp_Trsf2d aTresult (*this);
    aTresult.Multiply (theT);
    return aTresult;
  }

  mobiusCore_NODISCARD gp_Trsf2d operator * (const gp_Trsf2d& theT) const { return Multiplied (theT); }

  //! Computes the transformation composed from <me> and theT.
  //! <me> = <me> * theT
  mobiusGeom_EXPORT void Multiply (const gp_Trsf2d& theT);

  void operator *= (const gp_Trsf2d& theT) { Multiply (theT); }

  //! Computes the transformation composed from <me> and theT.
  //! <me> = theT * <me>
  mobiusGeom_EXPORT void PreMultiply (const gp_Trsf2d& theT);

  mobiusGeom_EXPORT void Power (const int theN);

  //! Computes the following composition of transformations
  //! <me> * <me> * .......* <me>,  theN time.
  //! if theN = 0 <me> = Identity
  //! if theN < 0 <me> = <me>.Inverse() *...........* <me>.Inverse().
  //!
  //! Raises if theN < 0 and if the matrix of the transformation not
  //! inversible.
  gp_Trsf2d Powered (const int theN)
  {
    gp_Trsf2d aT = *this;
    aT.Power (theN);
    return aT;
  }

  mobiusGeom_EXPORT void Transforms (double& theX, double& theY) const;

  //! Transforms  a doublet XY with a Trsf2d
  mobiusGeom_EXPORT void Transforms (gp_XY& theCoord) const;

  //! Sets the coefficients  of the transformation. The
  //! transformation  of the  point  x,y is  the point
  //! x',y' with :
  //! @code
  //! x' = a11 x + a12 y + a13
  //! y' = a21 x + a22 y + a23
  //! @endcode
  //! The method Value(i,j) will return aij.
  //! Raises ConstructionError if the determinant of the aij is null.
  //! If the matrix as not a uniform scale it will be orthogonalized before future using.
  mobiusGeom_EXPORT void SetValues (const double a11, const double a12, const double a13, const double a21, const double a22, const double a23);

friend class gp_GTrsf2d;

protected:

  //! Makes orthogonalization of "matrix"
  mobiusGeom_EXPORT void Orthogonalize();

private:

  double scale;
  gp_TrsfForm shape;
  gp_Mat2d matrix;
  gp_XY loc;

};

}
}

#endif // _gp_Trsf2d_HeaderFile
