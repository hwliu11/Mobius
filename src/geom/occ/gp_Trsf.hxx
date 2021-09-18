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

#ifndef _gp_Trsf_HeaderFile
#define _gp_Trsf_HeaderFile

#include <mobius/gp_TrsfForm.hxx>
#include <mobius/gp_Mat.hxx>
#include <mobius/gp_XYZ.hxx>

class gp_Pnt;
class gp_Trsf2d;
class gp_Ax1;
class gp_Ax2;
class gp_Quaternion;
class gp_Ax3;
class gp_Vec;

// Avoid possible conflict with SetForm macro defined by windows.h
#ifdef SetForm
#undef SetForm
#endif

//! Defines a non-persistent transformation in 3D space.
//! The following transformations are implemented :
//! . Translation, Rotation, Scale
//! . Symmetry with respect to a point, a line, a plane.
//! Complex transformations can be obtained by combining the
//! previous elementary transformations using the method
//! Multiply.
//! The transformations can be represented as follow :
//! @code
//!    V1   V2   V3    T       XYZ        XYZ
//! | a11  a12  a13   a14 |   | x |      | x'|
//! | a21  a22  a23   a24 |   | y |      | y'|
//! | a31  a32  a33   a34 |   | z |   =  | z'|
//! |  0    0    0     1  |   | 1 |      | 1 |
//! @endcode
//! where {V1, V2, V3} defines the vectorial part of the
//! transformation and T defines the translation part of the
//! transformation.
//! This transformation never change the nature of the objects.
class gp_Trsf 
{
public:

  //! Returns the identity transformation.
  gp_Trsf();

  //! Creates  a 3D transformation from the 2D transformation theT.
  //! The resulting transformation has a homogeneous
  //! vectorial part, V3, and a translation part, T3, built from theT:
  //! a11    a12
  //! 0             a13
  //! V3 =    a21    a22    0       T3
  //! =   a23
  //! 0    0    1.
  //! 0
  //! It also has the same scale factor as theT. This
  //! guarantees (by projection) that the transformation
  //! which would be performed by theT in a plane (2D space)
  //! is performed by the resulting transformation in the xOy
  //! plane of the 3D space, (i.e. in the plane defined by the
  //! origin (0., 0., 0.) and the vectors DX (1., 0., 0.), and DY
  //! (0., 1., 0.)). The scale factor is applied to the entire space.
  mobiusGeom_EXPORT gp_Trsf (const gp_Trsf2d& theT);

  //! Makes the transformation into a symmetrical transformation.
  //! theP is the center of the symmetry.
  void SetMirror (const gp_Pnt& theP);

  //! Makes the transformation into a symmetrical transformation.
  //! theA1 is the center of the axial symmetry.
  mobiusGeom_EXPORT void SetMirror (const gp_Ax1& theA1);

  //! Makes the transformation into a symmetrical transformation.
  //! theA2 is the center of the planar symmetry
  //! and defines the plane of symmetry by its origin, "X
  //! Direction" and "Y Direction".
  mobiusGeom_EXPORT void SetMirror (const gp_Ax2& theA2);

  //! Changes the transformation into a rotation.
  //! theA1 is the rotation axis and theAng is the angular value of the
  //! rotation in radians.
  mobiusGeom_EXPORT void SetRotation (const gp_Ax1& theA1, const double theAng);

  //! Changes the transformation into a rotation defined by quaternion.
  //! Note that rotation is performed around origin, i.e.
  //! no translation is involved.
  mobiusGeom_EXPORT void SetRotation (const gp_Quaternion& theR);

  //! Replaces the rotation part with specified quaternion.
  mobiusGeom_EXPORT void SetRotationPart (const gp_Quaternion& theR);

  //! Changes the transformation into a scale.
  //! theP is the center of the scale and theS is the scaling value.
  //! Raises ConstructionError  If <theS> is null.
  mobiusGeom_EXPORT void SetScale (const gp_Pnt& theP, const double theS);

  //! Modifies this transformation so that it transforms the
  //! coordinate system defined by theFromSystem1 into the
  //! one defined by theToSystem2. After this modification, this
  //! transformation transforms:
  //! -   the origin of theFromSystem1 into the origin of theToSystem2,
  //! -   the "X Direction" of theFromSystem1 into the "X
  //! Direction" of theToSystem2,
  //! -   the "Y Direction" of theFromSystem1 into the "Y
  //! Direction" of theToSystem2, and
  //! -   the "main Direction" of theFromSystem1 into the "main
  //! Direction" of theToSystem2.
  //! Warning
  //! When you know the coordinates of a point in one
  //! coordinate system and you want to express these
  //! coordinates in another one, do not use the
  //! transformation resulting from this function. Use the
  //! transformation that results from SetTransformation instead.
  //! SetDisplacement and SetTransformation create
  //! related transformations: the vectorial part of one is the
  //! inverse of the vectorial part of the other.
  mobiusGeom_EXPORT void SetDisplacement (const gp_Ax3& theFromSystem1, const gp_Ax3& theToSystem2);

  //! Modifies this transformation so that it transforms the
  //! coordinates of any point, (x, y, z), relative to a source
  //! coordinate system into the coordinates (x', y', z') which
  //! are relative to a target coordinate system, but which
  //! represent the same point
  //! The transformation is from the coordinate
  //! system "theFromSystem1" to the coordinate system "theToSystem2".
  //! Example :
  //! @code
  //! gp_Ax3 theFromSystem1, theToSystem2;
  //! double x1, y1, z1;  // are the coordinates of a point in the local system theFromSystem1
  //! double x2, y2, z2;  // are the coordinates of a point in the local system theToSystem2
  //! gp_Pnt P1 (x1, y1, z1)
  //! gp_Trsf T;
  //! T.SetTransformation (theFromSystem1, theToSystem2);
  //! gp_Pnt P2 = P1.Transformed (T);
  //! P2.Coord (x2, y2, z2);
  //! @endcode
  mobiusGeom_EXPORT void SetTransformation (const gp_Ax3& theFromSystem1, const gp_Ax3& theToSystem2);

  //! Modifies this transformation so that it transforms the
  //! coordinates of any point, (x, y, z), relative to a source
  //! coordinate system into the coordinates (x', y', z') which
  //! are relative to a target coordinate system, but which
  //! represent the same point
  //! The transformation is from the default coordinate system
  //! @code
  //! {P(0.,0.,0.), VX (1.,0.,0.), VY (0.,1.,0.), VZ (0., 0. ,1.) }
  //! @endcode
  //! to the local coordinate system defined with the Ax3 theToSystem.
  //! Use in the same way  as the previous method. FromSystem1 is
  //! defaulted to the absolute coordinate system.
  mobiusGeom_EXPORT void SetTransformation (const gp_Ax3& theToSystem);

  //! Sets transformation by directly specified rotation and translation.
  mobiusGeom_EXPORT void SetTransformation (const gp_Quaternion& R, const gp_Vec& theT);

  //! Changes the transformation into a translation.
  //! theV is the vector of the translation.
  void SetTranslation (const gp_Vec& theV);

  //! Makes the transformation into a translation where the translation vector
  //! is the vector (theP1, theP2) defined from point theP1 to point theP2.
  void SetTranslation (const gp_Pnt& theP1, const gp_Pnt& theP2);

  //! Replaces the translation vector with the vector theV.
  mobiusGeom_EXPORT void SetTranslationPart (const gp_Vec& theV);

  //! Modifies the scale factor.
  //! Raises ConstructionError  If theS is null.
  mobiusGeom_EXPORT void SetScaleFactor (const double theS);

  void SetForm (const gp_TrsfForm theP) { shape = theP; }

  //! Sets the coefficients  of the transformation.  The
  //! transformation  of the  point  x,y,z is  the point
  //! x',y',z' with :
  //! @code
  //! x' = a11 x + a12 y + a13 z + a14
  //! y' = a21 x + a22 y + a23 z + a24
  //! z' = a31 x + a32 y + a33 z + a34
  //! @endcode
  //! The method Value(i,j) will return aij.
  //! Raises ConstructionError if the determinant of  the aij is null.
  //! The matrix is orthogonalized before future using.
  mobiusGeom_EXPORT void SetValues (const double a11, const double a12, const double a13, const double a14, const double a21, const double a22, const double a23, const double a24, const double a31, const double a32, const double a33, const double a34);

  //! Returns true if the determinant of the vectorial part of
  //! this transformation is negative.
  bool IsNegative() const { return (scale < 0.0); }

  //! Returns the nature of the transformation. It can be: an
  //! identity transformation, a rotation, a translation, a mirror
  //! transformation (relative to a point, an axis or a plane), a
  //! scaling transformation, or a compound transformation.
  gp_TrsfForm Form() const { return shape; }

  //! Returns the scale factor.
  double ScaleFactor() const { return scale; }

  //! Returns the translation part of the transformation's matrix
  const gp_XYZ& TranslationPart() const { return loc; }

  //! Returns the boolean True if there is non-zero rotation.
  //! In the presence of rotation, the output parameters store the axis
  //! and the angle of rotation. The method always returns positive
  //! value "theAngle", i.e., 0. < theAngle <= PI.
  //! Note that this rotation is defined only by the vectorial part of
  //! the transformation; generally you would need to check also the
  //! translational part to obtain the axis (gp_Ax1) of rotation.
  mobiusGeom_EXPORT bool GetRotation (gp_XYZ& theAxis, double& theAngle) const;

  //! Returns quaternion representing rotational part of the transformation.
  mobiusGeom_EXPORT gp_Quaternion GetRotation() const;

  //! Returns the vectorial part of the transformation. It is
  //! a 3*3 matrix which includes the scale factor.
  mobiusGeom_EXPORT gp_Mat VectorialPart() const;

  //! Computes the homogeneous vectorial part of the transformation.
  //! It is a 3*3 matrix which doesn't include the scale factor.
  //! In other words, the vectorial part of this transformation is equal
  //! to its homogeneous vectorial part, multiplied by the scale factor.
  //! The coefficients of this matrix must be multiplied by the
  //! scale factor to obtain the coefficients of the transformation.
  const gp_Mat& HVectorialPart() const { return matrix; }

  //! Returns the coefficients of the transformation's matrix.
  //! It is a 3 rows * 4 columns matrix.
  //! This coefficient includes the scale factor.
  //! Raises OutOfRanged if theRow < 1 or theRow > 3 or theCol < 1 or theCol > 4
  double Value (const int theRow, const int theCol) const;

  mobiusGeom_EXPORT void Invert();

  //! Computes the reverse transformation
  //! Raises an exception if the matrix of the transformation
  //! is not inversible, it means that the scale factor is lower
  //! or equal to Resolution from package gp.
  //! Computes the transformation composed with T and  <me>.
  //! In a C++ implementation you can also write Tcomposed = <me> * T.
  //! Example :
  //! @code
  //! gp_Trsf T1, T2, Tcomp; ...............
  //! Tcomp = T2.Multiplied(T1);         // or   (Tcomp = T2 * T1)
  //! gp_Pnt P1(10.,3.,4.);
  //! gp_Pnt P2 = P1.Transformed(Tcomp); // using Tcomp
  //! gp_Pnt P3 = P1.Transformed(T1);    // using T1 then T2
  //! P3.Transform(T2);                  // P3 = P2 !!!
  //! @endcode
  mobiusCore_NODISCARD gp_Trsf Inverted() const
  {
    gp_Trsf aT = *this;
    aT.Invert();
    return aT;
  }
  
  mobiusCore_NODISCARD gp_Trsf Multiplied (const gp_Trsf& theT) const
  {
    gp_Trsf aTresult (*this);
    aTresult.Multiply (theT);
    return aTresult;
  }

  mobiusCore_NODISCARD gp_Trsf operator * (const gp_Trsf& theT) const { return Multiplied (theT); }

  //! Computes the transformation composed with <me> and theT.
  //! <me> = <me> * theT
  mobiusGeom_EXPORT void Multiply (const gp_Trsf& theT);

  void operator *= (const gp_Trsf& theT) { Multiply (theT); }

  //! Computes the transformation composed with <me> and T.
  //! <me> = theT * <me>
  mobiusGeom_EXPORT void PreMultiply (const gp_Trsf& theT);

  mobiusGeom_EXPORT void Power (const int theN);

  //! Computes the following composition of transformations
  //! <me> * <me> * .......* <me>, theN time.
  //! if theN = 0 <me> = Identity
  //! if theN < 0 <me> = <me>.Inverse() *...........* <me>.Inverse().
  //!
  //! Raises if theN < 0 and if the matrix of the transformation not
  //! inversible.
  mobiusCore_NODISCARD gp_Trsf Powered (const int theN) const
  {
    gp_Trsf aT = *this;
    aT.Power (theN);
    return aT;
  }

  void Transforms (double& theX, double& theY, double& theZ) const;

  //! Transformation of a triplet XYZ with a Trsf
  void Transforms (gp_XYZ& theCoord) const;


friend class gp_GTrsf;

protected:

  //! Makes orthogonalization of "matrix"
  mobiusGeom_EXPORT void Orthogonalize();

private:

  double scale;
  gp_TrsfForm shape;
  gp_Mat matrix;
  gp_XYZ loc;

};

#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_Vec.hxx>
#include <mobius/gp_Pnt.hxx>

//=======================================================================
//function : gp_Trsf
// purpose :
//=======================================================================
inline gp_Trsf::gp_Trsf ()
: scale (1.0),
  shape (gp_Identity),
  matrix (1, 0, 0, 0, 1, 0, 0, 0, 1),
  loc (0.0, 0.0, 0.0)
{}

//=======================================================================
//function : SetMirror
// purpose :
//=======================================================================
inline void gp_Trsf::SetMirror (const gp_Pnt& theP)
{
  shape = gp_PntMirror;
  scale = -1.0;
  loc = theP.XYZ();
  matrix.SetIdentity();
  loc.Multiply (2.0);
}

//=======================================================================
//function : SetTranslation
// purpose :
//=======================================================================
inline void gp_Trsf::SetTranslation (const gp_Vec& theV) 
{
  shape = gp_Translation;
  scale = 1.;
  matrix.SetIdentity();
  loc = theV.XYZ();
}

//=======================================================================
//function : SetTranslation
// purpose :
//=======================================================================
inline void gp_Trsf::SetTranslation (const gp_Pnt& theP1,
                                     const gp_Pnt& theP2) 
{
  shape = gp_Translation;
  scale = 1.0;
  matrix.SetIdentity();
  loc = (theP2.XYZ()).Subtracted (theP1.XYZ());
}

//=======================================================================
//function : Value
// purpose :
//=======================================================================
inline double gp_Trsf::Value (const int theRow, const int theCol) const
{
  if (theCol < 4)
  {
    return scale * matrix.Value (theRow, theCol);
  }
  else
  {
    return loc.Coord (theRow);
  }
}

//=======================================================================
//function : Transforms
// purpose :
//=======================================================================
inline void gp_Trsf::Transforms (double& theX,
                                 double& theY,
                                 double& theZ) const 
{
  gp_XYZ aTriplet (theX, theY, theZ);
  aTriplet.Multiply (matrix);
  if (scale != 1.0)
  {
    aTriplet.Multiply (scale);
  }
  aTriplet.Add (loc);
  theX = aTriplet.X();
  theY = aTriplet.Y();
  theZ = aTriplet.Z();
}

//=======================================================================
//function : Transforms
// purpose :
//=======================================================================
inline void gp_Trsf::Transforms (gp_XYZ& theCoord) const
{
  theCoord.Multiply (matrix);
  if (scale != 1.0)
  {
    theCoord.Multiply (scale);
  }
  theCoord.Add (loc);
}

#endif // _gp_Trsf_HeaderFile
