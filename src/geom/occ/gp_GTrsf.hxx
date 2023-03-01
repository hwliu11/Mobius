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

#ifndef _gp_GTrsf_HeaderFile
#define _gp_GTrsf_HeaderFile

#include <mobius/gp_Ax1.hxx>
#include <mobius/gp_Ax2.hxx>
#include <mobius/gp_Mat.hxx>
#include <mobius/gp_Trsf.hxx>
#include <mobius/gp_TrsfForm.hxx>
#include <mobius/gp_XYZ.hxx>

namespace mobius {
namespace occ {

// Avoid possible conflict with SetForm macro defined by windows.h
#ifdef SetForm
#undef SetForm
#endif

//! Defines a non-persistent transformation in 3D space.
//! This transformation is a general transformation.
//! It can be a gp_Trsf, an affinity, or you can define
//! your own transformation giving the matrix of transformation.
//!
//! With a gp_GTrsf you can transform only a triplet of coordinates gp_XYZ.
//! It is not possible to transform other geometric objects
//! because these transformations can change the nature of non-elementary geometric objects.
//! The transformation gp_GTrsf can be represented as follow:
//! @code
//!    V1   V2   V3    T       XYZ        XYZ
//! | a11  a12  a13   a14 |   | x |      | x'|
//! | a21  a22  a23   a24 |   | y |      | y'|
//! | a31  a32  a33   a34 |   | z |   =  | z'|
//! |  0    0    0     1  |   | 1 |      | 1 |
//! @endcode
//! where {V1, V2, V3} define the vectorial part of the
//! transformation and T defines the translation part of the transformation.
//! Warning
//! A gp_GTrsf transformation is only applicable to coordinates.
//! Be careful if you apply such a transformation to all points of a geometric object,
//! as this can change the nature of the object and thus render it incoherent!
//! Typically, a circle is transformed into an ellipse by an affinity transformation.
//! To avoid modifying the nature of an object, use a gp_Trsf transformation instead,
//! as objects of this class respect the nature of geometric objects.
class gp_GTrsf 
{
public:

  

  //! Returns the Identity transformation.
  gp_GTrsf()
  {
    shape = gp_Identity;
    matrix.SetScale (1.0);
    loc.SetCoord (0.0, 0.0, 0.0);
    scale = 1.0;
  }

  //! Converts the gp_Trsf transformation theT into a
  //! general transformation, i.e. Returns a GTrsf with
  //! the same matrix of coefficients as the Trsf theT.
  gp_GTrsf (const gp_Trsf& theT)
  {
    shape = theT.Form();
    matrix = theT.matrix;
    loc = theT.TranslationPart();
    scale = theT.ScaleFactor();
  }

  //! Creates a transformation based on the matrix theM and the
  //! vector theV where theM defines the vectorial part of
  //! the transformation, and V the translation part, or
  gp_GTrsf (const gp_Mat& theM, const gp_XYZ& theV)
  : matrix (theM),
    loc (theV)
  {
    shape = gp_Other;
    scale = 0.0;
  }

  //! Changes this transformation into an affinity of ratio theRatio
  //! with respect to the axis theA1.
  //! Note: an affinity is a point-by-point transformation that
  //! transforms any point P into a point P' such that if H is
  //! the orthogonal projection of P on the axis theA1 or the
  //! plane A2, the vectors HP and HP' satisfy:
  //! HP' = theRatio * HP.
  void SetAffinity (const gp_Ax1& theA1, const double theRatio);

  //! Changes this transformation into an affinity of ratio theRatio
  //! with respect to  the plane defined by the origin, the "X Direction" and
  //! the "Y Direction" of coordinate system theA2.
  //! Note: an affinity is a point-by-point transformation that
  //! transforms any point P into a point P' such that if H is
  //! the orthogonal projection of P on the axis A1 or the
  //! plane theA2, the vectors HP and HP' satisfy:
  //! HP' = theRatio * HP.
  void SetAffinity (const gp_Ax2& theA2, const double theRatio);

  //! Replaces  the coefficient (theRow, theCol) of the matrix representing
  //! this transformation by theValue.  Raises OutOfRange
  //! if  theRow < 1 or theRow > 3 or theCol < 1 or theCol > 4
  void SetValue (const int theRow, const int theCol, const double theValue);

  //! Replaces the vectorial part of this transformation by theMatrix.
  void SetVectorialPart (const gp_Mat& theMatrix)
  {
    matrix = theMatrix;
    shape = gp_Other;
    scale = 0.0;
  }

  //! Replaces the translation part of
  //! this transformation by the coordinates of the number triple theCoord.
  mobiusGeom_EXPORT void SetTranslationPart (const gp_XYZ& theCoord);

  //! Assigns the vectorial and translation parts of theT to this transformation.
  void SetTrsf (const gp_Trsf& theT)
  {
    shape = theT.shape;
    matrix = theT.matrix;
    loc = theT.loc;
    scale = theT.scale;
  }

  //! Returns true if the determinant of the vectorial part of
  //! this transformation is negative.
  bool IsNegative() const { return matrix.Determinant() < 0.0; }

  //! Returns true if this transformation is singular (and
  //! therefore, cannot be inverted).
  //! Note: The Gauss LU decomposition is used to invert the
  //! transformation matrix. Consequently, the transformation
  //! is considered as singular if the largest pivot found is less
  //! than or equal to gp::Resolution().
  //! Warning
  //! If this transformation is singular, it cannot be inverted.
  bool IsSingular() const { return matrix.IsSingular(); }

  //! Returns the nature of the transformation.  It can be an
  //! identity transformation, a rotation, a translation, a mirror
  //! transformation (relative to a point, an axis or a plane), a
  //! scaling transformation, a compound transformation or
  //! some other type of transformation.
  gp_TrsfForm Form() const { return shape; }

  //! verify and set the shape of the GTrsf Other or CompoundTrsf
  //! Ex :
  //! @code
  //! myGTrsf.SetValue(row1,col1,val1);
  //! myGTrsf.SetValue(row2,col2,val2);
  //! ...
  //! myGTrsf.SetForm();
  //! @endcode
  mobiusGeom_EXPORT void SetForm();

  //! Returns the translation part of the GTrsf.
  const gp_XYZ& TranslationPart() const { return loc; }

  //! Computes the vectorial part of the GTrsf. The returned Matrix
  //! is a  3*3 matrix.
  const gp_Mat& VectorialPart() const { return matrix; }

  //! Returns the coefficients of the global matrix of transformation.
  //! Raises OutOfRange if theRow < 1 or theRow > 3 or theCol < 1 or theCol > 4
  double Value (const int theRow, const int theCol) const;

  double operator() (const int theRow, const int theCol) const { return Value (theRow, theCol); }

  mobiusGeom_EXPORT void Invert();

  //! Computes the reverse transformation.
  //! Raises an exception if the matrix of the transformation
  //! is not inversible.
  mobiusCore_NODISCARD gp_GTrsf Inverted() const
  {
    gp_GTrsf aT = *this;
    aT.Invert();
    return aT;
  }

  //! Computes the transformation composed from theT and <me>.
  //! In a C++ implementation you can also write Tcomposed = <me> * theT.
  //! Example :
  //! @code
  //! gp_GTrsf T1, T2, Tcomp; ...............
  //! //composition :
  //! Tcomp = T2.Multiplied(T1);         // or   (Tcomp = T2 * T1)
  //! // transformation of a point
  //! gp_XYZ P(10.,3.,4.);
  //! gp_XYZ P1(P);
  //! Tcomp.Transforms(P1);               //using Tcomp
  //! gp_XYZ P2(P);
  //! T1.Transforms(P2);                  //using T1 then T2
  //! T2.Transforms(P2);                  // P1 = P2 !!!
  //! @endcode
  mobiusCore_NODISCARD gp_GTrsf Multiplied (const gp_GTrsf& theT) const
   {
    gp_GTrsf aTres = *this;
    aTres.Multiply (theT);
    return aTres;
  }

  mobiusCore_NODISCARD gp_GTrsf operator * (const gp_GTrsf& theT)  const { return Multiplied (theT); }

  //! Computes the transformation composed with <me> and theT.
  //! <me> = <me> * theT
  mobiusGeom_EXPORT void Multiply (const gp_GTrsf& theT);

  void operator *= (const gp_GTrsf& theT) { Multiply (theT); }

  //! Computes the product of the transformation theT and this
  //! transformation and assigns the result to this transformation.
  //! this = theT * this
  mobiusGeom_EXPORT void PreMultiply (const gp_GTrsf& theT);

  mobiusGeom_EXPORT void Power (const int theN);

  //! Computes:
  //! -   the product of this transformation multiplied by itself
  //! theN times, if theN is positive, or
  //! -   the product of the inverse of this transformation
  //! multiplied by itself |theN| times, if theN is negative.
  //! If theN equals zero, the result is equal to the Identity
  //! transformation.
  //! I.e.:  <me> * <me> * .......* <me>, theN time.
  //! if theN =0 <me> = Identity
  //! if theN < 0 <me> = <me>.Inverse() *...........* <me>.Inverse().
  //!
  //! Raises an exception if N < 0 and if the matrix of the
  //! transformation not inversible.
  mobiusCore_NODISCARD gp_GTrsf Powered (const int theN) const
  {
    gp_GTrsf aT = *this;
    aT.Power (theN);
    return aT;
  }

  void Transforms (gp_XYZ& theCoord) const;

  //! Transforms a triplet XYZ with a GTrsf.
  void Transforms (double& theX, double& theY, double& theZ) const;

  gp_Trsf Trsf() const;


  //! Dumps the content of me into the stream
  mobiusGeom_EXPORT void DumpJson (std::ostream& theOStream, int theDepth = -1) const;

private:

  gp_Mat matrix;
  gp_XYZ loc;
  gp_TrsfForm shape;
  double scale;

};


//=======================================================================
//function : SetAffinity
// purpose :
//=======================================================================
inline void gp_GTrsf::SetAffinity (const gp_Ax1& theA1, const double theRatio)
{
  shape = gp_Other;
  scale = 0.0;
  matrix.SetDot (theA1.Direction().XYZ());
  matrix.Multiply (1.0 - theRatio);
  matrix.SetDiagonal (matrix.Value (1,1) + theRatio,
                      matrix.Value (2,2) + theRatio,
                      matrix.Value (3,3) + theRatio);
  loc = theA1.Location().XYZ();
  loc.Reverse ();
  loc.Multiply (matrix);
  loc.Add (theA1.Location().XYZ());
}

//=======================================================================
//function : SetAffinity
// purpose :
//=======================================================================
inline void gp_GTrsf::SetAffinity (const gp_Ax2& theA2, const double theRatio)
{
  shape = gp_Other;
  scale = 0.0;
  matrix.SetDot (theA2.Direction().XYZ());
  matrix.Multiply (theRatio - 1.);
  loc = theA2.Location().XYZ();
  loc.Reverse ();
  loc.Multiply (matrix);
  matrix.SetDiagonal (matrix.Value (1,1) + 1.,
                      matrix.Value (2,2) + 1.,
                      matrix.Value (3,3) + 1.);
}

//=======================================================================
//function : SetValue
// purpose :
//=======================================================================
inline void gp_GTrsf::SetValue (const int theRow,
                                const int theCol,
                                const double theValue)
{
  if (theCol == 4)
  {
    loc.SetCoord (theRow, theValue);
    if (shape == gp_Identity)
    {
      shape = gp_Translation;
    }
    return;
  }
  else
  {
    if (!(shape == gp_Other) && !(scale == 1.0))
    {
      matrix.Multiply (scale);
    }
    matrix.SetValue (theRow, theCol, theValue);
    shape = gp_Other;
    scale = 0.0;
    return;
  }
}

//=======================================================================
//function : Value
// purpose :
//=======================================================================
inline double gp_GTrsf::Value (const int theRow,
                                      const int theCol) const
{
  if (theCol == 4)
  {
    return loc.Coord (theRow);
  }
  if (shape == gp_Other)
  {
    return matrix.Value (theRow, theCol);
  }
  return scale * matrix.Value (theRow, theCol);
}

//=======================================================================
//function : Transforms
// purpose :
//=======================================================================
inline void gp_GTrsf::Transforms (gp_XYZ& theCoord) const
{
  theCoord.Multiply (matrix);
  if (!(shape == gp_Other) && !(scale == 1.0))
  {
    theCoord.Multiply (scale);
  }
  theCoord.Add (loc);
}

//=======================================================================
//function : Transforms
// purpose :
//=======================================================================
inline void gp_GTrsf::Transforms (double& theX, double& theY, double& theZ) const
{
  gp_XYZ aTriplet (theX, theY, theZ);
  aTriplet.Multiply (matrix);
  if (!(shape == gp_Other) && !(scale == 1.0))
  {
    aTriplet.Multiply (scale);
  }
  aTriplet.Add (loc);
  aTriplet.Coord (theX, theY, theZ);
}

//=======================================================================
//function : Trsf
// purpose :
//=======================================================================
inline gp_Trsf gp_GTrsf::Trsf() const
{
  if (Form() == gp_Other)
  {
    throw std::runtime_error("gp_GTrsf::Trsf() - non-orthogonal GTrsf");
  }
  gp_Trsf aT;
  aT.shape = shape;
  aT.scale = scale;
  aT.matrix = matrix;
  aT.loc = loc;
  return aT;
}

}
}

#endif // _gp_GTrsf_HeaderFile
