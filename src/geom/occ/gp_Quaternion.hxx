// Created on: 2010-05-11
// Created by: Kirill GAVRILOV
// Copyright (c) 2010-2014 OPEN CASCADE SAS
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

#ifndef _gp_Quaternion_HeaderFile
#define _gp_Quaternion_HeaderFile

#include <mobius/gp_Mat.hxx>
#include <mobius/gp_Vec.hxx>

namespace mobius {
namespace occ {

//! Enumerates all 24 possible variants of generalized
//! Euler angles, defining general 3d rotation by three
//! rotations around main axes of coordinate system,
//! in different possible orders.
//!
//! The name of the enumeration
//! corresponds to order of rotations, prefixed by type
//! of coordinate system used:
//! - Intrinsic: rotations are made around axes of rotating
//!   coordinate system associated with the object
//! - Extrinsic: rotations are made around axes of fixed
//!   (static) coordinate system
//!
//! Two specific values are provided for most frequently used
//! conventions: classic Euler angles (intrinsic ZXZ) and
//! yaw-pitch-roll (intrinsic ZYX).

enum gp_EulerSequence
{
  //! Classic Euler angles, alias to Intrinsic_ZXZ
  gp_EulerAngles,

  //! Yaw Pitch Roll (or nautical) angles, alias to Intrinsic_ZYX
  gp_YawPitchRoll,

  // Tait-Bryan angles (using three different axes)
  gp_Extrinsic_XYZ,
  gp_Extrinsic_XZY,
  gp_Extrinsic_YZX,
  gp_Extrinsic_YXZ,
  gp_Extrinsic_ZXY,
  gp_Extrinsic_ZYX,

  gp_Intrinsic_XYZ,
  gp_Intrinsic_XZY,
  gp_Intrinsic_YZX,
  gp_Intrinsic_YXZ,
  gp_Intrinsic_ZXY,
  gp_Intrinsic_ZYX,

  // Proper Euler angles (using two different axes, first and third the same)
  gp_Extrinsic_XYX,
  gp_Extrinsic_XZX,
  gp_Extrinsic_YZY,
  gp_Extrinsic_YXY,
  gp_Extrinsic_ZYZ,
  gp_Extrinsic_ZXZ,

  gp_Intrinsic_XYX,
  gp_Intrinsic_XZX,
  gp_Intrinsic_YZY,
  gp_Intrinsic_YXY,
  gp_Intrinsic_ZXZ,
  gp_Intrinsic_ZYZ
};

//! Represents operation of rotation in 3d space as quaternion
//! and implements operations with rotations basing on
//! quaternion mathematics.
//!
//! In addition, provides methods for conversion to and from other
//! representations of rotation (3*3 matrix, vector and
//! angle, Euler angles)
class gp_Quaternion 
{
public:

  

  //! Creates an identity quaternion
  gp_Quaternion()
  : x (0.0),
    y (0.0),
    z (0.0),
    w (1.0)
  {}

  //! Creates quaternion directly from component values
  gp_Quaternion (const double theX, const double theY, const double theZ, const double theW)
  : x (theX),
    y (theY),
    z (theZ),
    w (theW)
  {}

  //! Creates quaternion representing shortest-arc rotation
  //! operator producing vector theVecTo from vector theVecFrom.
  gp_Quaternion (const gp_Vec& theVecFrom, const gp_Vec& theVecTo)
  {
    SetRotation (theVecFrom, theVecTo);
  }

  //! Creates quaternion representing shortest-arc rotation
  //! operator producing vector theVecTo from vector theVecFrom.
  //! Additional vector theHelpCrossVec defines preferred direction for
  //! rotation and is used when theVecTo and theVecFrom are directed
  //! oppositely.
  gp_Quaternion(const gp_Vec& theVecFrom, const gp_Vec& theVecTo, const gp_Vec& theHelpCrossVec)
  {
    SetRotation (theVecFrom, theVecTo, theHelpCrossVec);
  }

  //! Creates quaternion representing rotation on angle
  //! theAngle around vector theAxis
  gp_Quaternion(const gp_Vec& theAxis, const double theAngle)
  {
    SetVectorAndAngle (theAxis, theAngle);
  }

  //! Creates quaternion from rotation matrix 3*3
  //! (which should be orthonormal skew-symmetric matrix)
  gp_Quaternion(const gp_Mat& theMat)
  {
    SetMatrix (theMat);
  }

  //! Simple equal test without precision
  mobiusGeom_EXPORT bool IsEqual (const gp_Quaternion& theOther) const;

  //! Sets quaternion to shortest-arc rotation producing
  //! vector theVecTo from vector theVecFrom.
  //! If vectors theVecFrom and theVecTo are opposite then rotation
  //! axis is computed as theVecFrom ^ (1,0,0) or theVecFrom ^ (0,0,1).
  mobiusGeom_EXPORT void SetRotation (const gp_Vec& theVecFrom, const gp_Vec& theVecTo);

  //! Sets quaternion to shortest-arc rotation producing
  //! vector theVecTo from vector theVecFrom.
  //! If vectors theVecFrom and theVecTo are opposite then rotation
  //! axis is computed as theVecFrom ^ theHelpCrossVec.
  mobiusGeom_EXPORT void SetRotation (const gp_Vec& theVecFrom, const gp_Vec& theVecTo, const gp_Vec& theHelpCrossVec);

  //! Create a unit quaternion from Axis+Angle representation
  mobiusGeom_EXPORT void SetVectorAndAngle (const gp_Vec& theAxis, const double theAngle);

  //! Convert a quaternion to Axis+Angle representation,
  //! preserve the axis direction and angle from -PI to +PI
  mobiusGeom_EXPORT void GetVectorAndAngle (gp_Vec& theAxis, double& theAngle) const;

  //! Create a unit quaternion by rotation matrix
  //! matrix must contain only rotation (not scale or shear)
  //!
  //! For numerical stability we find first the greatest component of quaternion
  //! and than search others from this one
  mobiusGeom_EXPORT void SetMatrix (const gp_Mat& theMat);

  //! Returns rotation operation as 3*3 matrix
  mobiusGeom_EXPORT gp_Mat GetMatrix() const;

  //! Create a unit quaternion representing rotation defined
  //! by generalized Euler angles
  mobiusGeom_EXPORT void SetEulerAngles (const gp_EulerSequence theOrder, const double theAlpha, const double theBeta, const double theGamma);

  //! Returns Euler angles describing current rotation
  mobiusGeom_EXPORT void GetEulerAngles (const gp_EulerSequence theOrder, double& theAlpha, double& theBeta, double& theGamma) const;

  void Set (const double theX, const double theY, const double theZ, const double theW);

  void Set (const gp_Quaternion& theQuaternion);

  double X() const { return x; }

  double Y() const { return y; }

  double Z() const { return z; }

  double W() const { return w; }

  //! Make identity quaternion (zero-rotation)
  void SetIdent()
  {
    x = y = z = 0.0;
    w = 1.0;
  }

  //! Reverse direction of rotation (conjugate quaternion)
  void Reverse()
  {
    x = -x;
    y = -y;
    z = -z;
  }

  //! Return rotation with reversed direction (conjugated quaternion)
  mobiusCore_NODISCARD gp_Quaternion Reversed() const { return gp_Quaternion (-x, -y, -z, w); }

  //! Inverts quaternion (both rotation direction and norm)
  void Invert()
  {
    double anIn = 1.0 / SquareNorm();
    Set (-x * anIn, -y * anIn, -z * anIn, w * anIn);
  }

  //! Return inversed quaternion q^-1
  mobiusCore_NODISCARD gp_Quaternion Inverted() const
  {
    double anIn = 1.0 / SquareNorm();
    return gp_Quaternion (-x * anIn, -y * anIn, -z * anIn, w * anIn);
  }

  //! Returns square norm of quaternion
  double SquareNorm() const
  {
    return x * x + y * y + z * z + w * w;
  }

  //! Returns norm of quaternion
  double Norm() const { return Sqrt (SquareNorm()); }

  //! Scale all components by quaternion by theScale; note that
  //! rotation is not changed by this operation (except 0-scaling)
  void Scale (const double theScale);

  void operator *= (const double theScale) { Scale (theScale); }

  //! Returns scaled quaternion
  mobiusCore_NODISCARD gp_Quaternion Scaled (const double theScale) const
  {
    return gp_Quaternion (x * theScale, y * theScale, z * theScale, w * theScale);
  }

  mobiusCore_NODISCARD gp_Quaternion operator * (const double theScale) const { return Scaled (theScale); }

  //! Stabilize quaternion length within 1 - 1/4.
  //! This operation is a lot faster than normalization
  //! and preserve length goes to 0 or infinity
  mobiusGeom_EXPORT void StabilizeLength();

  //! Scale quaternion that its norm goes to 1.
  //! The appearing of 0 magnitude or near is a error,
  //! so we can be sure that can divide by magnitude
  mobiusGeom_EXPORT void Normalize();

  //! Returns quaternion scaled so that its norm goes to 1.
  mobiusCore_NODISCARD gp_Quaternion Normalized() const
  {
    gp_Quaternion aNormilizedQ (*this);
    aNormilizedQ.Normalize();
    return aNormilizedQ;
  }

  //! Returns quaternion with all components negated.
  //! Note that this operation does not affect neither
  //! rotation operator defined by quaternion nor its norm.
  mobiusCore_NODISCARD gp_Quaternion Negated() const { return gp_Quaternion (-x, -y, -z, -w); }

  mobiusCore_NODISCARD gp_Quaternion operator -() const { return Negated(); }

  //! Makes sum of quaternion components; result is "rotations mix"
  mobiusCore_NODISCARD gp_Quaternion Added (const gp_Quaternion& theOther) const
  {
    return gp_Quaternion (x + theOther.x, y + theOther.y, z + theOther.z, w + theOther.w);
  }

  mobiusCore_NODISCARD gp_Quaternion operator + (const gp_Quaternion& theOther) const { return Added (theOther); }

  //! Makes difference of quaternion components; result is "rotations mix"
  mobiusCore_NODISCARD gp_Quaternion Subtracted (const gp_Quaternion& theOther) const
  {
    return gp_Quaternion (x - theOther.x, y - theOther.y, z - theOther.z, w - theOther.w);
  }

  mobiusCore_NODISCARD gp_Quaternion operator - (const gp_Quaternion& theOther) const { return Subtracted (theOther); }

  //! Multiply function - work the same as Matrices multiplying.
  //! @code
  //! qq' = (cross(v,v') + wv' + w'v, ww' - dot(v,v'))
  //! @endcode
  //! Result is rotation combination: q' than q (here q=this, q'=theQ).
  //! Notices that:
  //! @code
  //! qq' != q'q;
  //! qq^-1 = q;
  //! @endcode
  mobiusCore_NODISCARD gp_Quaternion Multiplied (const gp_Quaternion& theOther) const;

  mobiusCore_NODISCARD gp_Quaternion operator * (const gp_Quaternion& theOther) const { return Multiplied (theOther); }

  //! Adds components of other quaternion; result is "rotations mix"
  void Add (const gp_Quaternion& theOther);

  void operator += (const gp_Quaternion& theOther) { Add (theOther); }

  //! Subtracts components of other quaternion; result is "rotations mix"
  void Subtract (const gp_Quaternion& theOther);

  void operator -= (const gp_Quaternion& theOther) { Subtract (theOther); }

  //! Adds rotation by multiplication
  void Multiply (const gp_Quaternion& theOther)
  {
    (*this) = Multiplied (theOther);  // have no optimization here
  }

  void operator *= (const gp_Quaternion& theOther) { Multiply (theOther); }

  //! Computes inner product / scalar product / Dot
  double Dot (const gp_Quaternion& theOther) const
  {
    return x * theOther.x + y * theOther.y + z * theOther.z + w * theOther.w;
  }

  //! Return rotation angle from -PI to PI
  mobiusGeom_EXPORT double GetRotationAngle() const;

  //! Rotates vector by quaternion as rotation operator
  mobiusGeom_EXPORT gp_Vec Multiply (const gp_Vec& theVec) const;

  gp_Vec operator * (const gp_Vec& theVec) const { return Multiply (theVec); }

private:

  double x;
  double y;
  double z;
  double w;

};

//=======================================================================
//function : Set
//purpose  :
//=======================================================================
inline void gp_Quaternion::Set (double theX, double theY,
                                double theZ, double theW)
{
  this->x = theX;
  this->y = theY;
  this->z = theZ;
  this->w = theW;
}

//=======================================================================
//function : Set
//purpose  :
//=======================================================================
inline void gp_Quaternion::Set (const gp_Quaternion& theQuaternion)
{
  x = theQuaternion.x; 
  y = theQuaternion.y; 
  z = theQuaternion.z; 
  w = theQuaternion.w;
}

//=======================================================================
//function : Scale
//purpose  :
//=======================================================================
inline void gp_Quaternion::Scale (const double theScale)
{
  x *= theScale; 
  y *= theScale; 
  z *= theScale; 
  w *= theScale;
}

//=======================================================================
//function : Multiplied
//purpose  :
//=======================================================================
inline gp_Quaternion gp_Quaternion::Multiplied (const gp_Quaternion& theQ) const
{
  return gp_Quaternion (w * theQ.x + x * theQ.w + y * theQ.z - z * theQ.y,
                        w * theQ.y + y * theQ.w + z * theQ.x - x * theQ.z,
                        w * theQ.z + z * theQ.w + x * theQ.y - y * theQ.x,
                        w * theQ.w - x * theQ.x - y * theQ.y - z * theQ.z);
  // 16 multiplications    12 addidtions    0 variables
}

//=======================================================================
//function : Add
//purpose  :
//=======================================================================
inline void gp_Quaternion::Add (const gp_Quaternion& theQ)
{
  x += theQ.x; 
  y += theQ.y; 
  z += theQ.z; 
  w += theQ.w;
}

//=======================================================================
//function : Subtract
//purpose  :
//=======================================================================
inline void gp_Quaternion::Subtract (const gp_Quaternion& theQ)
{
  x -= theQ.x;
  y -= theQ.y;
  z -= theQ.z;
  w -= theQ.w;
}

}
}

#endif // _gp_Quaternion_HeaderFile
