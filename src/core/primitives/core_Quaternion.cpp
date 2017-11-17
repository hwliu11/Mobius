//-----------------------------------------------------------------------------
// Created on: 15 January 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/core_Quaternion.h>

// core includes
#include <mobius/core_Precision.h>

//! Default constructor. Creates identity quaternion [1, 0, 0, 0].
mobius::core_Quaternion::core_Quaternion()
{
  q0 = 1.0;
  qx = qy = qz = 0.0;
}

//! Complete constructor.
//! \param p0 [in] 1-st component.
//! \param px [in] 2-nd component.
//! \param py [in] 3-rd component.
//! \param pz [in] 4-th component.
mobius::core_Quaternion::core_Quaternion(const double p0,
                                         const double px,
                                         const double py,
                                         const double pz)
{
  q0 = p0;
  qx = px;
  qy = py;
  qz = pz;
}

//! Creates quaternion representing rotation around this given axes by the
//! given angle (in radians).
//! \param axis  [in] axis of rotation.
//! \param angle [in] angle of rotation (in radians).
mobius::core_Quaternion::core_Quaternion(const core_XYZ& axis,
                                         const double    angle)
{
  this->SetRotation(axis, angle);
}

//! Creates pure imaginary quaternion from the passed vector.
//! \param Qn_im [in] vector to represent as pure imaginary quaternion.
mobius::core_Quaternion::core_Quaternion(const core_XYZ& Qn_im)
{
  q0 = 0.0;
  qx = Qn_im.X();
  qy = Qn_im.Y();
  qz = Qn_im.Z();
}

//! Creates new quaternion as a copy of the passed one.
//! \param Qn [in] quaternion to copy.
mobius::core_Quaternion::core_Quaternion(const core_Quaternion& Qn)
{
  this->operator=(Qn);
}

//! Destructor.
mobius::core_Quaternion::~core_Quaternion()
{}

//! Initializes quaternion with axis of rotation and angle.
//! \param axis  [in] axis of rotation.
//! \param angle [in] rotation angle.
void mobius::core_Quaternion::SetRotation(const core_XYZ& axis,
                                          const double    angle)
{
  const double a2 = (double) ( angle/2.0 );
  const double cs = (double) ( cos(a2) );
  const double sn = (double) ( sin(a2) );
  core_XYZ     A  = axis.Normalized();

  q0 = cs;
  qx = sn*A.X();
  qy = sn*A.Y();
  qz = sn*A.Z();
}

//! Converts this quaternion to 3x3 matrix of rotation:
//!
//! <pre>
//! |tx'|   |R[0][0] R[0][1] R[0][2]| |tx|
//! |ty'| = |R[1][0] R[1][1] R[1][2]| |ty|
//! |tz'|   |R[2][0] R[2][1] R[2][2]| |tz|
//! </pre>
//!
//! \param mx [out] output matrix.
void mobius::core_Quaternion::Matrix3x3(double (&mx)[3][3]) const
{
  mx[0][0] =  q0*q0 + qx*qx - qy*qy - qz*qz;
  mx[0][1] = -2*q0*qz + 2*qx*qy;
  mx[0][2] =  2*q0*qy + 2*qx*qz;

  mx[1][0] =  2*q0*qz + 2*qx*qy;
  mx[1][1] =  q0*q0 - qx*qx + qy*qy - qz*qz;
  mx[1][2] = -2*q0*qx + 2*qy*qz;

  mx[2][0] = -2*q0*qy + 2*qx*qz;
  mx[2][1] =  2*q0*qx + 2*qy*qz;
  mx[2][2] =  q0*q0 - qx*qx - qy*qy + qz*qz;
}

//! Returns q0 component of the quaternion.
//! \return requested component.
double mobius::core_Quaternion::Q0() const
{
  return q0;
}

//! Returns qx component of the quaternion.
//! \return requested component.
double mobius::core_Quaternion::Qx() const
{
  return qx;
}

//! Returns qy component of the quaternion.
//! \return requested component.
double mobius::core_Quaternion::Qy() const
{
  return qy;
}

//! Returns qz component of the quaternion.
//! \return requested component.
double mobius::core_Quaternion::Qz() const
{
  return qz;
}

//! Attempts to represent this quaternion as vector. This is possible for
//! pure imaginary quaternions only.
//! \param XYZ [out] result.
//! \return true in case of success, false -- otherwise.
bool mobius::core_Quaternion::AsVector(core_XYZ& XYZ) const
{
  if ( fabs(q0) > core_Precision::Resolution3D() )
    return false;

  XYZ.SetX(qx);
  XYZ.SetY(qy);
  XYZ.SetZ(qz);
  return true;
}

//! Adds the passed quaternion to this one.
//! \param Qn [in] quaternion to add.
//! \return addition result.
mobius::core_Quaternion
  mobius::core_Quaternion::operator+(const core_Quaternion& Qn) const
{
  core_Quaternion Res;
  Res.q0 = q0 + Qn.q0;
  Res.qx = qx + Qn.qx;
  Res.qy = qy + Qn.qy;
  Res.qz = qz + Qn.qz;
  return Res;
}

//! Subtracts the passed quaternion from this one.
//! \param Qn [in] quaternion to subtract.
//! \return subtraction result.
mobius::core_Quaternion
  mobius::core_Quaternion::operator-(const core_Quaternion& Qn) const
{
  core_Quaternion Res;
  Res.q0 = q0 - Qn.q0;
  Res.qx = qx - Qn.qx;
  Res.qy = qy - Qn.qy;
  Res.qz = qz - Qn.qz;
  return Res;
}

//! Algebraic product of {this} and {Qn}. The result is:
//!
//! <pre>
//! result = this*Qn;
//! </pre>
//!
//! \param Qn [in] quaternion to multiply.
//! \return result of algebraic product.
mobius::core_Quaternion
  mobius::core_Quaternion::operator*(const core_Quaternion& Qn) const
{
  core_Quaternion Q;

  // 16 multiplications, 12 additions
  Q.q0 = q0*Qn.q0 - qx*Qn.qx - qy*Qn.qy - qz*Qn.qz;
  Q.qx = q0*Qn.qx + qx*Qn.q0 + qy*Qn.qz - qz*Qn.qy;
  Q.qy = q0*Qn.qy - qx*Qn.qz + qy*Qn.q0 + qz*Qn.qx;
  Q.qz = q0*Qn.qz + qx*Qn.qy - qy*Qn.qx + qz*Qn.q0;

  return Q;
}

//! Multiplies this quaternion to the passed vector. The latter vector is
//! simply a pure imaginary quaternion, so this product is just a special
//! case of general algebraic product.
//!
//! \param Qn_im [in] pure imaginary quaternion to multiply.
//! \return result of algebraic product.
mobius::core_Quaternion
  mobius::core_Quaternion::operator*(const core_XYZ& Qn_im) const
{
  core_Quaternion P(Qn_im);
  return this->operator*(P);
}

//! Multiplies quaternion by scalar value.
//! \param k [in] scalar to multiply the quaternion by.
//! \return multiplication result.
mobius::core_Quaternion
  mobius::core_Quaternion::operator*(const double k) const
{
  core_Quaternion Q;
  Q.q0 = q0*k;
  Q.qx = qx*k;
  Q.qy = qy*k;
  Q.qz = qz*k;
  return Q;
}

//! Dot product of {this} and {Qn}. The result is:
//!
//! <pre>
//! result = <this,Qn>;
//! </pre>
//!
//! \param Qn [in] quaternion to multiply.
//! \return dot product.
double mobius::core_Quaternion::Dot(const core_Quaternion& Qn) const
{
  return q0*Qn.q0 + qx*Qn.qx + qy*Qn.qy + qz*Qn.qz;
}

//! Cross product of {this} and {Qn}. The result is:
//!
//! <pre>
//! result = [this,Qn];
//! </pre>
//!
//! \param Qn [in] quaternion to multiply.
//! \return cross product.
mobius::core_Quaternion
  mobius::core_Quaternion::Cross(const core_Quaternion& Qn) const
{
  core_Quaternion Res, P(*this);
  return (P*Qn - Qn*P)*0.5;
}

//! Assigns the passed quaternion to this one.
//! \param Qn [in] quaternion to assign.
//! \return this.
mobius::core_Quaternion&
  mobius::core_Quaternion::operator=(const core_Quaternion& Qn)
{
  q0 = Qn.q0;
  qx = Qn.qx;
  qy = Qn.qy;
  qz = Qn.qz;
  return *this;
}

//! Inverts this quaternion.
void mobius::core_Quaternion::Invert()
{
  core_Quaternion Q(*this), QQ = this->Conjugated();
  this->operator=( QQ*( (double) ( 1.0 / Q.Dot(Q) ) ) );
}

//! Returns inverted copy of this quaternion.
//! \return inverted copy of this quaternion.
mobius::core_Quaternion mobius::core_Quaternion::Inverted() const
{
  core_Quaternion Res(*this);
  Res.Invert();
  return Res;
}

//! Conjugates this quaternion. E.g. if the original quaternion is:
//!
//! <pre>
//! q = q0 + i*qx + j*qy + k*qz;
//! </pre>
//!
//! Then after conjugation we will have:
//!
//! <pre>
//! q* = q0 - i*qx - j*qy - k*qz;
//! </pre>
void mobius::core_Quaternion::Conjugate()
{
  qx = -qx;
  qy = -qy;
  qz = -qz;
}

//! Returns conjugated copy of this quaternion.
//! \return conjugated copy of this quaternion.
mobius::core_Quaternion mobius::core_Quaternion::Conjugated() const
{
  core_Quaternion Res(*this);
  Res.Conjugate();
  return Res;
}
