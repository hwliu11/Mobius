//-----------------------------------------------------------------------------
// Created on: 02 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/core_XYZ.h>

//-----------------------------------------------------------------------------
// Class-level API
//-----------------------------------------------------------------------------

//! Returns (0, 0, 0).
//! \return origin.
mobius::core_XYZ mobius::core_XYZ::O()
{
  return core_XYZ();
}

//! Returns (1, 0, 0).
//! \return direction for OX axis.
mobius::core_XYZ mobius::core_XYZ::OX()
{
  return core_XYZ(1.0, 0.0, 0.0);
}

//! Returns (0, 1, 0).
//! \return direction for OY axis.
mobius::core_XYZ mobius::core_XYZ::OY()
{
  return core_XYZ(0.0, 1.0, 0.0);
}

//! Returns (0, 0, 1).
//! \return direction for OZ axis.
mobius::core_XYZ mobius::core_XYZ::OZ()
{
  return core_XYZ(0.0, 0.0, 1.0);
}

//-----------------------------------------------------------------------------
// Object-level API
//-----------------------------------------------------------------------------

//! Returns modulus of the point's radius vector.
//! \return modulus.
inline double mobius::core_XYZ::Modulus() const
{
  return sqrt( this->SquaredModulus() );
}

//! Returns squared modulus of the point's radius vector.
//! \return squared modulus.
inline double mobius::core_XYZ::SquaredModulus() const
{
  return m_fX*m_fX + m_fY*m_fY + m_fZ*m_fZ;
}

//! Multiplies copy of point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return resulting point.
inline mobius::core_XYZ mobius::core_XYZ::Multiplied(const double coeff) const
{
  return this->operator*(coeff);
}

//! Normalizes point so that it gets unit length.
inline void mobius::core_XYZ::Normalize()
{
  const double modulus = this->Modulus();
  m_fX /= modulus;
  m_fY /= modulus;
  m_fZ /= modulus;
}

//! Creates a normalized copy of this point.
//! \return normalized copy.
inline mobius::core_XYZ mobius::core_XYZ::Normalized() const
{
  core_XYZ C(m_fX, m_fY, m_fZ);
  C.Normalize();
  return C;
}

//! Calculates dot product between this and another vector.
//! \param XYZ [in] another vector.
//! \return dot product.
inline double mobius::core_XYZ::Dot(const core_XYZ& XYZ) const
{
  return m_fX*XYZ.m_fX + m_fY*XYZ.m_fY + m_fZ*XYZ.m_fZ;
}

//! Calculates cross product between this and another vector.
//! \param XYZ [in] another vector.
//! \return cross product.
inline mobius::core_XYZ mobius::core_XYZ::Cross(const core_XYZ& XYZ) const
{
  const double x = m_fY*XYZ.m_fZ - m_fZ*XYZ.m_fY;
  const double y = m_fZ*XYZ.m_fX - m_fX*XYZ.m_fZ;
  const double z = m_fX*XYZ.m_fY - m_fY*XYZ.m_fX;

  return core_XYZ(x, y, z);
}

//! Calculates angle between two points.
//! \param XYZ [in] another vector.
//! \return angle in radians.
inline double mobius::core_XYZ::Angle(const core_XYZ& XYZ) const
{
  core_XYZ V1 = this->Normalized();
  core_XYZ V2 = XYZ.Normalized();

  double cos_alpha = V1.Dot(V2);

  // Compensate rounding errors
  if ( cos_alpha > 1 )
    cos_alpha = 1.0;

  // Compensate rounding errors
  if ( cos_alpha < -1 )
    cos_alpha = -1.0;

  const double alpha = acos(cos_alpha);
  return alpha;
}

//! Assignment operator.
//! \param XYZ [in] point to copy into this one.
//! \return this one.
inline mobius::core_XYZ& mobius::core_XYZ::operator=(const core_XYZ& XYZ)
{
  m_fX = XYZ.m_fX;
  m_fY = XYZ.m_fY;
  m_fZ = XYZ.m_fZ;
  return *this;
}

//! Multiplies copy of point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return resulting point.
inline mobius::core_XYZ mobius::core_XYZ::operator*(const double coeff) const
{
  core_XYZ XYZ_Copy(*this);
  XYZ_Copy *= coeff;
  return XYZ_Copy;
}

//! Multiplies this point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return this point multiplied by the passed scalar.
inline mobius::core_XYZ mobius::core_XYZ::operator*=(const double coeff)
{
  this->m_fX *= coeff;
  this->m_fY *= coeff;
  this->m_fZ *= coeff;
  return *this;
}

//! Divides copy of point by the passed scalar value.
//! \param coeff [in] scalar value to divide point by.
//! \return resulting point.
inline mobius::core_XYZ mobius::core_XYZ::operator/(const double coeff) const
{
  core_XYZ XYZ_Copy(*this);
  XYZ_Copy /= coeff;
  return XYZ_Copy;
}

//! Divides this point by the passed scalar value.
//! \param coeff [in] scalar value to divide point by.
//! \return this point multiplied by the passed scalar.
inline mobius::core_XYZ mobius::core_XYZ::operator/=(const double coeff)
{
  this->m_fX /= coeff;
  this->m_fY /= coeff;
  this->m_fZ /= coeff;
  return *this;
}

//! Adds the passed point to the copy of this one.
//! \param XYZ [in] point to add.
//! \return result of addition.
inline mobius::core_XYZ mobius::core_XYZ::operator+(const core_XYZ& XYZ) const
{
  core_XYZ XYZ_Copy(*this);
  XYZ_Copy += XYZ;
  return XYZ_Copy;
}

//! Adds the passed point to this one.
//! \param XYZ [in] point to add.
//! \return result of addition.
inline mobius::core_XYZ& mobius::core_XYZ::operator+=(const core_XYZ& XYZ)
{
  this->m_fX += XYZ.m_fX;
  this->m_fY += XYZ.m_fY;
  this->m_fZ += XYZ.m_fZ;
  return *this;
}

//! Inverts the copy of this point.
//! \return result of inversion.
inline mobius::core_XYZ mobius::core_XYZ::Invert() const
{
  core_XYZ XYZ_Copy(*this);
  XYZ_Copy.m_fX = -this->m_fX;
  XYZ_Copy.m_fY = -this->m_fY;
  XYZ_Copy.m_fZ = -this->m_fZ;
  return XYZ_Copy;
}

//! Subtracts the passed point from the copy of this one.
//! \param XYZ [in] point to subtract.
//! \return result of subtraction.
inline mobius::core_XYZ mobius::core_XYZ::operator-(const core_XYZ& XYZ) const
{
  return this->operator+( XYZ.Invert() );
}

//! Adds the passed point to this one.
//! \param XYZ [in] point to add.
//! \return result of subtraction.
inline mobius::core_XYZ& mobius::core_XYZ::operator-=(const core_XYZ& XYZ)
{
  return this->operator+=( XYZ.Invert() );
}
