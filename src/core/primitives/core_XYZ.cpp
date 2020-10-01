//-----------------------------------------------------------------------------
// Created on: 02 November 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of Sergey Slyadnev nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

// Own include
#include <mobius/core_XYZ.h>

// Standard includes
#include <math.h>
#include <set>

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

//! Checks if the passed collection of vectors span a single plane.
//! \param[in] dirs the directions to check.
//! \param[in] prec the precision value to use for computing angles
//!                 and dot products.
//! \return true/false.
bool mobius::core_XYZ::AreSamePlane(const std::vector<core_XYZ>& dirs,
                                    const double                 prec)
{
  if ( dirs.size() < 3 )
    return true;

  // Select planar "basis" to compute a reference norm.
  core_XYZ N;
  std::set<int> basis = {0};
  //
  for ( int k = 1; k < dirs.size(); ++k )
  {
    const double ang = dirs[k].Angle(dirs[0]);
    //
    if ( (fabs(ang) > prec) && (fabs(ang - M_PI) > prec) )
    {
      N = dirs[k]^dirs[0];
      basis.insert(k);
      break;
    }
  }

  if ( N.Modulus() < prec )
    return false; // No idea, hence false.

  // Check all remaining directions w.r.t. the basis' norm.
  for ( int k = 1; k < dirs.size(); ++k )
  {
    if ( basis.find(k) != basis.end() )
      continue; // Skip already accounted vectors.

    if ( fabs( dirs[k].Dot(N) ) > prec )
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// Object-level API
//-----------------------------------------------------------------------------

//! Returns modulus of the point's radius vector.
//! \return modulus.
double mobius::core_XYZ::Modulus() const
{
  return sqrt( this->SquaredModulus() );
}

//! Returns squared modulus of the point's radius vector.
//! \return squared modulus.
double mobius::core_XYZ::SquaredModulus() const
{
  return m_fX*m_fX + m_fY*m_fY + m_fZ*m_fZ;
}

//! Multiplies copy of point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return resulting point.
mobius::core_XYZ mobius::core_XYZ::Multiplied(const double coeff) const
{
  return this->operator*(coeff);
}

//! Normalizes point so that it gets unit length.
void mobius::core_XYZ::Normalize()
{
  const double modulus = this->Modulus();
  m_fX /= modulus;
  m_fY /= modulus;
  m_fZ /= modulus;
}

//! Creates a normalized copy of this point.
//! \return normalized copy.
mobius::core_XYZ mobius::core_XYZ::Normalized() const
{
  core_XYZ C(m_fX, m_fY, m_fZ);
  C.Normalize();
  return C;
}

//! Calculates dot product between this and another vector.
//! \param XYZ [in] another vector.
//! \return dot product.
double mobius::core_XYZ::Dot(const core_XYZ& XYZ) const
{
  return m_fX*XYZ.m_fX + m_fY*XYZ.m_fY + m_fZ*XYZ.m_fZ;
}

//! Calculates cross product between this and another vector.
//! \param XYZ [in] another vector.
//! \return cross product.
mobius::core_XYZ mobius::core_XYZ::Cross(const core_XYZ& XYZ) const
{
  const double x = m_fY*XYZ.m_fZ - m_fZ*XYZ.m_fY;
  const double y = m_fZ*XYZ.m_fX - m_fX*XYZ.m_fZ;
  const double z = m_fX*XYZ.m_fY - m_fY*XYZ.m_fX;

  return core_XYZ(x, y, z);
}

//! Calculates angle between two points.
//! \param XYZ [in] another vector.
//! \return angle in radians.
double mobius::core_XYZ::Angle(const core_XYZ& XYZ) const
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

//! Calculates the angle, in radians, between this vector and vector `other`.
//! The result is a value between -M_PI and M_PI. The vector `ref` defines
//! the positive sense of rotation: the angular value is positive, if the
//! cross product `this ^ other` has the same orientation as `ref` relative
//! to the plane defined by the vectors `this` and `other`. Otherwise, the
//! angular value is negative.
double mobius::core_XYZ::AngleWithRef(const core_XYZ& other,
                                      const core_XYZ& ref) const
{
  core_XYZ XYZ = this->Cross(other).Normalized();

  double alpha;
  double Cosinus = this->Normalized().Dot( other.Normalized() );
  double Sinus   = XYZ.Modulus();
  //
  if ( Cosinus > -0.70710678118655 && Cosinus < 0.70710678118655 )
    alpha = acos(Cosinus);
  else {
    if ( Cosinus < 0.0 ) alpha = M_PI - asin(Sinus);
    else                 alpha =        asin(Sinus);
  }
  if ( XYZ.Dot(ref) >= 0.0 )  return  alpha;
  else                        return -alpha;
}

//! Exact equality operator. Use with care.
//! \param XYZ [in] point to compare with.
//! \return true in case of exact (bit-to-bit) equality.
bool mobius::core_XYZ::operator==(const core_XYZ& XYZ) const
{
  return (m_fX == XYZ.m_fX) && (m_fY == XYZ.m_fY) && (m_fZ == XYZ.m_fZ);
}

//! Assignment operator.
//! \param XYZ [in] point to copy into this one.
//! \return this one.
mobius::core_XYZ& mobius::core_XYZ::operator=(const core_XYZ& XYZ)
{
  m_fX = XYZ.m_fX;
  m_fY = XYZ.m_fY;
  m_fZ = XYZ.m_fZ;
  return *this;
}

//! Multiplies copy of point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return resulting point.
mobius::core_XYZ mobius::core_XYZ::operator*(const double coeff) const
{
  core_XYZ XYZ_Copy(*this);
  XYZ_Copy *= coeff;
  return XYZ_Copy;
}

//! Multiplies this point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return this point multiplied by the passed scalar.
mobius::core_XYZ mobius::core_XYZ::operator*=(const double coeff)
{
  this->m_fX *= coeff;
  this->m_fY *= coeff;
  this->m_fZ *= coeff;
  return *this;
}

//! Computes the cross product between this triple of coordinates and the
//! passed argument.
//! \param[in] XYZ argument triple.
//! \return cross product.
mobius::core_XYZ mobius::core_XYZ::operator^(const core_XYZ& XYZ) const
{
  return this->Cross(XYZ);
}

//! Divides copy of point by the passed scalar value.
//! \param coeff [in] scalar value to divide point by.
//! \return resulting point.
mobius::core_XYZ mobius::core_XYZ::operator/(const double coeff) const
{
  core_XYZ XYZ_Copy(*this);
  XYZ_Copy /= coeff;
  return XYZ_Copy;
}

//! Divides this point by the passed scalar value.
//! \param coeff [in] scalar value to divide point by.
//! \return this point multiplied by the passed scalar.
mobius::core_XYZ mobius::core_XYZ::operator/=(const double coeff)
{
  this->m_fX /= coeff;
  this->m_fY /= coeff;
  this->m_fZ /= coeff;
  return *this;
}

//! Adds the passed point to the copy of this one.
//! \param XYZ [in] point to add.
//! \return result of addition.
mobius::core_XYZ mobius::core_XYZ::operator+(const core_XYZ& XYZ) const
{
  core_XYZ XYZ_Copy(*this);
  XYZ_Copy += XYZ;
  return XYZ_Copy;
}

//! Adds the passed point to this one.
//! \param XYZ [in] point to add.
//! \return result of addition.
mobius::core_XYZ& mobius::core_XYZ::operator+=(const core_XYZ& XYZ)
{
  this->m_fX += XYZ.m_fX;
  this->m_fY += XYZ.m_fY;
  this->m_fZ += XYZ.m_fZ;
  return *this;
}

//! Inverts the copy of this point.
//! \return result of inversion.
mobius::core_XYZ mobius::core_XYZ::Invert() const
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
mobius::core_XYZ mobius::core_XYZ::operator-(const core_XYZ& XYZ) const
{
  return core_XYZ(this->m_fX - XYZ.m_fX, this->m_fY - XYZ.m_fY, this->m_fZ - XYZ.m_fZ);
}

//! Adds the passed point to this one.
//! \param XYZ [in] point to add.
//! \return result of subtraction.
mobius::core_XYZ& mobius::core_XYZ::operator-=(const core_XYZ& XYZ)
{
  return this->operator+=( XYZ.Invert() );
}
