//-----------------------------------------------------------------------------
// Created on: 02 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/core_UV.h>

//! Default constructor. Initializes point coordinates with origin values:
//! (0, 0).
mobius::core_UV::core_UV()
{
  m_fU = m_fV = 0;
}

//! Complete constructor.
//! \param u [in] first coordinate.
//! \param v [in] second coordinate.
mobius::core_UV::core_UV(const double u, const double v)
{
  m_fU = u;
  m_fV = v;
}

//! Copy constructor.
//! \param UV [in] point to copy.
mobius::core_UV::core_UV(const core_UV& UV)
{
  this->operator=(UV);
}

//! Destructor.
mobius::core_UV::~core_UV()
{}

//-----------------------------------------------------------------------------
// API
//-----------------------------------------------------------------------------

//! Returns modulus of the point's radius vector.
//! \return modulus.
inline double mobius::core_UV::Modulus() const
{
  return sqrt( this->SquaredModulus() );
}

//! Returns squared modulus of the point's radius vector.
//! \return modulus.
inline double mobius::core_UV::SquaredModulus() const
{
  return m_fU*m_fU + m_fV*m_fV;
}

//! Calculates dot product between this and another vector.
//! \param UV [in] another vector.
//! \return dot product.
inline double mobius::core_UV::Dot(const core_UV& UV) const
{
  return m_fU*UV.m_fU + m_fV*UV.m_fV;
}

//! Assignment operator.
//! \param UV [in] point to copy into this one.
//! \return this one.
inline mobius::core_UV& mobius::core_UV::operator=(const core_UV& UV)
{
  m_fU = UV.m_fU;
  m_fV = UV.m_fV;
  return *this;
}

//! Multiplies copy of point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return resulting point.
inline mobius::core_UV mobius::core_UV::operator*(const double coeff) const
{
  core_UV UV_Copy(*this);
  UV_Copy *= coeff;
  return UV_Copy;
}

//! Multiplies this point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return this point multiplied by the passed scalar.
inline mobius::core_UV mobius::core_UV::operator*=(const double coeff)
{
  this->m_fU *= coeff;
  this->m_fV *= coeff;
  return *this;
}

//! Divides copy of point by the passed scalar value.
//! \param coeff [in] scalar value to divide point by.
//! \return resulting point.
inline mobius::core_UV mobius::core_UV::operator/(const double coeff) const
{
  core_UV UV_Copy(*this);
  UV_Copy /= coeff;
  return UV_Copy;
}

//! Divides this point by the passed scalar value.
//! \param coeff [in] scalar value to divide point by.
//! \return this point multiplied by the passed scalar.
inline mobius::core_UV mobius::core_UV::operator/=(const double coeff)
{
  this->m_fU /= coeff;
  this->m_fV /= coeff;
  return *this;
}

//! Adds the passed point to the copy of this one.
//! \param UV [in] point to add.
//! \return result of addition.
inline mobius::core_UV mobius::core_UV::operator+(const core_UV& UV) const
{
  core_UV UV_Copy(*this);
  UV_Copy += UV;
  return UV_Copy;
}

//! Adds the passed point to this one.
//! \param UV [in] point to add.
//! \return result of addition.
inline mobius::core_UV& mobius::core_UV::operator+=(const core_UV& UV)
{
  this->m_fU += UV.m_fU;
  this->m_fV += UV.m_fV;
  return *this;
}

//! Inverts the copy of this point.
//! \return result of inversion.
inline mobius::core_UV mobius::core_UV::Invert() const
{
  core_UV UV_Copy(*this);
  UV_Copy.m_fU = -this->m_fU;
  UV_Copy.m_fV = -this->m_fV;
  return UV_Copy;
}

//! Subtracts the passed point from the copy of this one.
//! \param UV [in] point to subtract.
//! \return result of subtraction.
inline mobius::core_UV mobius::core_UV::operator-(const core_UV& UV) const
{
  return this->operator+( UV.Invert() );
}

//! Adds the passed point to this one.
//! \param UV [in] point to add.
//! \return result of subtraction.
inline mobius::core_UV& mobius::core_UV::operator-=(const core_UV& UV)
{
  return this->operator+=( UV.Invert() );
}
