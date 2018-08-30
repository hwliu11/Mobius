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

#ifndef core_XYZ_HeaderFile
#define core_XYZ_HeaderFile

// core includes
#include <mobius/core.h>

// STL includes
#include <float.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Geometric primitive for 3D point.
class core_XYZ
{
// Static API:
public:

  //! \return number of coordinates.
  static int num_coordinates()
  {
    return 3;
  }

  mobiusCore_EXPORT static core_XYZ
    O();

  mobiusCore_EXPORT static core_XYZ
    OX();

  mobiusCore_EXPORT static core_XYZ
    OY();

  mobiusCore_EXPORT static core_XYZ
    OZ();

// Construction & destruction:
public:

  //! Default constructor. Initializes point coordinates with origin values:
  //! (0, 0, 0).
  core_XYZ()
  {
    m_fX = m_fY = m_fZ = 0.0;
  }

  //! Constructor with coordinates.
  //! \param x [in] first coordinate.
  //! \param y [in] second coordinate.
  //! \param z [in] third coordinate.
  core_XYZ(const adouble x,
           const adouble y,
           const adouble z)
  {
    m_fX = x;
    m_fY = y;
    m_fZ = z;
  }

  //! Assignment constructor.
  //! \param XYZ [in] point to assign to this one.
  core_XYZ(const core_XYZ& XYZ)
  {
    m_fX = XYZ.X();
    m_fY = XYZ.Y();
    m_fZ = XYZ.Z();
  }

  //! Destructor.
  ~core_XYZ() {}

public:

  //! Returns X coordinate of the 3D point.
  //! \return X coordinate.
  adouble X() const
  {
    return m_fX;
  }

  //! Sets X coordinate.
  //! \param x [in] value to set.
  void SetX(const adouble x)
  {
    m_fX = x;
  }

  //! Returns Y coordinate of the 3D point.
  //! \return Y coordinate.
  adouble Y() const
  {
    return m_fY;
  }

  //! Sets Y coordinate.
  //! \param y [in] value to set.
  void SetY(const adouble y)
  {
    m_fY = y;
  }

  //! Returns Z coordinate of the 3D point.
  //! \return Z coordinate.
  adouble Z() const
  {
    return m_fZ;
  }

  //! Sets Z coordinate.
  //! \param z [in] value to set.
  void SetZ(const adouble z)
  {
    m_fZ = z;
  }

  //! Returns coordinate by its 0-based index.
  //! \param idx [in] 0 for X, 1 for Y, 2 for Z.
  //! \return requested coordinate.
  adouble Coord(const int idx) const
  {
    if ( idx == 0 )
      return this->X();

    if ( idx == 1 )
      return this->Y();

    if ( idx == 2 )
      return this->Z();

    return FLT_MAX;
  }

  //! Updates coordinate having the specified 0-based index with the
  //! passed value.
  //! \param idx [in] 0 for X, 1 for Y, 2 for Z.
  //! \param val [in] value to set.
  adouble SetCoord(const int        idx,
                  const adouble val)
  {
    adouble* coord = NULL;

    if ( idx == 0 )
      coord = &m_fX;
    if ( idx == 1 )
      coord = &m_fY;
    if ( idx == 2 )
      coord = &m_fZ;

    if ( coord )
      *coord = val;

    return FLT_MAX;
  }

  //! Returns true if this vector is zero.
  //! \param tol3D [in] three-dimensional tolerance for comparison.
  //! \return true/false.
  bool IsOrigin(const adouble tol3D = 0) const
  {
    return ( fabs(m_fX) <= tol3D ) &&
           ( fabs(m_fY) <= tol3D ) &&
           ( fabs(m_fZ) <= tol3D );
  }

public:

  mobiusCore_EXPORT adouble
    Modulus() const;

  mobiusCore_EXPORT adouble
    SquaredModulus() const;

  mobiusCore_EXPORT core_XYZ
    Multiplied(const adouble coeff) const;

  mobiusCore_EXPORT void
    Normalize();

  mobiusCore_EXPORT core_XYZ
    Normalized() const;

  mobiusCore_EXPORT adouble
    Dot(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ
    Cross(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT adouble
    Angle(const core_XYZ& XYZ) const;

public:

  mobiusCore_EXPORT core_XYZ&
    operator=(const core_XYZ& XYZ);

  mobiusCore_EXPORT core_XYZ
    operator*(const adouble coeff) const;

  mobiusCore_EXPORT core_XYZ
    operator*=(const adouble coeff);

  mobiusCore_EXPORT core_XYZ
    operator/(const adouble coeff) const;

  mobiusCore_EXPORT core_XYZ
    operator/=(const adouble coeff);

  mobiusCore_EXPORT core_XYZ
    operator+(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ&
    operator+=(const core_XYZ& XYZ);

  mobiusCore_EXPORT core_XYZ
    Invert() const;

  mobiusCore_EXPORT core_XYZ
    operator-(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ&
    operator-=(const core_XYZ& XYZ);

private:

  adouble m_fX; //!< X coordinate.
  adouble m_fY; //!< Y coordinate.
  adouble m_fZ; //!< Z coordinate.

};

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_XYZ xyz;

};

#endif
