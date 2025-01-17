//-----------------------------------------------------------------------------
// Created on: 02 November 2012
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

#ifndef core_UV_HeaderFile
#define core_UV_HeaderFile

// core includes
#include <mobius/core.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Geometric primitive for 2D point.
class core_UV
{
public:

  //! \return number of coordinates.
  static int num_coordinates()
  {
    return 2;
  }

// Construction & destruction:
public:

  mobiusCore_EXPORT
    core_UV();

  mobiusCore_EXPORT
    core_UV(const double u, const double v);

  mobiusCore_EXPORT
    core_UV(const core_UV& UV);

  mobiusCore_EXPORT virtual
    ~core_UV();

public:

  //! Returns U coordinate of the 2D point.
  //! \return U coordinate.
  double U() const
  {
    return m_fU;
  }

  //! Sets U coordinate.
  //! \param u [in] value to set.
  void SetU(const double u)
  {
    m_fU = u;
  }

  //! Returns V coordinate of the 2D point.
  //! \return V coordinate.
  double V() const
  {
    return m_fV;
  }

  //! Sets V coordinate.
  //! \param v [in] value to set.
  void SetV(const double v)
  {
    m_fV = v;
  }

  //! Returns coordinate by its 0-based index.
  //! \param idx [in] 0 for U, 1 for V.
  //! \return requested coordinate.
  double Coord(const int idx) const
  {
    if ( idx == 0 )
      return this->U();

    if ( idx == 1 )
      return this->V();

    return 0;
  }

  //! Updates coordinate having the specified 0-based index with the
  //! passed value.
  //! \param idx [in] 0 for U, 1 for V.
  //! \param val [in] value to set.
  double SetCoord(const int        idx,
                      const double val)
  {
    double* coord = nullptr;

    if ( idx == 0 )
      coord = &m_fU;
    if ( idx == 1 )
      coord = &m_fV;

    if ( coord )
      *coord = val;

    return 0;
  }

  //! \return max components of the tuple.
  double GetMaxComponent() const
  {
    return m_fU > m_fV ? m_fU : m_fV;
  }

public:

  mobiusCore_EXPORT double
    Modulus() const;

  mobiusCore_EXPORT double
    SquaredModulus() const;

  mobiusCore_EXPORT double
    Dot(const core_UV& UV) const;

public:

  mobiusCore_EXPORT core_UV&
    operator=(const core_UV& UV);

  mobiusCore_EXPORT core_UV
    operator*(const double coeff) const;

  mobiusCore_EXPORT core_UV
    operator*=(const double coeff);

  mobiusCore_EXPORT core_UV
    operator/(const double coeff) const;

  mobiusCore_EXPORT core_UV
    operator/=(const double coeff);

  mobiusCore_EXPORT core_UV
    operator+(const core_UV& UV) const;

  mobiusCore_EXPORT core_UV&
    operator+=(const core_UV& UV);

  mobiusCore_EXPORT core_UV
    Invert() const;

  mobiusCore_EXPORT core_UV
    operator-(const core_UV& UV) const;

  mobiusCore_EXPORT core_UV&
    operator-=(const core_UV& UV);

private:

  double m_fU; //!< U coordinate.
  double m_fV; //!< V coordinate.

};

//! \ingroup MOBIUS_CORE
//!
//! Multiplication operator which allows prefix form.
//! \param[in] coeff  coefficient to multiply by.
//! \param[in] coords coordinates tuple.
//! \return multiplication result.
inline core_UV operator*(const double coeff, const core_UV& coords)
{
  core_UV r = coords*coeff;
  return r;
}

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_UV t_uv;

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_UV core_XY;

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_XY t_xy;

}

#endif
