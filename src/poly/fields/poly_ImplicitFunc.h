//-----------------------------------------------------------------------------
// Created on: 11 December (*) 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

#ifndef poly_ImplicitFunc_HeaderFile
#define poly_ImplicitFunc_HeaderFile

// Poly includes
#include <mobius/poly.h>

// Core includes
#include <mobius/core_TrivariateFunc.h>
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Base class for implicit functions.
class poly_ImplicitFunc : public core_TrivariateFunc
{
public:

  //! Default ctor.
  poly_ImplicitFunc()
  //
  : core_TrivariateFunc (),
    m_domainMin         (-DBL_MAX, -DBL_MAX, -DBL_MAX),
    m_domainMax         ( DBL_MAX,  DBL_MAX,  DBL_MAX)
  {}

  //! Ctor which accepts the bounded region where the function is defined.
  //! \param[in] domainMin lower bound of the function domain.
  //! \param[in] domainMax upper bound of the function domain.
  poly_ImplicitFunc(const t_xyz& domainMin,
                    const t_xyz& domainMax)
  //
  : core_TrivariateFunc (),
    m_domainMin         (domainMin),
    m_domainMax         (domainMax)
  {}

public:

  //! \return lower bound of the function domain.
  const t_xyz& GetDomainMin() const
  {
    return m_domainMin;
  }

  //! \return upper bound of the function domain.
  const t_xyz& GetDomainMax() const
  {
    return m_domainMax;
  }

protected:

  t_xyz m_domainMin; //!< Min bound of the three-dimensional domain.
  t_xyz m_domainMax; //!< Max bound of the three-dimensional domain.

};

};

#endif
