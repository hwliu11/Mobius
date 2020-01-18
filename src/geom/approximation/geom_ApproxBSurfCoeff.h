//-----------------------------------------------------------------------------
// Created on: 16 June 2019
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

#ifndef geom_ApproxBSurfCoeff_HeaderFile
#define geom_ApproxBSurfCoeff_HeaderFile

// Geom includes
#include <mobius/geom_BSurfNk.h>

// Core includes
#include <mobius/core_Ptr.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Base class for surface approximation coefficients.
class geom_ApproxBSurfCoeff : public core_OBJECT
{
public:

  //! Ctor accepting the parameterization of the data points.
  //! \param[in] UVs parameterization of the data points.
  //! \param[in] Nk  evaluators of basis functions.
  geom_ApproxBSurfCoeff(const std::vector<t_uv>&                  UVs,
                        const std::vector< t_ptr<geom_BSurfNk> >& Nk)
  : core_OBJECT (),
    m_UVs       (UVs),
    m_Nk        (Nk)
  {}

private:

  geom_ApproxBSurfCoeff() = delete;
  void operator=(const geom_ApproxBSurfCoeff&) = delete;

protected:

  const std::vector<t_uv>&                  m_UVs; //!< Input parameterization.
  const std::vector< t_ptr<geom_BSurfNk> >& m_Nk;  //!< Pre-computed basis functions.

};

}

#endif
