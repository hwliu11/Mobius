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

#ifndef geom_ApproxBSurfBi_HeaderFile
#define geom_ApproxBSurfBi_HeaderFile

// Geom includes
#include <mobius/geom_ApproxBSurfCoeff.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Twovariate function to interface approximation rhs coefficients \f$B_i\f$.
class geom_ApproxBSurfBi : public geom_ApproxBSurfCoeff
{
public:

  //! Ctor.
  //! \param[in] i   0-based index of the row in the matrix.
  //! \param[in] pts data points being approximated.
  //! \param[in] UVs parameterization of the input data points `pts`.
  //! \param[in] Nk  prepared evaluators for the basis functions.
  mobiusGeom_EXPORT
    geom_ApproxBSurfBi(const int                                 i,
                       const std::vector<t_xyz>&                 pts,
                       const std::vector<t_uv>&                  UVs,
                       const std::vector< t_ptr<geom_BSurfNk> >& Nk);

public:

  //! Evaluates the coefficient for the passed coordinate index.
  //! \param[in] coord coordinate index (0 -- X, 1 -- Y, 2 -- Z).
  //! \return evaluated coefficient.
  mobiusGeom_EXPORT double
    Eval(const int coord);

private:

  geom_ApproxBSurfBi() = delete;
  void operator=(const geom_ApproxBSurfBi&) = delete;

protected:

  int                       m_iI; //!< I index.
  const std::vector<t_xyz>& m_R;  //!< Points to approximate.

};

};

#endif
