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

#ifndef geom_ApproxBSurfMij_HeaderFile
#define geom_ApproxBSurfMij_HeaderFile

// Geometry includes
#include <mobius/geom_ApproxBSurfCoeff.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Twovariate function to interface approximation coefficients \f$M_{i,j}\f$.
class geom_ApproxBSurfMij : public geom_ApproxBSurfCoeff
{
public:

  //! Ctor.
  //! \param[in] i   0-based index 1.
  //! \param[in] j   0-based index 2.
  //! \param[in] UVs parameterization of the data points to approximate.
  //! \param[in] Nk  evaluators for functions \f$N_i(u,v)\f$ and \f$N_j(u,v)\f$.
  mobiusGeom_EXPORT
    geom_ApproxBSurfMij(const int                                 i,
                        const int                                 j,
                        const std::vector<t_uv>&                  UVs,
                        const std::vector< t_ptr<geom_BSurfNk> >& Nk);

public:

  //! Evaluates coefficient.
  //! \return calculated value.
  mobiusGeom_EXPORT double
    Eval();

protected:

  //! Evaluates product of \f$N_i(u,v) N_j(u,v)\f$ in the given
  //! parameter's pair \f$(u,v)\f$.
  //! \param[in] u first parameter.
  //! \param[in] v second parameter.
  //! \return evaluation result.
  mobiusGeom_EXPORT double
    eval_Ni_Nj(const double u, const double v);

private:

  geom_ApproxBSurfMij() = delete;
  void operator=(const geom_ApproxBSurfMij&) = delete;

protected:

  int m_iJ; //!< J index.
  int m_iI; //!< I index.

};

};

#endif
