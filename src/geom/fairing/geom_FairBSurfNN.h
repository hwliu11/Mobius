//-----------------------------------------------------------------------------
// Created on: 20 August 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

#ifndef geom_FairBSurfNN_HeaderFile
#define geom_FairBSurfNN_HeaderFile

// Geom includes
#include <mobius/geom.h>

// Core includes
#include <mobius/core_OBJECT.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Function representing a product \f$N_k(u,v)\f$ of two B-spline basis
//! functions \f$N_i^p(u)\f$ and \f$N_j^q(v)\f$ with their derivatives.
//!
//! \f[ \textbf{s}(u,v) = \sum_i \sum_j \textbf{P}_{ij} N_i^p(u) N_j^q(v) = \sum_k \textbf{C}_k N_k(u,v) \f]
//!
//! To use functions \f$N_k(u,v)\f$, all control points of the surface \f$\textbf{s}(u,v)\f$
//! have to be renumbered.
class geom_FairBSurfNN : public core_OBJECT
{
public:

  //! \brief Ctor accepting all properties of the involved i-th and j-th
  //!        basis functions.
  //! \param[in] U knot vector of the i-th function.
  //! \param[in] V knot vector of the j-th function.
  //! \param[in] p degree of the i-th function.
  //! \param[in] q degree of the j-th function.
  //! \param[in] i zero-based index of the i-th function.
  //! \param[in] j zero-based index of the j-th function.
  geom_FairBSurfNN(const std::vector<double>& U,
                   const std::vector<double>& V,
                   const int                  p,
                   const int                  q,
                   const int                  i,
                   const int                  j)
  : core_OBJECT()
  {
    // Initialize Ni.
    m_Ni.U = U;
    m_Ni.p = p;
    m_Ni.i = i;

    // Initialize Nj.
    m_Nj.V = V;
    m_Nj.q = q;
    m_Nj.j = j;
  }

public:

  //! Evaluates function up to its second derivatives.
  //! \param[in]  u       first parameter value.
  //! \param[in]  v       second parameter value.
  //! \param[out] N       primal value.
  //! \param[out] dN_dU   first-order derivative by U.
  //! \param[out] dN_dV   first-order derivative by V.
  //! \param[out] d2N_dU2 second-order derivative by U.
  //! \param[out] d2N_dUV second-order derivative by U and V.
  //! \param[out] d2N_dV2 second-order derivative by V.
  mobiusGeom_EXPORT virtual void
    Eval_D2(const double u,
            const double v,
            double&      N,
            double&      dN_dU,
            double&      dN_dV,
            double&      d2N_dU2,
            double&      d2N_dUV,
            double&      d2N_dV2) const;

protected:

  //! First basis function \f$N_i^p(u)\f$.
  struct
  {
    std::vector<double> U; //!< Knot vector.
    int                 p; //!< Degree.
    int                 i; //!< 0-based index.
  } m_Ni;

  //! Second basis function \f$N_j^q(v)\f$.
  struct
  {
    std::vector<double> V; //!< Knot vector.
    int                 q; //!< Degree.
    int                 j; //!< 0-based index.
  } m_Nj;

};

};

#endif
