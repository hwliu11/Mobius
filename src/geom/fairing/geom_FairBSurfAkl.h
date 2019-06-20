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

#ifndef geom_FairBSurfAkl_HeaderFile
#define geom_FairBSurfAkl_HeaderFile

// Geometry includes
#include <mobius/geom_BSurfNk.h>
#include <mobius/geom_FairBSurfCoeff.h>

// Core includes
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Twovariate function to interface fairing coefficients \f$A_{k,l}\f$. These
//! coefficients are used to compose the matrix for the linear system in such
//! algorithms as surface fairing (mobius::geom_FairBSurf) and surface
//! approximation from scattered points (mobius::geom_ApproxBSurf). According
//! to the paper
//!
//! [Kallay, M. 1993. Constrained Optimization in Surface Design.
//!  Proceedings of Modeling in Computer Graphics. Springer-Verlag.]
//!
//! the coefficients are formulated as follows:
//!
//! \f[ A_{k,l} = \sum_iE_i \int\int{\lambda \frac{\partial^2{N_k}}{\partial{u}^{2-i}\partial{v}^{i}} \frac{\partial^2{N_l}}{\partial{u}^{2-i}\partial{v}^{i}} + N_k N_l du dv} \f]
//!
//! You may refer to the original paper by M. Kallay for the conventions. Here,
//! we only draw your attention to the fact that \f$A_{k,l}\f$ coefficients
//! are composed of two terms. The second term \f$N_k N_l\f$ is due to the
//! original formulation of the constrained optimization problem which employs
//! deviation \f$(\textbf{s}-\textbf{s}_0)\f$ between the optimized surface
//! \f$\textbf{s}\f$ and the original surface \f$\textbf{s}_0\f$. If, however,
//! the smoothing terms are employed in a least squares formulation
//! (see mobius::geom_ApproxBSurf), only the first term of \f$A_{k,l}\f$ should
//! be used.
class geom_FairBSurfAkl : public geom_FairBSurfCoeff
{
public:

  //! Ctor.
  //! \param[in] k           0-based index 1.
  //! \param[in] l           0-based index 2.
  //! \param[in] lambda      fairing coefficent.
  //! \param[in] Nk          evaluators for functions \f$N_k(u,v)\f$ and
  //!                        \f$N_l(u,v)\f$.
  //! \param[in] pureFairing indicates whether the coefficients are used in
  //!                        pure fairing formulation. If not, additional
  //!                        term induced by squared deviation functional
  //!                        will be used.
  mobiusGeom_EXPORT
    geom_FairBSurfAkl(const int                                 k,
                      const int                                 l,
                      const double                              lambda,
                      const std::vector< t_ptr<geom_BSurfNk> >& Nk,
                      const bool                                pureFairing);

public:

  //! Evaluates function.
  //! \param[in] u first argument as `u` coordinate from `(u,v)` pair.
  //! \param[in] v second argument as `v` coordinate from `(u,v)` pair.
  //! \return value.
  mobiusGeom_EXPORT virtual double
    Eval(const double u, const double v) const;

public:

  //! Returns the indices of knot spans whether the function \f$N_k(u,v)\f$
  //! is not zero. See for example P2.1 at p. 55 in "The NURBS Book".
  virtual void GetSupportSpans(int& ifirst, int& ilast,
                               int& jfirst, int& jlast) const
  {
    int Nk_supportArea[4];
    m_Nk[m_iK]->GetSupportSpans(Nk_supportArea[0], Nk_supportArea[1],
                                Nk_supportArea[2], Nk_supportArea[3]);

    int Nl_supportArea[4];
    m_Nk[m_iL]->GetSupportSpans(Nl_supportArea[0], Nl_supportArea[1],
                                Nl_supportArea[2], Nl_supportArea[3]);

    ifirst = max(Nk_supportArea[0], Nl_supportArea[0]);
    ilast  = min(Nk_supportArea[1], Nl_supportArea[1]);
    jfirst = max(Nk_supportArea[2], Nl_supportArea[2]);
    jlast  = min(Nk_supportArea[3], Nl_supportArea[3]);
  }

private:

  geom_FairBSurfAkl() = delete;
  void operator=(const geom_FairBSurfAkl&) = delete;

protected:

  //! K index.
  int m_iK;

  //! L index.
  int m_iL;

  //! Pre-computed basis functions.
  const std::vector< t_ptr<geom_BSurfNk> >& m_Nk;

  //! Indicates whether the deviation term should be excluded.
  bool m_bPureFairing;

};

};

#endif
