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
//! Univariate function to interface fairing coefficients A_{k,l}.
class geom_FairBSurfAkl : public geom_FairBSurfCoeff
{
public:

  //! ctor.
  //! \param[in] k      0-based index 1.
  //! \param[in] l      0-based index 2.
  //! \param[in] lambda fairing coefficent.
  //! \param[in] Nk     evaluators for functions \f$N_k(u,v)\f$ and \f$N_l(u,v)\f$.
  mobiusGeom_EXPORT
    geom_FairBSurfAkl(const int                               k,
                      const int                               l,
                      const double                            lambda,
                      const std::vector< ptr<geom_BSurfNk> >& Nk);

public:

  //! Evaluates function.
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

  int                                     m_iK; //!< K index.
  int                                     m_iL; //!< L index.
  const std::vector< ptr<geom_BSurfNk> >& m_Nk; //!< Pre-computed basis functions.

};

};

#endif
