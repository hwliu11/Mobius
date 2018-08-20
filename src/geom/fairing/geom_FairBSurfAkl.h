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
#include <mobius/geom_FairBSurfCoeff.h>
#include <mobius/geom_FairBSurfNN.h>

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
  //! \param[in] U       knot vector in U parametric direction.
  //! \param[in] V       knot vector in V parametric direction.
  //! \param[in] p       B-spline degree in U parametric direction.
  //! \param[in] q       B-spline degree in V parametric direction.
  //! \param[in] k       0-based index 1.
  //! \param[in] l       0-based index 2.
  //! \param[in] numCols number of poles in V direction (used to convert indices).
  //! \param[in] lambda  fairing coefficent.
  //! \param[in] alloc   shared memory allocator.
  mobiusGeom_EXPORT
    geom_FairBSurfAkl(const std::vector<double>& U,
                      const std::vector<double>& V,
                      const int                  p,
                      const int                  q,
                      const int                  k,
                      const int                  l,
                      const int                  numCols,
                      const double               lambda,
                      ptr<alloc2d>               alloc);

public:

  //! Evaluates function.
  //! \return value.
  mobiusGeom_EXPORT virtual double
    Eval(const double u, const double v) const;

private:

  geom_FairBSurfAkl() = delete;
  void operator=(const geom_FairBSurfAkl&) = delete;

protected:

  int                   m_iNumCols; //!< Number of poles in V direction of surface.
  ptr<geom_FairBSurfNN> m_Nk;       //!< B-spline product function \f$N_k(u,v)\f$.
  ptr<geom_FairBSurfNN> m_Nl;       //!< B-spline product function \f$N_l(u,v)\f$.
  ptr<alloc2d>          m_alloc;    //!< Allocator with reserved memory blocks.

};

};

#endif
