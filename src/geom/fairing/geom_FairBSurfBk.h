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

#ifndef geom_FairBSurfBk_HeaderFile
#define geom_FairBSurfBk_HeaderFile

// Geom includes
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_BSurfNk.h>
#include <mobius/geom_FairBSurfCoeff.h>

// Core includes
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Twovariate function to interface fairing rhs coefficients \f$B_k\f$.
class geom_FairBSurfBk : public geom_FairBSurfCoeff
{
public:

  //! Ctor.
  //! \param[in] surface B-spline surface in question (the one to fair).
  //! \param[in] coord   index of coordinate to use (0 for X, 1 for Y, and 2 for Z).
  //! \param[in] k       0-based index.
  //! \param[in] Nk      evaluators for functions \f$N_k(u,v)\f$.
  //! \param[in] lambda  fairing coefficent.
  //! \param[in] alloc   shared memory allocator.
  mobiusGeom_EXPORT
    geom_FairBSurfBk(const t_ptr<t_bsurf>&                     surface,
                     const int                                 coord,
                     const int                                 k,
                     const std::vector< t_ptr<geom_BSurfNk> >& Nk,
                     const double                              lambda,
                     t_ptr<t_alloc2d>                          alloc);

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
    m_Nk[m_iK]->GetSupportSpans(ifirst, ilast, jfirst, jlast);
  }

private:

  geom_FairBSurfBk() = delete;
  void operator=(const geom_FairBSurfBk&) = delete;

protected:

  int                                       m_iK;      //!< Index of basis function.
  const std::vector< t_ptr<geom_BSurfNk> >& m_Nk;      //!< Evaluators of basis functions.
  t_ptr<t_bsurf>                            m_surface; //!< Surface in question.
  int                                       m_iCoord;  //!< Coordinate in question.
  t_ptr<t_alloc2d>                          m_alloc;   //!< Shared memory allocator.

};

}

#endif
