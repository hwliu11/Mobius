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

#ifndef geom_FairBSurfBl_HeaderFile
#define geom_FairBSurfBl_HeaderFile

// Geom includes
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_FairBSurfCoeff.h>
#include <mobius/geom_FairBSurfNk.h>

// Core includes
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Univariate function to interface fairing rhs coefficients B_l.
class geom_FairBSurfBl : public geom_FairBSurfCoeff
{
public:

  //! ctor.
  //! \param[in] surface B-spline surface in question (the one to fair).
  //! \param[in] coord   index of coordinate to use (0 for X, 1 for Y, and 2 for Z).
  //! \param[in] l       0-based index.
  //! \param[in] Nk      evaluators for functions \f$N_l(u,v)\f$.
  //! \param[in] lambda  fairing coefficent.
  //! \param[in] alloc   shared memory allocator.
  mobiusGeom_EXPORT
    geom_FairBSurfBl(const ptr<bsurf>&                           surface,
                     const int                                   coord,
                     const int                                   l,
                     const std::vector< ptr<geom_FairBSurfNk> >& Nk,
                     const double                                lambda,
                     ptr<alloc2d>                                alloc);

public:

  //! Evaluates function.
  //! \return value.
  mobiusGeom_EXPORT virtual double
    Eval(const double u, const double v) const;

private:

  geom_FairBSurfBl() = delete;
  void operator=(const geom_FairBSurfBl&) = delete;

protected:

  int                                         m_iL;      //!< Index of basis function.
  const std::vector< ptr<geom_FairBSurfNk> >& m_Nk;      //!< Evaluators of basis functions.
  ptr<bsurf>                                  m_surface; //!< Surface in question.
  int                                         m_iCoord;  //!< Coordinate in question.
  ptr<alloc2d>                                m_alloc;   //!< Shared memory allocator.

};

};

#endif
