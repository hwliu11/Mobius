//-----------------------------------------------------------------------------
// Created on: 03 March 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018, Sergey Slyadnev
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

#ifndef geom_FairBCurveBi_HeaderFile
#define geom_FairBCurveBi_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_FairBCurveCoeff.h>

// Core includes
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Univariate function to interface fairing rhs coefficients \f$B_i\f$.
class geom_FairBCurveBi : public geom_FairBCurveCoeff
{
public:

  //! ctor.
  //! \param[in] curve  B-spline curve in question (the one to fair).
  //! \param[in] coord  index of coordinate to use (0 for X, 1 for Y, and 2 for Z).
  //! \param[in] i      0-based index of the B-spline function.
  //! \param[in] lambda fairing coefficent.
  //! \param[in] alloc  shared memory allocator.
  mobiusGeom_EXPORT
    geom_FairBCurveBi(const t_ptr<t_bcurve>& curve,
                      const int              coord,
                      const int              i,
                      const double           lambda,
                      t_ptr<t_alloc2d>       alloc);

public:

  //! Evaluates function.
  //! \param[in] u parameter `u` on the curve being faired.
  //! \return value.
  mobiusGeom_EXPORT virtual double
    Eval(const double u) const;

private:

  geom_FairBCurveBi() = delete;
  void operator=(const geom_FairBCurveBi&) = delete;

protected:

  t_ptr<t_bcurve>  m_curve;  //!< Curve in question.
  int              m_iCoord; //!< Coordinate in question.
  int              m_iIndex; //!< 0-based index of the spline function.
  t_ptr<t_alloc2d> m_alloc;  //!< Shared memory allocator.

};

};

#endif
