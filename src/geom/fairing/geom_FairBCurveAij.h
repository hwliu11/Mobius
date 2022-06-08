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

#ifndef geom_FairBCurveAij_HeaderFile
#define geom_FairBCurveAij_HeaderFile

// Geometry includes
#include <mobius/geom_FairBCurveCoeff.h>

// Core includes
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Univariate function to interface fairing coefficients \f$A_{i,j}\f$.
class geom_FairBCurveAij : public geom_FairBCurveCoeff
{
public:

  //! ctor.
  //! \param[in] U      knot vector.
  //! \param[in] p      B-spline degree.
  //! \param[in] i      0-based index 1.
  //! \param[in] j      0-based index 2.
  //! \param[in] lambda fairing coefficent.
  //! \param[in] alloc  shared memory allocator.
  mobiusGeom_EXPORT
    geom_FairBCurveAij(const std::vector<double>& U,
                       const int                  p,
                       const int                  i,
                       const int                  j,
                       const double               lambda,
                       t_ptr<t_alloc2d>           alloc);

public:

  //! Evaluates function.
  //! \param[in] u parameter `u` on the curve being faired.
  //! \return value.
  mobiusGeom_EXPORT virtual double
    Eval(const double u) const;

private:

  geom_FairBCurveAij() = delete;
  void operator=(const geom_FairBCurveAij&) = delete;

protected:

  const std::vector<double>& m_U;       //!< Knot vector ("flat" knots).
  int                        m_iDegree; //!< Degree of the spline function.
  int                        m_iIndex1; //!< 0-based index 1.
  int                        m_iIndex2; //!< 0-based index 2.
  t_ptr<t_alloc2d>           m_alloc;   //!< Allocator with reserved memory blocks.

};

}

#endif
