//-----------------------------------------------------------------------------
// Created on: 23 May 2019
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

#ifndef core_SolveCovarianceEigens_HeaderFile
#define core_SolveCovarianceEigens_HeaderFile

// core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Solver for eigen vectors and values of a covariance matrix built for the
//! passed vector of points.
class core_SolveCovarianceEigens
{
// Construction & destruction:
public:

  //! Ctor.
  mobiusCore_EXPORT
    core_SolveCovarianceEigens();

  //! Dtor.
  mobiusCore_EXPORT virtual
    ~core_SolveCovarianceEigens();

public:

  //! Computes the eigen vectors of a covariance matrix built for the passed
  //! point set.
  //! \param[in]  pts    collection of points in question.
  //! \param[out] center barycenter of the point set.
  //! \param[out] Dx     1-st eigen vector.
  //! \param[out] Dy     2-nd eigen vector.
  //! \param[out] Dz     3-rd eigen vector.
  //! \return true in case of success, false -- otherwise.
  mobiusCore_EXPORT bool
    operator()(const std::vector<t_xyz>& pts,
               t_xyz&                    center,
               t_xyz&                    Dx,
               t_xyz&                    Dy,
               t_xyz&                    Dz);

};

};

#endif
