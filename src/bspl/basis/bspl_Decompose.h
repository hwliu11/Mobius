//-----------------------------------------------------------------------------
// Created on: 10 December 2018
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

#ifndef bspl_Decompose_HeaderFile
#define bspl_Decompose_HeaderFile

// bspl includes
#include <mobius/bspl_ParamDirection.h>

// core includes
#include <mobius/core_UV.h>
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Decomposition algorithm from "The NURBS Book".
class bspl_Decompose
{
public:

  //! Implements decomposition algorithm for curves (A5.6).
  //!
  //! \param[in]  n           index of the last pole in the original curve.
  //! \param[in]  p           degree.
  //! \param[in]  U           knot vector.
  //! \param[in]  Pw          controls points.
  //! \param[out] nb          number of Bezier segments.
  //! \param[out] Qw          poles of the resulting Bezier segments.
  //! \param[out] breakpoints indices of the knots where the original B-curve
  //!                         is split. The number of values in this collection
  //!                         is equal to the number of Bezier segments `nb`.
  //!
  //! \return true in case of success, false -- otherwise.
  mobiusBSpl_EXPORT bool
    operator()(const int                        n,
               const int                        p,
               const std::vector<double>&       U,
               const std::vector<xyz>&          Pw,
               int&                             nb,
               std::vector< std::vector<xyz> >& Qw,
               std::vector<int>&                breakpoints) const;

};

};

#endif
