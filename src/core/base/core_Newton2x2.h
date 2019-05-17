//-----------------------------------------------------------------------------
// Created on: 17 May 2019
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

#ifndef core_Newton2x2_HeaderFile
#define core_Newton2x2_HeaderFile

// Core includes
#include <mobius/core_IAlgorithm.h>
#include <mobius/core_TwovariateFunc.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Newton method for solving `F(x) = 0` where `F = (f, g)` is a two-dimensional
//! function of two variables `u` and `v` (hence the dimension of the problem
//! is 2x2).
class core_Newton2x2 : public core_IAlgorithm
{
public:

  //! Ctor.
  //! \param[in] f first component of the objective vector function.
  //! \param[in] g second component of the objective vector function.
  core_Newton2x2(const core_Ptr<core_TwovariateFunc>& f,
                 const core_Ptr<core_TwovariateFunc>& g)
  : core_IAlgorithm()
  {}

public:

  bool Perform()

};

};

#endif
