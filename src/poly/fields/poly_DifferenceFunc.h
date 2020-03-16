//-----------------------------------------------------------------------------
// Created on: 15 March 2020
//-----------------------------------------------------------------------------
// Copyright (c) 2020-present, Sergey Slyadnev
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

#ifndef poly_DifferenceFunc_HeaderFile
#define poly_DifferenceFunc_HeaderFile

// Poly includes
#include <mobius/poly_BooleanFunc.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Difference function.
class poly_DifferenceFunc : public poly_BooleanFunc
{
public:

  //! Ctor accepting the operand functions.
  //! \param[in] opLeft  left operand function.
  //! \param[in] opRight right operand function.
  mobiusPoly_EXPORT
    poly_DifferenceFunc(const t_ptr<poly_RealFunc>& opLeft,
                        const t_ptr<poly_RealFunc>& opRight);

public:

  //! Evaluates the common difference in the given point of
  //! the ambient space.
  //! \param[in] x X coordinate of the point in question.
  //! \param[in] y Y coordinate of the point in question.
  //! \param[in] z Z coordinate of the point in question.
  //! \return function value.
  mobiusPoly_EXPORT virtual double
    Eval(const double x, const double y, const double z) const;

};

}

#endif
