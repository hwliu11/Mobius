//-----------------------------------------------------------------------------
// Created on: 28 February 2020
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

#ifndef poly_BooleanFunc_HeaderFile
#define poly_BooleanFunc_HeaderFile

// Poly includes
#include <mobius/poly_ImplicitFunc.h>

// Core includes
#include <mobius/core_Ptr.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Base class for Boolean functions, such as Union, Difference and Common.
class poly_BooleanFunc : public poly_ImplicitFunc
{
public:

  //! Default ctor.
  poly_BooleanFunc() : poly_ImplicitFunc() {}

  //! Ctor accepting the operand functions.
  //! \param[in] opLeft  left operand function.
  //! \param[in] opRight right operand function.
  poly_BooleanFunc(const t_ptr<poly_ImplicitFunc>& opLeft,
                   const t_ptr<poly_ImplicitFunc>& opRight)
  //
  : m_opLeft(opLeft), m_opRight(opRight) {}

protected:

  t_ptr<poly_ImplicitFunc> m_opLeft;  //!< Left operand of the Boolean function.
  t_ptr<poly_ImplicitFunc> m_opRight; //!< Right operand of the Boolean function.

};

}

#endif
