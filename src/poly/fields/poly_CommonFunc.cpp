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

// Poly includes
#include <mobius/poly_CommonFunc.h>

// Standard includes
#include <algorithm>

//-----------------------------------------------------------------------------

mobius::poly_CommonFunc::poly_CommonFunc(const t_ptr<poly_RealFunc>& opLeft,
                                         const t_ptr<poly_RealFunc>& opRight)
//
: poly_BooleanFunc(opLeft, opRight)
{
  const t_xyz& leftDomMin  = opLeft  ->GetDomainMin();
  const t_xyz& leftDomMax  = opLeft  ->GetDomainMax();
  const t_xyz& rightDomMin = opRight ->GetDomainMin();
  const t_xyz& rightDomMax = opRight ->GetDomainMax();

  // Define the extended domain of the union function.
  m_domainMin = leftDomMin.CWiseMin(rightDomMin);
  m_domainMax = leftDomMax.CWiseMax(rightDomMax);
}

//-----------------------------------------------------------------------------

double mobius::poly_CommonFunc::Eval(const double x,
                                     const double y,
                                     const double z) const
{
  const double l = m_opLeft ->Eval(x, y, z);
  const double r = m_opRight->Eval(x, y, z);

  return std::max(l, r);
}
