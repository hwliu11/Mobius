//-----------------------------------------------------------------------------
// Created on: 19 July 2017
//-----------------------------------------------------------------------------
// Copyright (c) 2017-present, Sergey Slyadnev
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

// BSpl includes
#include <mobius/bspl.h>

//-----------------------------------------------------------------------------

int mobius::bspl::M(const int n, const int p)
{
  return n + p + 1; // Common formula.
}

//-----------------------------------------------------------------------------

int mobius::bspl::N(const int m, const int p)
{
  return m - p - 1; // Common formula.
}

//-----------------------------------------------------------------------------

int mobius::bspl::NumberOfKnots(const int n, const int p)
{
  return M(n, p) + 1;
}

//-----------------------------------------------------------------------------

bool mobius::bspl::Check(const int n, const int p)
{
  const int r = NumberOfKnots(n, p);
  //
  if ( r < 2*(p + 1) )
    return false;

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::bspl::Check(const int n, const int m, const int p)
{
  const int m_expected = NumberOfKnots(n, p) - 1;
  return m == m_expected;
}
