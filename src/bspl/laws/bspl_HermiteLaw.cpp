//-----------------------------------------------------------------------------
// Created on: 14 December 2018
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

// Own include
#include <mobius/bspl_HermiteLaw.h>

//-----------------------------------------------------------------------------

mobius::bspl_HermiteLaw::bspl_HermiteLaw(const int idx)
{
  m_iIdx = idx;
}

//-----------------------------------------------------------------------------

double mobius::bspl_HermiteLaw::Eval(const double u) const
{
  double res = 0;
  switch ( m_iIdx )
  {
    case 0:
      res = 1.0 - 3.0*pow(u,2) + 2.0*pow(u,3);
      break;
    case 1:
      res = 3.0*pow(u,2) - 2.0*pow(u,3);
      break;
    case 2:
      res = u - 2.0*pow(u,2) + pow(u,3);
      break;
    case 3:;
      res = -pow(u,2) + pow(u,3);
      break;
    default:
      res = 0.0;
      break;
  };

  return res;
}

//-----------------------------------------------------------------------------

double mobius::bspl_HermiteLaw::Eval_D1(const double u) const
{
  double res = 0;
  switch ( m_iIdx )
  {
    case 0:
      res = -6.0*u + 6.0*pow(u,2);
      break;
    case 1:
      res = 6.0*u - 6.0*pow(u,2);
      break;
    case 2:
      res = 1 - 4.0*u + 3.0*pow(u,2);
      break;
    case 3:;
      res = -2.0*u + 3.0*pow(u,2);
      break;
    default:
      res = 0.0;
      break;
  };

  return res;
}
