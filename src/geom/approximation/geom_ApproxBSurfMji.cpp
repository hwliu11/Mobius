//-----------------------------------------------------------------------------
// Created on: 16 June 2019
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

// Own include
#include <mobius/geom_ApproxBSurfMji.h>

//-----------------------------------------------------------------------------

mobius::geom_ApproxBSurfMji::geom_ApproxBSurfMji(const int                                 j,
                                                 const int                                 i,
                                                 const std::vector<t_uv>&                  UVs,
                                                 const std::vector< t_ptr<geom_BSurfNk> >& Nk)
: geom_ApproxBSurfCoeff (UVs, Nk),
  m_iJ                  (j),
  m_iI                  (i)
{}

//-----------------------------------------------------------------------------

double mobius::geom_ApproxBSurfMji::Eval()
{
  // Sum products of N_i N_j for each data point.
  double res = 0.;
  for ( size_t k = 0; k < m_UVs.size(); ++k )
  {
    const double NjNi = this->eval_Nj_Ni( m_UVs[k].U(), m_UVs[k].V() );
    //
    res += NjNi;
  }

  return res;
}

//-----------------------------------------------------------------------------

double mobius::geom_ApproxBSurfMji::eval_Nj_Ni(const double u, const double v)
{
  // Evaluate function N_j(u,v).
  double Nj;
  m_Nk[m_iJ]->Eval(u, v, Nj);

  // Evaluate function N_i(u,v).
  double Ni;
  m_Nk[m_iI]->Eval(u, v, Ni);

  // Calculate result.
  const double res = Nj*Ni;
  //
  return res;
}
