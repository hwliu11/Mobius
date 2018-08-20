//-----------------------------------------------------------------------------
// Created on: 20 August 2018
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
#include <mobius/geom_FairBSurfNN.h>

// BSpl includes
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>

// Core includes
#include <mobius/core_HeapAlloc.h>

//-----------------------------------------------------------------------------

void mobius::geom_FairBSurfNN::Eval_D2(const double u,
                                       const double v,
                                       double&      N,
                                       double&      dN_dU,
                                       double&      dN_dV,
                                       double&      d2N_dU2,
                                       double&      d2N_dUV,
                                       double&      d2N_dV2) const
{
  const int orderU = m_Ni.p + 1;
  const int orderV = m_Nj.q + 1;
  double    Ni     = 0.0;
  double    Nj     = 0.0;
  double    d1Ni   = 0.0;
  double    d1Nj   = 0.0;
  double    d2Ni   = 0.0;
  double    d2Nj   = 0.0;

  // Find span and index of the first non-vanishing spline.
  bspl_FindSpan FindSpanU(m_Ni.U, m_Ni.p),
                FindSpanV(m_Nj.V, m_Nj.q);
  //
  int basisIndexU = 0, basisIndexV = 0;
  int Iu          = FindSpanU(u, basisIndexU);
  int Iv          = FindSpanV(v, basisIndexV);

  // Prepare matrix.
  ptr<alloc2d> localAlloc = new alloc2d;
  //
  double** dNi, **dNj;
  //
  dNi = localAlloc->Allocate(3, orderU, true);
  dNj = localAlloc->Allocate(3, orderV, true);

  // Evaluate both functions: one for u and another for v.
  bspl_EffectiveNDers EvalNu(NULL, -1),
                      EvalNv(NULL, -1);
  //
  EvalNu(u, m_Ni.U, m_Ni.p, Iu, 2, dNi);
  EvalNv(v, m_Nj.V, m_Nj.q, Iv, 2, dNj);

  // Take value of N_i.
  if ( (m_Ni.i >= basisIndexU) && (m_Ni.i < basisIndexU + orderU) )
  {
    Ni   = dNi[0][m_Ni.i - basisIndexU];
    d1Ni = dNi[1][m_Ni.i - basisIndexU];
    d2Ni = dNi[2][m_Ni.i - basisIndexU];
  }

  // Take value of N_j.
  if ( (m_Nj.j >= basisIndexV) && (m_Nj.j < basisIndexV + orderV) )
  {
    Nj   = dNj[0][m_Nj.j - basisIndexV];
    d1Nj = dNj[1][m_Nj.j - basisIndexV];
    d2Nj = dNj[2][m_Nj.j - basisIndexV];
  }

  // Calculate results.
  N       = Ni*Nj;
  dN_dU   = d1Ni*Nj;
  dN_dV   = Ni*d1Nj;
  d2N_dU2 = d2Ni*Nj;
  d2N_dUV = d1Ni*d1Nj;
  d2N_dV2 = Ni*d2Nj;
}
