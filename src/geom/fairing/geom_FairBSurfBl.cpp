//-----------------------------------------------------------------------------
// Created on: 03 March 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018, Sergey Slyadnev
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
#include <mobius/geom_FairBSurfBl.h>

// Geom includes
#include <mobius/geom_FairingMemBlocks.h>

// BSpl includes
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

mobius::geom_FairBSurfBl::geom_FairBSurfBl(const ptr<bsurf>& surface,
                                           const int         coord,
                                           const int         l,
                                           const int         numCols,
                                           const double      lambda,
                                           ptr<alloc2d>      alloc)
: geom_FairBSurfCoeff (lambda),
  m_surface           (surface),
  m_iCoord            (coord),
  m_alloc             (alloc)
{
  // Prepare evaluator for N_l(u,v).
  int l_i, l_j;
  bspl::PairIndicesFromSerial(l, numCols, l_i, l_j);
  //
  m_Nl = new geom_FairBSurfNN(surface->Knots_U(),
                              surface->Knots_V(),
                              surface->Degree_U(),
                              surface->Degree_V(),
                              l_i,
                              l_j);
}

//-----------------------------------------------------------------------------

double mobius::geom_FairBSurfBl::Eval(const double u, const double v) const
{
  // Evaluate B-surface.
  xyz S, dS_dU, dS_dV, d2S_dU2, d2S_dUV, d2S_dV2;
  m_surface->Eval_D2(u, v, S, dS_dU, dS_dV, d2S_dU2, d2S_dUV, d2S_dV2);

  // Evaluate function N_l(u,v).
  double Nl, dNl_dU, dNl_dV, d2Nl_dU2, d2Nl_dUV, d2Nl_dV2;
  m_Nl->Eval_D2(u, v, Nl, dNl_dU, dNl_dV, d2Nl_dU2, d2Nl_dUV, d2Nl_dV2);

  // Calculate result.
  const xyz res =  d2S_dU2 * d2Nl_dU2 * m_fLambda
                +  d2S_dUV * d2Nl_dUV * m_fLambda * 2.
                +  d2S_dV2 * d2Nl_dV2 * m_fLambda;
  //
  const double res_proj = res.Coord(m_iCoord);

  return res_proj;
}
