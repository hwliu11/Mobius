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
#include <mobius/geom_FairBSurfBk.h>

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

mobius::geom_FairBSurfBk::geom_FairBSurfBk(const t_ptr<t_bsurf>&                     surface,
                                           const int                                 coord,
                                           const int                                 k,
                                           const std::vector< t_ptr<geom_BSurfNk> >& Nk,
                                           const double                              lambda,
                                           t_ptr<t_alloc2d>                          alloc)
: geom_FairBSurfCoeff (lambda),
  m_surface           (surface),
  m_iCoord            (coord),
  m_iK                (k),
  m_Nk                (Nk),
  m_alloc             (alloc)
{}

//-----------------------------------------------------------------------------

double mobius::geom_FairBSurfBk::Eval(const double u, const double v) const
{
  // Evaluate B-surface.
  t_xyz S, dS_dU, dS_dV, d2S_dU2, d2S_dUV, d2S_dV2;
  m_surface->Eval_D2(u, v, S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV,
                     m_alloc,
                     memBlockSurf_BSplineSurfEvalD2U,
                     memBlockSurf_BSplineSurfEvalD2V,
                     memBlockSurf_BSplineSurfEvalInternal);

  // Evaluate function N_l(u,v).
  double Nl, dNl_dU, dNl_dV, d2Nl_dU2, d2Nl_dUV, d2Nl_dV2;
  m_Nk[m_iK]->Eval_D2(u, v, Nl, dNl_dU, dNl_dV, d2Nl_dU2, d2Nl_dUV, d2Nl_dV2);

  // Calculate result.
  const t_xyz res = d2S_dU2 * d2Nl_dU2 * m_fLambda
                  + d2S_dUV * d2Nl_dUV * m_fLambda * 2.
                  + d2S_dV2 * d2Nl_dV2 * m_fLambda;
  //
  const double res_proj = res.Coord(m_iCoord);

  return res_proj;
}
