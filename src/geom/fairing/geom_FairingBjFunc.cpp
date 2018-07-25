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
#include <mobius/geom_FairingBjFunc.h>

// BSpl includes
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

mobius::geom_FairingBjFunc::geom_FairingBjFunc(const ptr<bcurve>&         curve,
                                               const int                  coord,
                                               const std::vector<double>& U,
                                               const int                  p,
                                               const int                  i,
                                               const double               lambda,
                                               core_HeapAlloc2D<double>*  alloc)
: geom_FairingCoeffFunc (lambda),
  m_U                   (U),
  m_curve               (curve),
  m_iCoord              (coord),
  m_iDegree             (p),
  m_iIndex              (i),
  m_alloc               (alloc)
{}

//-----------------------------------------------------------------------------

double mobius::geom_FairingBjFunc::Eval(const double u) const
{
  /* ==================================================================
   *  Calculate 2-nd derivative of basis spline at the given parameter
   * ================================================================== */

  const int order    = m_iDegree + 1;
  double    d2N      = 0.0;
  double    d2C_proj = 0.0;

  // Find span and index of the first non-vanishing spline.
  bspl_FindSpan FindSpan(m_U, m_iDegree);
  //
  int basisIndex = 0;
  int I          = FindSpan(u, basisIndex);

  bspl_EffectiveNDers Eval;

  // Prepare matrix.
  double** dN = m_alloc->Access(0).Ptr;

  // Evaluate.
  Eval(u, m_U, m_iDegree, I, 2, dN);

#if defined COUT_DEBUG
  // Dump.
  std::cout << "---" << std::endl;
  std::cout << "\tFirst Non-Zero Index for " << u << " is " << basisIndex << std::endl;
  for ( int kk = 0; kk < order; ++kk )
  {
    std::cout << "\t" << dN[2][kk];
  }
  std::cout << std::endl;
#endif

  // For indices in a band of width (p + 1), we can query what
  // Mobius returns. Otherwise, the derivative vanishes.
  if ( (m_iIndex >= basisIndex) && (m_iIndex < basisIndex + order) )
    d2N = dN[2][m_iIndex - basisIndex];

  /* ====================================================
   *  Calculate 2-nd derivative of the curve in question
   * ==================================================== */

  xyz d2C;
  m_curve->Eval_Dk(u, 2, d2C);

  // Get projection of the second derivative.
  d2C_proj = d2C.Coord(m_iCoord);

  // Calculate the result.
  return this->GetLambda()*d2N*d2C_proj;
}
