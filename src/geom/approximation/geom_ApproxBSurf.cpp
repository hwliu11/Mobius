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
#include <mobius/geom_ApproxBSurf.h>

// BSpl includes
#include <mobius/bspl_KnotsUniform.h>

//-----------------------------------------------------------------------------

mobius::geom_ApproxBSurf::geom_ApproxBSurf(const t_ptr<t_pcloud>& points,
                                           const int              uDegree,
                                           const int              vDegree,
                                           const int              numPolesU,
                                           const int              numPolesV,
                                           core_ProgressEntry     progress,
                                           core_PlotterEntry      plotter)
: core_OPERATOR(progress, plotter)
{
  m_inputPoints = points;
  m_iDegreeU    = uDegree;
  m_iDegreeV    = vDegree;
  m_iNumPolesU  = numPolesU;
  m_iNumPolesV  = numPolesV;
}

//-----------------------------------------------------------------------------

bool mobius::geom_ApproxBSurf::Perform()
{
  t_ptr<t_alloc2d> alloc = new t_alloc2d;

  // Prepare twovariate basis functions.
  this->prepareNk(alloc);

  // Prepare knot vectors.
  int r, s;
  //
  if ( bspl_KnotsUniform::Calculate(m_iNumPolesU - 1, m_iDegreeU, r, m_U) != bspl_KnotsUniform::ErrCode_NoError )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot compute knot vector U.");
    return false;
  }
  //
  if ( bspl_KnotsUniform::Calculate(m_iNumPolesV - 1, m_iDegreeV, s, m_V) != bspl_KnotsUniform::ErrCode_NoError )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot compute knot vector V.");
    return false;
  }

  // TODO: NYI

  return false;
}

//-----------------------------------------------------------------------------

void mobius::geom_ApproxBSurf::prepareNk(t_ptr<t_alloc2d> alloc)
{
  const int nPoles = m_iNumPolesU*m_iNumPolesV;

  for ( int k = 0; k < nPoles; ++k )
  {
    // Prepare evaluator for N_k(u,v).
    t_ptr<geom_BSurfNk>
      Nk = new geom_BSurfNk(m_U, m_iDegreeU, m_V, m_iDegreeV, k, m_iNumPolesV - 1, alloc);
    //
    m_Nk.push_back(Nk);
  }
}
