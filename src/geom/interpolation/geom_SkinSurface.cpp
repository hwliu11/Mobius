//-----------------------------------------------------------------------------
// Created on: 09 March 2015
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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
#include <mobius/geom_SkinSurface.h>

// Geometry includes
#include <mobius/geom_InterpolateCurve.h>
#include <mobius/geom_UnifyBCurves.h>

// BSpl includes
#include <mobius/bspl_KnotsAverage.h>
#include <mobius/bspl_ParamsCentripetal.h>
#include <mobius/bspl_ParamsChordLength.h>
#include <mobius/bspl_UnifyKnots.h>

//-----------------------------------------------------------------------------

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

mobius::geom_SkinSurface::geom_SkinSurface(core_ProgressEntry progress,
                                           core_PlotterEntry  plotter)
: core_OPERATOR (progress, plotter),
  m_iDeg_V      (1),
  m_bUnify      (false)
{
  m_errCode = ErrCode_NotInitialized;
}

//-----------------------------------------------------------------------------

mobius::geom_SkinSurface::geom_SkinSurface(const std::vector< t_ptr<t_bcurve> >& curves,
                                           const int                             deg_V,
                                           const bool                            unifyCurves,
                                           core_ProgressEntry                    progress,
                                           core_PlotterEntry                     plotter)
: core_OPERATOR(progress, plotter)
{
  this->Init(curves, deg_V, unifyCurves);
}

//-----------------------------------------------------------------------------

void mobius::geom_SkinSurface::Init(const std::vector< t_ptr<t_bcurve> >& curves,
                                    const int                             deg_V,
                                    const bool                            unifyCurves)
{
  m_curves  = curves;
  m_iDeg_V  = deg_V;
  m_bUnify  = unifyCurves;
  m_errCode = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

void mobius::geom_SkinSurface::AddLeadingTangencies(const std::vector<t_xyz>& tangencies)
{
  m_D1lead = tangencies;
}

//-----------------------------------------------------------------------------

void mobius::geom_SkinSurface::AddTrailingTangencies(const std::vector<t_xyz>& tangencies)
{
  m_D1tail = tangencies;
}

//-----------------------------------------------------------------------------

bool mobius::geom_SkinSurface::Perform()
{
  // Prepare sections.
  if ( !this->PrepareSections() )
    return false;

  // Build construction curves.
  if ( !this->BuildIsosU() )
    return false;

  // Construct interpolant surface.
  if ( !this->BuildSurface() )
    return false;

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::geom_SkinSurface::PrepareSections()
{
  m_errCode = ErrCode_NoError;

  if ( m_curves.size() < 2 )
  {
    m_errCode = ErrCode_NotEnoughCurves;
    return false;
  }

  // Prepare compatibility tool.
  geom_UnifyBCurves unifyCurves(m_progress, m_plotter);
  //
  for ( size_t c = 0; c < m_curves.size(); ++c )
    unifyCurves.AddCurve(m_curves[c]);

  // Check compatibility of curves.
  const bool areCompatible = unifyCurves.AreCompatible();
  //
  if ( !areCompatible && !m_bUnify )
  {
    m_errCode = ErrCode_NotCompatibleCurves;
    m_progress.SendLogMessage(MobiusErr(Normal) << "Incompatible curves should be unified.");
    return false;
  }

  // Now if the curves are not compatible, it is time to make them such.
  if ( !areCompatible && m_bUnify )
  {
    if ( !unifyCurves.Perform() )
    {
      m_errCode = ErrCode_NotDoneUnification;
      m_progress.SendLogMessage(MobiusErr(Normal) << "Unification process failed.");
      return false;
    }

    // Update curves.
    m_curves = unifyCurves.GetResult();
  }

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::geom_SkinSurface::BuildIsosU()
{
  // Working dimensions.
  const int n = (int) (m_curves[0]->GetPoles().size() - 1);
  const int K = (int) (m_curves.size() - 1);

  // Check if tangency constraints are defined.
  const bool isTangLead = (m_D1lead.size() > 0);
  const bool isTangTail = (m_D1tail.size() > 0);

  // Check if the V degree is suitable.
  if ( !bspl::Check(K + (isTangLead ? 1 : 0) + (isTangTail ? 1 : 0), m_iDeg_V) )
  {
    m_errCode = ErrCode_BadVDegree;
    return false;
  }

  /* -----------------------------------------------------------
   *  Choose parameters for u-isos by averaged centripetal rule
   * ----------------------------------------------------------- */

  // Prepare rectangular collection of control points.
  std::vector< std::vector<t_xyz> > Q;
  for ( int i = 0; i <= n; ++i )
  {
    std::vector<t_xyz> poles;
    for ( int c = 0; c < (int) m_curves.size(); ++c )
    {
      const t_ptr<t_bcurve>& crv = m_curves[c];
      poles.push_back( crv->GetPoles()[i] );
    }
    Q.push_back(poles);
  }

  // Allocate arrays for reper parameters.
  double* params_V = m_alloc.Allocate(K + 1, true);
  /*if ( bspl_ParamsCentripetal::Calculate_V(Q, params_V) != bspl_ParamsCentripetal::ErrCode_NoError )
  {
    m_errCode = ErrCode_CannotSelectParameters;
    return false;
  }*/
  if ( bspl_ParamsChordLength::Calculate_V(Q, params_V) != bspl_ParamsChordLength::ErrCode_NoError )
  {
    m_errCode = ErrCode_CannotSelectParameters;
    return false;
  }

  // Store the computed parameters.
  for ( int l = 0; l < K + 1; ++l )
    m_params.push_back(params_V[l]);

  /* ---------------------------
   *  Choose knots by averaging
   * --------------------------- */

  // Availability of tangency constraints makes it necessary to have
  // additional knots.
  const int m = bspl::M(K, m_iDeg_V) + (isTangLead ? 1 : 0) + (isTangTail ? 1 : 0);

  // Choose knots.
  m_V.resize(m + 1);
  //
  if ( bspl_KnotsAverage::Calculate(params_V, K, m_iDeg_V, m,
                                    bspl_KnotsAverage::Recognize(isTangLead,
                                                                 isTangTail,
                                                                 false,
                                                                 false),
                                    &m_V[0]) != bspl_KnotsAverage::ErrCode_NoError )
  {
    m_errCode = ErrCode_CannotSelectKnots;
    return false;
  }

  // Check if the resulting knot vector is clamped.
  if ( !bspl::CheckClampedKnots(m_V, m_iDeg_V) )
  {
    m_errCode = ErrCode_BadVDegree;
    return false;
  }

  /* -------------------------------------------------
   *  Interpolate poles of curves in iso-U directions
   * ------------------------------------------------- */

  IsoU_Curves.clear();
  for ( int i = 0; i <= n; ++i )
  {
    // Populate reper points (poles of curves) for fixed U values.
    std::vector<t_xyz> iso_U_poles;
    for ( int k = 0; k <= K; ++k )
      iso_U_poles.push_back(Q[i][k]);

    // Interpolate over these poles.
    t_ptr<t_bcurve> iso_U;
    if ( !geom_InterpolateCurve::Interp(iso_U_poles, K, m_iDeg_V, params_V, &m_V[0], m,
                                        isTangLead,
                                        isTangTail,
                                        false,
                                        false,
                                        isTangLead ? m_D1lead[i] : t_xyz(),
                                        isTangTail ? m_D1tail[i] : t_xyz(),
                                        t_xyz(),
                                        t_xyz(),
                                        iso_U) )
    {
      m_errCode = ErrCode_CannotInterpolateIsoU;
      return false;
    }

    IsoU_Curves.push_back(iso_U);
  }

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::geom_SkinSurface::BuildSurface()
{
  // Working variables.
  const int n = (int) (m_curves[0]->GetPoles().size() - 1);

  // Collect poles.
  std::vector< std::vector<t_xyz> > final_poles;
  for ( int i = 0; i <= n; ++i )
    final_poles.push_back( IsoU_Curves[i]->GetPoles() );
  
  // Construct B-surface.
  m_surface = new t_bsurf(final_poles,
                          m_curves[0]->GetKnots(),
                          m_V,
                          m_curves[0]->GetDegree(), m_iDeg_V);

  return true;
}
