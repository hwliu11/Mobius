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
: core_OPERATOR(progress, plotter)
{
  m_errCode = ErrCode_NotInitialized;
}

//-----------------------------------------------------------------------------

mobius::geom_SkinSurface::geom_SkinSurface(const std::vector< ptr<bcurve> >& curves,
                                           const int                         deg_V,
                                           const bool                        unifyCurves,
                                           core_ProgressEntry                progress,
                                           core_PlotterEntry                 plotter)
: core_OPERATOR(progress, plotter)
{
  this->Init(curves, deg_V, unifyCurves);
}

//-----------------------------------------------------------------------------

void mobius::geom_SkinSurface::Init(const std::vector< ptr<bcurve> >& curves,
                                    const int                         deg_V,
                                    const bool                        unifyCurves)
{
  m_curves  = curves;
  m_iDeg_V  = deg_V;
  m_bUnify  = unifyCurves;
  m_errCode = ErrCode_NotDone;
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

  // Check compatibility of curves.
  bool areCompatible = true;
  int ref_degree = 0;
  std::vector<double> ref_U;
  for ( size_t c = 0; c < m_curves.size(); ++c )
  {
    const ptr<bcurve>& crv = m_curves[c];
    if ( crv.IsNull() )
    {
      m_errCode = ErrCode_NullCurvePassed;
      return false;
    }

    if ( c == 0 )
    {
      ref_degree = crv->Degree();
      ref_U      = crv->Knots();
    }
    else
    {
      if ( crv->Degree() != ref_degree )
      {
        areCompatible = false;

        if ( !m_bUnify )
        {
          m_errCode = ErrCode_NotCompatibleCurves_Degree;
          return false;
        }
        else
          break;
      }

      const std::vector<double>& curr_U = crv->Knots();
      //
      if ( curr_U != ref_U )
      {
        areCompatible = false;

        if ( !m_bUnify )
        {
          m_errCode = ErrCode_NotCompatibleCurves_Knots;
          return false;
        }
        else
          break;
      }
    }
  }

  if ( !areCompatible && m_bUnify )
  {
    // Normalize and collect knot vectors.
    std::vector< std::vector<double> > U_all;
    for ( size_t c = 0; c < m_curves.size(); ++c )
    {
      // Normalize.
      m_curves[c]->ReparameterizeLinear(0.0, 1.0);

      // Get knots.
      std::vector<double> U = m_curves[c]->Knots();
      U_all.push_back(U);

#if defined COUT_DEBUG
      // Dump knots
      std::cout << "Curve " << (c + 1) << ": ";
      for ( size_t j = 0; j < U.size(); ++j )
      {
        std::cout << U[j] << "\t";
      }
      std::cout << std::endl;
#endif
    }

    // Compute extension.
    bspl_UnifyKnots Unify;
    std::vector< std::vector<double> > X = Unify(U_all);

    // Unify knots
    for ( size_t c = 0; c < m_curves.size(); ++c )
    {
      m_curves[c]->RefineKnots(X[c]);

#if defined COUT_DEBUG
      const std::vector<double>& U = m_curves[c]->Knots();

      // Dump knots
      std::cout << "Curve [refined] " << (c + 1) << ": ";
      for ( size_t j = 0; j < U.size(); ++j )
      {
        std::cout << U[j] << "\t";
      }
      std::cout << std::endl;
#endif
    }
  }

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::geom_SkinSurface::BuildIsosU()
{
  // Working dimensions.
  const int n = (int) (m_curves[0]->Poles().size() - 1);
  const int K = (int) (m_curves.size() - 1);

  // Check if the V degree is suitable.
  if ( !bspl::Check(K, m_iDeg_V) )
  {
    m_errCode = ErrCode_BadVDegree;
    return false;
  }

  /* -----------------------------------------------------------
   *  Choose parameters for u-isos by averaged centripetal rule
   * ----------------------------------------------------------- */

  // Prepare rectangular collection of control points.
  std::vector< std::vector<xyz> > Q;
  for ( int i = 0; i <= n; ++i )
  {
    std::vector<xyz> poles;
    for ( int c = 0; c < (int) m_curves.size(); ++c )
    {
      const ptr<bcurve>& crv = m_curves[c];
      poles.push_back( crv->Poles()[i] );
    }
    Q.push_back(poles);
  }

  // Allocate arrays for reper parameters
  double* params_V = m_alloc.Allocate(K + 1, true);
  if ( bspl_ParamsCentripetal::Calculate_V(Q, params_V) != bspl_ParamsCentripetal::ErrCode_NoError )
  {
    m_errCode = ErrCode_CannotSelectParameters;
    return false;
  }

  /* ---------------------------
   *  Choose knots by averaging
   * --------------------------- */

  const int m = bspl::M(K, m_iDeg_V);
  m_pV = m_alloc.Allocate(m + 1, true);

  if ( bspl_KnotsAverage::Calculate(params_V, K, m_iDeg_V, m,
                                    bspl_KnotsAverage::Recognize(false, false, false, false),
                                    m_pV) != bspl_KnotsAverage::ErrCode_NoError )
  {
    m_errCode = ErrCode_CannotSelectKnots;
    return false;
  }

  /* -------------------------------------------------
   *  Interpolate poles of curves in iso-U directions
   * ------------------------------------------------- */

  IsoU_Curves.clear();
  for ( int i = 0; i <= n; ++i )
  {
    // Populate reper points (poles of curves) for fixed U values.
    std::vector<xyz> iso_U_poles;
    for ( int k = 0; k <= K; ++k )
      iso_U_poles.push_back(Q[i][k]);

    // Interpolate over these poles.
    ptr<bcurve> iso_U;
    if ( !geom_InterpolateCurve::Interp(iso_U_poles, K, m_iDeg_V, params_V, m_pV, m,
                                        false,
                                        false,
                                        false,
                                        false,
                                        xyz(),
                                        xyz(),
                                        xyz(),
                                        xyz(),
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
  const int n = (int) (m_curves[0]->Poles().size() - 1);
  const int K = (int) (m_curves.size() - 1);
  const int m = bspl::M(K, m_iDeg_V);

  // Collect poles.
  std::vector< std::vector<xyz> > final_poles;
  //
  for ( int i = 0; i <= n; ++i )
  {
    final_poles.push_back( IsoU_Curves[i]->Poles() );
  }

  std::vector<double> U_knots = m_curves[0]->Knots();
  double *U = m_alloc.Allocate(U_knots.size(), true);
  for ( size_t i = 0; i < U_knots.size(); ++i )
    U[i] = U_knots[i];

  m_surface = new bsurf(final_poles,
                        U, m_pV,
                        (int) U_knots.size(), m + 1,
                        m_curves[0]->Degree(), m_iDeg_V);

  return true;
}
