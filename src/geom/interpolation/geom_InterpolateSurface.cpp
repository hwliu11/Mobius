//-----------------------------------------------------------------------------
// Created on: 23 December 2014
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
#include <mobius/geom_InterpolateSurface.h>

// Geometry includes
#include <mobius/geom_InterpolateCurve.h>

// Core includes
#include <mobius/core_HeapAlloc.h>
#include <mobius/core_SolveLinearSystem.h>

// BSpl includes
#include <mobius/bspl_KnotsAverage.h>
#include <mobius/bspl_N.h>
#include <mobius/bspl_ParamsCentripetal.h>
#include <mobius/bspl_ParamsChordLength.h>
#include <mobius/bspl_ParamsUniform.h>

#if defined Qr_DEBUG
  #include <mobius/core_FileDumper.h>
  #define dump_filename "../../test/dumping/log.log"
#endif

//-----------------------------------------------------------------------------

mobius::geom_InterpolateSurface::geom_InterpolateSurface(core_ProgressEntry progress,
                                                         core_PlotterEntry  plotter)
: core_OPERATOR(progress, plotter)
{
  m_errCode = ErrCode_NotInitialized;
}

//-----------------------------------------------------------------------------

mobius::geom_InterpolateSurface::geom_InterpolateSurface(const std::vector< std::vector<t_xyz> >& points,
                                                         const int                                deg_U,
                                                         const int                                deg_V,
                                                         const bspl_ParamsSelection               paramsType,
                                                         const bspl_KnotsSelection                knotsType,
                                                         core_ProgressEntry                       progress,
                                                         core_PlotterEntry                        plotter)
: core_OPERATOR(progress, plotter)
{
  this->Init(points, deg_U, deg_V, paramsType, knotsType);
}

//-----------------------------------------------------------------------------

mobius::geom_InterpolateSurface::geom_InterpolateSurface(const std::vector< std::vector<t_xyz> >& points,
                                                         const int                                deg_U,
                                                         const int                                deg_V,
                                                         const t_ptr<geom_VectorField>&           derivs_isoV_start_D1,
                                                         const t_ptr<geom_VectorField>&           derivs_isoV_end_D1,
                                                         const bspl_ParamsSelection               paramsType,
                                                         const bspl_KnotsSelection                knotsType,
                                                         core_ProgressEntry                       progress,
                                                         core_PlotterEntry                        plotter)
: core_OPERATOR(progress, plotter)
{
  this->Init(points, deg_U, deg_V,
             derivs_isoV_start_D1, derivs_isoV_end_D1,
             paramsType, knotsType);
}

//-----------------------------------------------------------------------------

mobius::geom_InterpolateSurface::geom_InterpolateSurface(const std::vector< std::vector<t_xyz> >& points,
                                                         const int                                deg_U,
                                                         const int                                deg_V,
                                                         const t_ptr<geom_VectorField>&           derivs_isoV_start_D1,
                                                         const t_ptr<geom_VectorField>&           derivs_isoV_end_D1,
                                                         const t_ptr<geom_VectorField>&           derivs_isoV_start_D2,
                                                         const t_ptr<geom_VectorField>&           derivs_isoV_end_D2,
                                                         const bspl_ParamsSelection               paramsType,
                                                         const bspl_KnotsSelection                knotsType,
                                                         core_ProgressEntry                       progress,
                                                         core_PlotterEntry                        plotter)
: core_OPERATOR(progress, plotter)
{
  this->Init(points, deg_U, deg_V,
             derivs_isoV_start_D1, derivs_isoV_end_D1,
             derivs_isoV_start_D2, derivs_isoV_end_D2,
             paramsType, knotsType);
}

//-----------------------------------------------------------------------------

void mobius::geom_InterpolateSurface::Init(const std::vector< std::vector<t_xyz> >& points,
                                           const int                                deg_U,
                                           const int                                deg_V,
                                           const bspl_ParamsSelection               paramsType,
                                           const bspl_KnotsSelection                knotsType)
{
  m_points     = points;
  m_iDeg_U     = deg_U;
  m_iDeg_V     = deg_V;
  m_paramsType = paramsType;
  m_knotsType  = knotsType;
  m_errCode    = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

void mobius::geom_InterpolateSurface::Init(const std::vector< std::vector<t_xyz> >& points,
                                           const int                                deg_U,
                                           const int                                deg_V,
                                           const t_ptr<geom_VectorField>&           derivs_isoV_start_D1,
                                           const t_ptr<geom_VectorField>&           derivs_isoV_end_D1,
                                           const bspl_ParamsSelection               paramsType,
                                           const bspl_KnotsSelection                knotsType)
{
  m_points               = points;
  m_iDeg_U               = deg_U;
  m_iDeg_V               = deg_V;
  m_derivs_isoV_start_D1 = derivs_isoV_start_D1;
  m_derivs_isoV_end_D1   = derivs_isoV_end_D1;
  m_paramsType           = paramsType;
  m_knotsType            = knotsType;
  m_errCode              = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

void mobius::geom_InterpolateSurface::Init(const std::vector< std::vector<t_xyz> >& points,
                                           const int                                deg_U,
                                           const int                                deg_V,
                                           const t_ptr<geom_VectorField>&           derivs_isoV_start_D1,
                                           const t_ptr<geom_VectorField>&           derivs_isoV_end_D1,
                                           const t_ptr<geom_VectorField>&           derivs_isoV_start_D2,
                                           const t_ptr<geom_VectorField>&           derivs_isoV_end_D2,
                                           const bspl_ParamsSelection               paramsType,
                                           const bspl_KnotsSelection                knotsType)
{
  m_points               = points;
  m_iDeg_U               = deg_U;
  m_iDeg_V               = deg_V;
  m_derivs_isoV_start_D1 = derivs_isoV_start_D1;
  m_derivs_isoV_end_D1   = derivs_isoV_end_D1;
  m_derivs_isoV_start_D2 = derivs_isoV_start_D2;
  m_derivs_isoV_end_D2   = derivs_isoV_end_D2;
  m_paramsType           = paramsType;
  m_knotsType            = knotsType;
  m_errCode              = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

bool mobius::geom_InterpolateSurface::Perform()
{
  m_errCode = ErrCode_NoError;

  // Heap allocator
  core_HeapAlloc<double> Alloc;

  /* ---------------------------------------
   *  Choose reper (interpolant) parameters
   * --------------------------------------- */

  if ( m_points.size() < 2 )
  {
    m_errCode = ErrCode_PoorInitialGrid;
    return false;
  }

  // Check if the passed grid is rectangular
  size_t record_size = m_points[0].size();
  if ( record_size < 2 )
  {
    m_errCode = ErrCode_PoorInitialGrid;
    return false;
  }
  for ( size_t record_idx = 1; record_idx < m_points.size(); ++record_idx )
  {
    if ( m_points[record_idx].size() != record_size )
    {
      m_errCode = ErrCode_NonRectangularGrid;
      return false;
    }
  }

  // Dimensions of reper grid
  const int n = (int) (m_points.size() - 1);
  const int m = (int) (m_points.at(0).size() - 1);

  // Get number of D1 interpolation constraints (if any)
  const int nDerivs_isoV_start_D1 = m_derivs_isoV_start_D1.IsNull() ? 0 : (int) m_derivs_isoV_start_D1->GetCloud()->GetNumberOfPoints();
  const int nDerivs_isoV_end_D1   = m_derivs_isoV_end_D1.IsNull() ? 0 : (int) m_derivs_isoV_end_D1->GetCloud()->GetNumberOfPoints();

  // Get number of D2 interpolation constraints (if any)
  const int nDerivs_isoV_start_D2 = m_derivs_isoV_start_D2.IsNull() ? 0 : (int) m_derivs_isoV_start_D2->GetCloud()->GetNumberOfPoints();
  const int nDerivs_isoV_end_D2   = m_derivs_isoV_end_D2.IsNull() ? 0 : (int) m_derivs_isoV_end_D2->GetCloud()->GetNumberOfPoints();

  // Check if D1 constraints are active
  const bool hasDerivs_isoV_start_D1 = (nDerivs_isoV_start_D1 > 0);
  const bool hasDerivs_isoV_end_D1 = (nDerivs_isoV_end_D1 > 0);

  // Check if D2 constraints are active
  const bool hasDerivs_isoV_start_D2 = (nDerivs_isoV_start_D2 > 0);
  const bool hasDerivs_isoV_end_D2 = (nDerivs_isoV_end_D2 > 0);

  // Check if interpolation constraints are set
  if ( hasDerivs_isoV_start_D1 && nDerivs_isoV_start_D1 != (m + 1) )
  {
    m_errCode = ErrCode_NotEnoughVDerivs_Start_D1;
    return false;
  }
  if ( hasDerivs_isoV_end_D1 && nDerivs_isoV_end_D1 != (m + 1) )
  {
    m_errCode = ErrCode_NotEnoughVDerivs_End_D1;
    return false;
  }
  if ( hasDerivs_isoV_start_D2 && nDerivs_isoV_start_D2 != (m + 1) )
  {
    m_errCode = ErrCode_NotEnoughVDerivs_Start_D2;
    return false;
  }
  if ( hasDerivs_isoV_end_D2 && nDerivs_isoV_end_D2 != (m + 1) )
  {
    m_errCode = ErrCode_NotEnoughVDerivs_End_D2;
    return false;
  }

  // Check if there are enough reper points
  if ( !bspl::Check(n, m_iDeg_U) || !bspl::Check(m, m_iDeg_V) )
    throw std::runtime_error("Poor collection of data points for the given degree(s)");

  // Allocate arrays for reper parameters
  double* params_U = Alloc.Allocate(n + 1, true);
  double* params_V = Alloc.Allocate(m + 1, true);

  // Prepare parameterization
  if ( m_paramsType == ParamsSelection_Uniform )
  {
    if ( bspl_ParamsUniform::Calculate(n, params_U) != bspl_ParamsUniform::ErrCode_NoError ||
         bspl_ParamsUniform::Calculate(m, params_V) != bspl_ParamsUniform::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return false;
    }
  }
  else if ( m_paramsType == ParamsSelection_ChordLength )
  {
    if ( bspl_ParamsChordLength::Calculate(m_points, params_U, params_V) != bspl_ParamsChordLength::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return false;
    }
  }
  else if ( m_paramsType == ParamsSelection_Centripetal )
  {
    if ( bspl_ParamsCentripetal::Calculate(m_points, params_U, params_V) != bspl_ParamsCentripetal::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return false;
    }
  }
  else
    throw std::runtime_error("NYI parameterization type");

  // TODO: introduce other parameterization techniques

  /* --------------------------
   *  Choose interpolant knots
   * -------------------------- */

  double *U = nullptr, *V = nullptr;

  const int r = bspl::M(n, m_iDeg_U)
              + (hasDerivs_isoV_start_D1 ? 1 : 0) + (hasDerivs_isoV_end_D1 ? 1 : 0)
              + (hasDerivs_isoV_start_D2 ? 1 : 0) + (hasDerivs_isoV_end_D2 ? 1 : 0);

  const int s = bspl::M(m, m_iDeg_V);

  if ( m_knotsType == KnotsSelection_Average )
  {
    U = Alloc.Allocate(r + 1, true);
    V = Alloc.Allocate(s + 1, true);

    if ( bspl_KnotsAverage::Calculate(params_U, n, m_iDeg_U, r,
                                      bspl_KnotsAverage::Recognize(hasDerivs_isoV_start_D1,
                                                                   hasDerivs_isoV_end_D1,
                                                                   hasDerivs_isoV_start_D2,
                                                                   hasDerivs_isoV_end_D2),
                                      U) != bspl_KnotsAverage::ErrCode_NoError
         ||
         bspl_KnotsAverage::Calculate(params_V, m, m_iDeg_V, s,
                                      bspl_KnotsAverage::Recognize(false, false, false, false),
                                      V) != bspl_KnotsAverage::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectKnots;
      return false;
    }
  }
  else
    throw std::runtime_error("NYI knots selection type");

  // TODO: introduce other parameterization techniques

  /* ---------------------------------------------
   *  Find R_{i,j} by interpolation of V-isolines
   * --------------------------------------------- */

  IsoV_Curves.clear();
  for ( int l = 0; l <= m; ++l )
  {
    // Populate reper points for fixed V values
    std::vector<t_xyz> iso_V_poles;
    for ( int k = 0; k <= n; ++k )
      iso_V_poles.push_back(m_points[k][l]);

    t_xyz D1_start = hasDerivs_isoV_start_D1 ? m_derivs_isoV_start_D1->GetVector(l) : t_xyz();
    t_xyz D1_end   = hasDerivs_isoV_end_D1   ? m_derivs_isoV_end_D1->GetVector(l)   : t_xyz();
    t_xyz D2_start = hasDerivs_isoV_start_D2 ? m_derivs_isoV_start_D2->GetVector(l) : t_xyz();
    t_xyz D2_end   = hasDerivs_isoV_end_D2   ? m_derivs_isoV_end_D2->GetVector(l)   : t_xyz();

    // Interpolate over these cross-sections only
    t_ptr<t_bcurve> iso_V;
    if ( !geom_InterpolateCurve::Interp(iso_V_poles, n, m_iDeg_U, params_U, U, r,
                                        hasDerivs_isoV_start_D1,
                                        hasDerivs_isoV_end_D1,
                                        hasDerivs_isoV_start_D2,
                                        hasDerivs_isoV_end_D2,
                                        D1_start,
                                        D1_end,
                                        D2_start,
                                        D2_end,
                                        iso_V) )
    {
      m_errCode = ErrCode_CannotInterpolateIsoV;
      return false;
    }
    IsoV_Curves.push_back(iso_V);
  }

  /* ------------------------------------------
   *  Find P_{i,j} by interpolation of R_{i,j}
   * ------------------------------------------ */

  // Poles of interpolant
  std::vector< std::vector<t_xyz> > final_poles;

  // Interpolate by new repers
  ReperU_Curves.clear();
  const int corrected_n = n
                        + (hasDerivs_isoV_start_D1 ? 1 : 0)
                        + (hasDerivs_isoV_end_D1 ? 1 : 0)
                        + (hasDerivs_isoV_start_D2 ? 1 : 0)
                        + (hasDerivs_isoV_end_D2 ? 1 : 0);

  for ( int k = 0; k <= corrected_n; ++k )
  {
    // Populate reper points: we use the control points of V-isocurves
    // as reper points now
    std::vector<t_xyz> R_poles;
    for ( int l = 0; l <= m; ++l )
      R_poles.push_back(IsoV_Curves[l]->GetPoles()[k]);

    // Interpolate again
    t_ptr<t_bcurve> R_interp;
    if ( !geom_InterpolateCurve::Interp(R_poles, m, m_iDeg_V, params_V, V, s,
                                        false, false, false, false,
                                        t_xyz(), t_xyz(), t_xyz(), t_xyz(),
                                        R_interp) )
    {
      m_errCode = ErrCode_CannotInterpolateIsoU;
      return false;
    }
    ReperU_Curves.push_back(R_interp);

    // Poles in V column of the resulting grid
    std::vector<t_xyz> V_column_poles;
    for ( int p = 0; p <= m; ++p )
      V_column_poles.push_back(R_interp->GetPoles()[p]);

    // Save to resulting grid
    final_poles.push_back(V_column_poles);
  }

  /* -----------------------
   *  Construct interpolant
   * ----------------------- */

  m_surface = new t_bsurf(final_poles,
                          U, V,
                          r + 1, s + 1,
                          m_iDeg_U, m_iDeg_V);

  return true;
}
