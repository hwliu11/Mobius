//-----------------------------------------------------------------------------
// Created on: 23 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
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

//! Default constructor.
mobius::geom_InterpolateSurface::geom_InterpolateSurface()
{
  m_errCode = ErrCode_NotInitialized;
}

//! Complete constructor. Initializes the interpolation tool with a set
//! of reper points and other properties which are necessary in order to
//! reduce the generally sophisticated interpolation problem to a
//! trivial linear algebraic system.
//!
//! \param points [in] data points to interpolate.
//! \param deg_U [in] U-degree of the interpolant surface.
//! \param deg_V [in] V-degree of the interpolant surface.
//! \param paramsType [in] strategy for choosing interpolant parameters for
//!        the data points.
//! \param knotsType [in] strategy for choosing knot vector for interpolant
//!        B-splines.
mobius::geom_InterpolateSurface::geom_InterpolateSurface(const std::vector< std::vector<xyz> >& points,
                                                         const int                              deg_U,
                                                         const int                              deg_V,
                                                         const bspl_ParamsSelection             paramsType,
                                                         const bspl_KnotsSelection              knotsType)
{
  this->Init(points, deg_U, deg_V, paramsType, knotsType);
}

//! Complete constructor. Initializes the interpolation tool with a set
//! of reper points and other properties which are necessary in order to
//! reduce the generally sophisticated interpolation problem to a
//! trivial linear algebraic system.
//! \param points [in] data points to interpolate.
//! \param deg_U [in] U-degree of the interpolant surface.
//! \param deg_V [in] V-degree of the interpolant surface.
//! \param derivs_isoV_start_D1 [in] leading V-iso derivatives D1.
//! \param derivs_isoV_end_D1 [in] trailing V-iso derivatives D1.
//! \param paramsType [in] strategy for choosing interpolant parameters for
//!        the data points.
//! \param knotsType [in] strategy for choosing knot vector for interpolant
//!        B-splines.
mobius::geom_InterpolateSurface::geom_InterpolateSurface(const std::vector< std::vector<xyz> >& points,
                                                         const int                              deg_U,
                                                         const int                              deg_V,
                                                         const Ptr<geom_VectorField>&           derivs_isoV_start_D1,
                                                         const Ptr<geom_VectorField>&           derivs_isoV_end_D1,
                                                         const bspl_ParamsSelection             paramsType,
                                                         const bspl_KnotsSelection              knotsType)
{
  this->Init(points, deg_U, deg_V,
             derivs_isoV_start_D1, derivs_isoV_end_D1,
             paramsType, knotsType);
}

//! Complete constructor. Initializes the interpolation tool with a set
//! of reper points and other properties which are necessary in order to
//! reduce the generally sophisticated interpolation problem to a
//! trivial linear algebraic system.
//!
//! \param points [in] data points to interpolate.
//! \param deg_U [in] U-degree of the interpolant surface.
//! \param deg_V [in] V-degree of the interpolant surface.
//! \param derivs_isoV_start_D1 [in] leading V-iso derivatives D1.
//! \param derivs_isoV_end_D1 [in] trailing V-iso derivatives D1.
//! \param derivs_isoV_start_D2 [in] leading V-iso derivatives D2.
//! \param derivs_isoV_end_D2 [in] trailing V-iso derivatives D2.
//! \param paramsType [in] strategy for choosing interpolant parameters for
//!        the data points.
//! \param knotsType [in] strategy for choosing knot vector for interpolant
//!        B-splines.
mobius::geom_InterpolateSurface::geom_InterpolateSurface(const std::vector< std::vector<xyz> >& points,
                                                         const int                              deg_U,
                                                         const int                              deg_V,
                                                         const Ptr<geom_VectorField>&           derivs_isoV_start_D1,
                                                         const Ptr<geom_VectorField>&           derivs_isoV_end_D1,
                                                         const Ptr<geom_VectorField>&           derivs_isoV_start_D2,
                                                         const Ptr<geom_VectorField>&           derivs_isoV_end_D2,
                                                         const bspl_ParamsSelection             paramsType,
                                                         const bspl_KnotsSelection              knotsType)
{
  this->Init(points, deg_U, deg_V,
             derivs_isoV_start_D1, derivs_isoV_end_D1,
             derivs_isoV_start_D2, derivs_isoV_end_D2,
             paramsType, knotsType);
}

//! Initializes interpolation tool.
//! \param points [in] data points to interpolate.
//! \param deg_U [in] U-degree of the interpolant surface.
//! \param deg_V [in] V-degree of the interpolant surface.
//! \param paramsType [in] strategy for choosing interpolant parameters for
//!        the data points.
//! \param knotsType [in] strategy for choosing knot vector for interpolant
//!        B-splines.
void mobius::geom_InterpolateSurface::Init(const std::vector< std::vector<xyz> >& points,
                                           const int                              deg_U,
                                           const int                              deg_V,
                                           const bspl_ParamsSelection             paramsType,
                                           const bspl_KnotsSelection              knotsType)
{
  m_points     = points;
  m_iDeg_U     = deg_U;
  m_iDeg_V     = deg_V;
  m_paramsType = paramsType;
  m_knotsType  = knotsType;
  m_errCode    = ErrCode_NotDone;
}

//! Initializes interpolation tool.
//! \param points [in] data points to interpolate.
//! \param deg_U [in] U-degree of the interpolant surface.
//! \param deg_V [in] V-degree of the interpolant surface.
//! \param derivs_isoV_start_D1 [in] leading V-iso derivatives D1.
//! \param derivs_isoV_end_D1 [in] trailing V-iso derivatives D1.
//! \param paramsType [in] strategy for choosing interpolant parameters for
//!        the data points.
//! \param knotsType [in] strategy for choosing knot vector for interpolant
//!        B-splines.
void mobius::geom_InterpolateSurface::Init(const std::vector< std::vector<xyz> >& points,
                                           const int                              deg_U,
                                           const int                              deg_V,
                                           const Ptr<geom_VectorField>&           derivs_isoV_start_D1,
                                           const Ptr<geom_VectorField>&           derivs_isoV_end_D1,
                                           const bspl_ParamsSelection             paramsType,
                                           const bspl_KnotsSelection              knotsType)
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

//! Initializes interpolation tool.
//! \param points [in] data points to interpolate.
//! \param deg_U [in] U-degree of the interpolant surface.
//! \param deg_V [in] V-degree of the interpolant surface.
//! \param derivs_isoV_start_D1 [in] leading V-iso derivatives D1.
//! \param derivs_isoV_end_D1 [in] trailing V-iso derivatives D1.
//! \param derivs_isoV_start_D2 [in] leading V-iso derivatives D2.
//! \param derivs_isoV_end_D2 [in] trailing V-iso derivatives D2.
//! \param paramsType [in] strategy for choosing interpolant parameters for
//!        the data points.
//! \param knotsType [in] strategy for choosing knot vector for interpolant
//!        B-splines.
void mobius::geom_InterpolateSurface::Init(const std::vector< std::vector<xyz> >& points,
                                           const int                              deg_U,
                                           const int                              deg_V,
                                           const Ptr<geom_VectorField>&           derivs_isoV_start_D1,
                                           const Ptr<geom_VectorField>&           derivs_isoV_end_D1,
                                           const Ptr<geom_VectorField>&           derivs_isoV_start_D2,
                                           const Ptr<geom_VectorField>&           derivs_isoV_end_D2,
                                           const bspl_ParamsSelection             paramsType,
                                           const bspl_KnotsSelection              knotsType)
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

//! Performs interpolation.
//! \return true in case of success, false -- otherwise.
void mobius::geom_InterpolateSurface::Perform()
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
    return;
  }

  // Check if the passed grid is rectangular
  size_t record_size = m_points[0].size();
  if ( record_size < 2 )
  {
    m_errCode = ErrCode_PoorInitialGrid;
    return;
  }
  for ( size_t record_idx = 1; record_idx < m_points.size(); ++record_idx )
  {
    if ( m_points[record_idx].size() != record_size )
    {
      m_errCode = ErrCode_NonRectangularGrid;
      return;
    }
  }

  // Dimensions of reper grid
  const int n = (int) (m_points.size() - 1);
  const int m = (int) (m_points.at(0).size() - 1);

  // Get number of D1 interpolation constraints (if any)
  const int nDerivs_isoV_start_D1 = m_derivs_isoV_start_D1.IsNull() ? 0 : (int) m_derivs_isoV_start_D1->Cloud()->NumberOfPoints();
  const int nDerivs_isoV_end_D1   = m_derivs_isoV_end_D1.IsNull() ? 0 : (int) m_derivs_isoV_end_D1->Cloud()->NumberOfPoints();

  // Get number of D2 interpolation constraints (if any)
  const int nDerivs_isoV_start_D2 = m_derivs_isoV_start_D2.IsNull() ? 0 : (int) m_derivs_isoV_start_D2->Cloud()->NumberOfPoints();
  const int nDerivs_isoV_end_D2   = m_derivs_isoV_end_D2.IsNull() ? 0 : (int) m_derivs_isoV_end_D2->Cloud()->NumberOfPoints();

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
    return;
  }
  if ( hasDerivs_isoV_end_D1 && nDerivs_isoV_end_D1 != (m + 1) )
  {
    m_errCode = ErrCode_NotEnoughVDerivs_End_D1;
    return;
  }
  if ( hasDerivs_isoV_start_D2 && nDerivs_isoV_start_D2 != (m + 1) )
  {
    m_errCode = ErrCode_NotEnoughVDerivs_Start_D2;
    return;
  }
  if ( hasDerivs_isoV_end_D2 && nDerivs_isoV_end_D2 != (m + 1) )
  {
    m_errCode = ErrCode_NotEnoughVDerivs_End_D2;
    return;
  }

  // Check if there are enough reper points
  if ( !bspl::Check(n, m_iDeg_U) || !bspl::Check(m, m_iDeg_V) )
    throw std::exception("Poor collection of data points for the given degree(s)");

  // Allocate arrays for reper parameters
  double* params_U = Alloc.Allocate(n + 1, true);
  double* params_V = Alloc.Allocate(m + 1, true);

  if ( m_paramsType == ParamsSelection_Uniform )
  {
    if ( bspl_ParamsUniform::Calculate(n, params_U) != bspl_ParamsUniform::ErrCode_NoError ||
         bspl_ParamsUniform::Calculate(m, params_V) != bspl_ParamsUniform::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return;
    }
  }
  else if ( m_paramsType == ParamsSelection_ChordLength )
  {
    if ( bspl_ParamsChordLength::Calculate(m_points, params_U, params_V) != bspl_ParamsChordLength::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return;
    }
  }
  else if ( m_paramsType == ParamsSelection_Centripetal )
  {
    if ( bspl_ParamsCentripetal::Calculate(m_points, params_U, params_V) != bspl_ParamsCentripetal::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return;
    }
  }
  else
    throw std::exception("NYI parameterization type");

  // TODO: introduce other parameterization techniques

  /* --------------------------
   *  Choose interpolant knots
   * -------------------------- */

  double *U = NULL, *V = NULL;

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
      return;
    }
  }
  else
    throw std::exception("NYI knots selection type");

  // TODO: introduce other parameterization techniques

  /* ---------------------------------------------
   *  Find R_{i,j} by interpolation of V-isolines
   * --------------------------------------------- */

  IsoV_Curves.clear();
  for ( int l = 0; l <= m; ++l )
  {
    // Populate reper points for fixed V values
    std::vector<xyz> iso_V_poles;
    for ( int k = 0; k <= n; ++k )
      iso_V_poles.push_back(m_points[k][l]);

    xyz D1_start = hasDerivs_isoV_start_D1 ? m_derivs_isoV_start_D1->Vector(l) : xyz();
    xyz D1_end   = hasDerivs_isoV_end_D1   ? m_derivs_isoV_end_D1->Vector(l)   : xyz();
    xyz D2_start = hasDerivs_isoV_start_D2 ? m_derivs_isoV_start_D2->Vector(l) : xyz();
    xyz D2_end   = hasDerivs_isoV_end_D2   ? m_derivs_isoV_end_D2->Vector(l)   : xyz();

    // Interpolate over these cross-sections only
    Ptr<bcurve> iso_V;
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
      return;
    }
    IsoV_Curves.push_back(iso_V);
  }

  /* ------------------------------------------
   *  Find P_{i,j} by interpolation of R_{i,j}
   * ------------------------------------------ */

  // Poles of interpolant
  std::vector< std::vector<xyz> > final_poles;

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
    std::vector<xyz> R_poles;
    for ( int l = 0; l <= m; ++l )
      R_poles.push_back(IsoV_Curves[l]->Poles()[k]);

    // Interpolate again
    Ptr<bcurve> R_interp;
    if ( !geom_InterpolateCurve::Interp(R_poles, m, m_iDeg_V, params_V, V, s,
                                        false, false, false, false,
                                        xyz(), xyz(), xyz(), xyz(),
                                        R_interp) )
    {
      m_errCode = ErrCode_CannotInterpolateIsoU;
      return;
    }
    ReperU_Curves.push_back(R_interp);

    // Poles in V column of the resulting grid
    std::vector<xyz> V_column_poles;
    for ( int p = 0; p <= m; ++p )
      V_column_poles.push_back(R_interp->Poles()[p]);

    // Save to resulting grid
    final_poles.push_back(V_column_poles);
  }

  /* -----------------------
   *  Construct interpolant
   * ----------------------- */

  m_surface = new bsurf(final_poles,
                        U, V,
                        r + 1, s + 1,
                        m_iDeg_U, m_iDeg_V);
}
