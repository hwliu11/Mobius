//-----------------------------------------------------------------------------
// Created on: 09 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
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

// Core includes
#include <mobius/core_HeapAlloc.h>

//! Default constructor.
mobius::geom_SkinSurface::geom_SkinSurface()
{
  m_errCode = ErrCode_NotInitialized;
}

//! Complete constructor. Initializes the interpolation tool with a set
//! of curves to skin the surface through.
//! \param[in] curves      curves to skin the surface through.
//! \param[in] deg_V       degree in V curvilinear direction.
//! \param[in] unifyCurves indicates whether to unify curves before skinning.
mobius::geom_SkinSurface::geom_SkinSurface(const std::vector< Ptr<bcurve> >& curves,
                                           const int                         deg_V,
                                           const bool                        unifyCurves)
{
  this->Init(curves, deg_V, unifyCurves);
}

//! Initializes skinning tool.
//! \param[in] curves      curves to skin the surface through.
//! \param[in] deg_V       degree in V curvilinear direction.
//! \param[in] unifyCurves indicates whether to unify curves before skinning.
void mobius::geom_SkinSurface::Init(const std::vector< Ptr<bcurve> >& curves,
                                    const int                         deg_V,
                                    const bool                        unifyCurves)
{
  m_curves  = curves;
  m_iDeg_V  = deg_V;
  m_bUnify  = unifyCurves;
  m_errCode = ErrCode_NotDone;
}

//! Performs skinning.
//! \return true in case of success, false -- otherwise.
void mobius::geom_SkinSurface::Perform()
{
  m_errCode = ErrCode_NoError;

  // Heap allocator
  core_HeapAlloc<double> Alloc;

  /* --------------------
   *  Preliminary checks
   * -------------------- */

  if ( m_curves.size() < 2 )
  {
    m_errCode = ErrCode_NotEnoughCurves;
    return;
  }

  // Check compatibility of curves
  bool areCompatible = true;
  int ref_degree = 0;
  std::vector<double> ref_U;
  for ( size_t c = 0; c < m_curves.size(); ++c )
  {
    const Ptr<bcurve>& crv = m_curves[c];
    if ( crv.IsNull() )
    {
      m_errCode = ErrCode_NullCurvePassed;
      return;
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
          return;
        }
        else
          break;
      }
      if ( crv->Knots() != ref_U && !m_bUnify )
      {
        areCompatible = false;

        if ( !m_bUnify )
        {
          m_errCode = ErrCode_NotCompatibleCurves_Knots;
          return;
        }
        else
          break;
      }
    }
  }

  if ( !areCompatible && m_bUnify )
  {
    // Normalize and collect knot vectors
    std::vector< std::vector<double> > U_all;
    for ( size_t c = 0; c < m_curves.size(); ++c )
    {
      // Normalize
      m_curves[c]->ReparameterizeLinear(0.0, 1.0);

      // Get knots
      std::vector<double> U = m_curves[c]->Knots();
      U_all.push_back(U);

      // Dump knots
      std::cout << "Curve " << (c + 1) << ": ";
      for ( size_t j = 0; j < U.size(); ++j )
      {
        std::cout << U[j] << "\t";
      }
      std::cout << std::endl;
    }

    // Compute extension
    bspl_UnifyKnots Unify;
    std::vector< std::vector<double> > X = Unify(U_all);

    // Unify knots
    for ( size_t c = 0; c < m_curves.size(); ++c )
    {
      m_curves[c]->RefineKnots(X[c]);
      const std::vector<double>& U = m_curves[c]->Knots();

      // Dump knots
      std::cout << "Curve [refined] " << (c + 1) << ": ";
      for ( size_t j = 0; j < U.size(); ++j )
      {
        std::cout << U[j] << "\t";
      }
      std::cout << std::endl;
    }
  }

  // Working dimensions
  const int n = (int) (m_curves[0]->Poles().size() - 1);
  const int K = (int) (m_curves.size() - 1);

  // Check if the V degree is suitable
  if ( !bspl::Check(K, m_iDeg_V) )
  {
    m_errCode = ErrCode_BadVDegree;
    return;
  }

  /* ------------------------------------------------------------
   *  Choose parameters for u-isos by averaged chord-length rule
   * ------------------------------------------------------------ */

  // Prepare rectangular collection of control points
  std::vector< std::vector<xyz> > Q;
  for ( int i = 0; i <= n; ++i )
  {
    std::vector<xyz> poles;
    for ( int c = 0; c < (int) m_curves.size(); ++c )
    {
      const Ptr<bcurve>& crv = m_curves[c];
      poles.push_back( crv->Poles()[i] );
    }
    Q.push_back(poles);
  }

  // Allocate arrays for reper parameters
  double* params_V = Alloc.Allocate(K + 1, true);
  if ( bspl_ParamsCentripetal::Calculate_V(Q, params_V) != bspl_ParamsChordLength::ErrCode_NoError )
  {
    m_errCode = ErrCode_CannotSelectParameters;
    return;
  }

  /* ---------------------------
   *  Choose knots by averaging
   * --------------------------- */

  const int m = bspl::M(K, m_iDeg_V);
  double   *V = Alloc.Allocate(m + 1, true);

  if ( bspl_KnotsAverage::Calculate(params_V, K, m_iDeg_V, m,
                                    bspl_KnotsAverage::Recognize(false, false, false, false),
                                    V) != bspl_KnotsAverage::ErrCode_NoError )
  {
    m_errCode = ErrCode_CannotSelectKnots;
    return;
  }

  /* -------------------------------------------------
   *  Interpolate poles of curves in iso-U directions
   * ------------------------------------------------- */

  std::vector< std::vector<xyz> > final_poles;
  IsoU_Curves.clear();
  for ( int i = 0; i <= n; ++i )
  {
    // Populate reper points (poles of curves) for fixed U values
    std::vector<xyz> iso_U_poles;
    for ( int k = 0; k <= K; ++k )
      iso_U_poles.push_back(Q[i][k]);

    // Interpolate over these poles
    Ptr<bcurve> iso_U;
    if ( !geom_InterpolateCurve::Interp(iso_U_poles, K, m_iDeg_V, params_V, V, m,
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
      return;
    }
    IsoU_Curves.push_back(iso_U);
    final_poles.push_back( iso_U->Poles() );
  }

  /* -----------------------
   *  Construct interpolant
   * ----------------------- */

  std::vector<double> U_knots = m_curves[0]->Knots();
  double *U = Alloc.Allocate(U_knots.size(), true);
  for ( size_t i = 0; i < U_knots.size(); ++i )
    U[i] = U_knots[i];

  m_surface = new bsurf(final_poles,
                        U, V,
                        (int) U_knots.size(), m + 1,
                        m_curves[0]->Degree(), m_iDeg_V);
}
