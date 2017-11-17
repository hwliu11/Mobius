//-----------------------------------------------------------------------------
// Created on: 26 October 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_InterpolateCurve.h>

// Core includes
#include <mobius/core_HeapAlloc.h>
#include <mobius/core_XYZ.h>

// BSpl includes
#include <mobius/bspl_KnotsAverage.h>
#include <mobius/bspl_N.h>
#include <mobius/bspl_ParamsCentripetal.h>
#include <mobius/bspl_ParamsChordLength.h>
#include <mobius/bspl_ParamsUniform.h>

// Core includes
#include <mobius/core_SolveLinearSystem.h>

#if defined Qr_DEBUG
  #include <mobius/core_FileDumper.h>
  #define dump_filename_N  "../../test/dumping/N_interp_log.log"
  #define dump_filename_Bx "../../test/dumping/N_interp_log_Bx.log"
  #define dump_filename_By "../../test/dumping/N_interp_log_By.log"
  #define dump_filename_Bz "../../test/dumping/N_interp_log_Bz.log"
#endif

//! Default constructor.
mobius::geom_InterpolateCurve::geom_InterpolateCurve()
{
  m_errCode = ErrCode_NotInitialized;
}

//! Complete constructor.
//! \param points     [in] data points to interpolate.
//! \param deg        [in] degree of B-spline functions to use for blending.
//! \param paramsType [in] strategy for choosing interpolant parameters in
//!                        the data points.
//! \param knotsType  [in] strategy for choosing knot vector for interpolant
//!                        B-splines.
mobius::geom_InterpolateCurve::geom_InterpolateCurve(const std::vector<xyz>&    points,
                                                     const int                  deg,
                                                     const bspl_ParamsSelection paramsType,
                                                     const bspl_KnotsSelection  knotsType)
{
  this->Init(points, deg, paramsType, knotsType);
}

//! Initializes interpolation tool.
//! \param points     [in] data points to interpolate.
//! \param deg        [in] degree of B-spline functions to use for blending.
//! \param paramsType [in] strategy for choosing interpolant parameters in
//!                        the data points.
//! \param knotsType  [in] strategy for choosing knot vector for interpolant
//!                        B-splines.
void mobius::geom_InterpolateCurve::Init(const std::vector<xyz>&    points,
                                         const int                  deg,
                                         const bspl_ParamsSelection paramsType,
                                         const bspl_KnotsSelection  knotsType)
{
  m_points     = points;
  m_iDeg       = deg;
  m_paramsType = paramsType;
  m_knotsType  = knotsType;
  m_errCode    = ErrCode_NotDone;
}

//! Initializes interpolation tool.
//! \param points     [in] data points to interpolate.
//! \param D0         [in] derivative D1 at the first point.
//! \param Dn         [in] derivative D1 at the last point.
//! \param deg        [in] degree of B-spline functions to use for blending.
//! \param paramsType [in] strategy for choosing interpolant parameters in
//!                        the data points.
//! \param knotsType  [in] strategy for choosing knot vector for interpolant
//!                        B-splines.
void mobius::geom_InterpolateCurve::Init(const std::vector<xyz>&    points,
                                         const xyz&                 D0,
                                         const xyz&                 Dn,
                                         const int                  deg,
                                         const bspl_ParamsSelection paramsType,
                                         const bspl_KnotsSelection  knotsType)
{
  m_points     = points;
  m_D0         = D0;
  m_Dn         = Dn;
  m_iDeg       = deg;
  m_paramsType = paramsType;
  m_knotsType  = knotsType;
  m_errCode    = ErrCode_NotDone;
}

//! Initializes interpolation tool.
//! \param points     [in] data points to interpolate.
//! \param D0         [in] derivative D1 at the first point.
//! \param Dn         [in] derivative D1 at the last point.
//! \param D20        [in] derivative D2 at the first point.
//! \param D2n        [in] derivative D2 at the last point.
//! \param deg        [in] degree of B-spline functions to use for blending.
//! \param paramsType [in] strategy for choosing interpolant parameters in
//!                        the data points.
//! \param knotsType  [in] strategy for choosing knot vector for interpolant
//!                        B-splines.
void mobius::geom_InterpolateCurve::Init(const std::vector<xyz>&    points,
                                         const xyz&                 D0,
                                         const xyz&                 Dn,
                                         const xyz&                 D20,
                                         const xyz&                 D2n,
                                         const int                  deg,
                                         const bspl_ParamsSelection paramsType,
                                         const bspl_KnotsSelection  knotsType)
{
  m_points     = points;
  m_D0         = D0;
  m_Dn         = Dn;
  m_D20        = D20;
  m_D2n        = D2n;
  m_iDeg       = deg;
  m_paramsType = paramsType;
  m_knotsType  = knotsType;
  m_errCode    = ErrCode_NotDone;
}

//! Performs interpolation.
//! \return true in case of success, false -- otherwise.
void mobius::geom_InterpolateCurve::Perform()
{
  m_errCode = ErrCode_NoError;

  // Heap allocator
  core_HeapAlloc<double> Alloc;

  /* -------------------------------
   *  Choose interpolant parameters
   * ------------------------------- */

  // There are as many parameters as many data points are passed for interpolation
  const int n     = this->last_index_poles();
  double*  params = Alloc.Allocate(n + 1, true);

  // Now parameterize
  if ( m_paramsType == ParamsSelection_Uniform )
  {
    if ( bspl_ParamsUniform::Calculate(n, params) != bspl_ParamsUniform::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return;
    }
  }
  else if ( m_paramsType == ParamsSelection_ChordLength )
  {
    if ( bspl_ParamsChordLength::Calculate(m_points, params) != bspl_ParamsChordLength::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return;
    }
  }
  else if ( m_paramsType == ParamsSelection_Centripetal )
  {
    if ( bspl_ParamsCentripetal::Calculate(m_points, params) != bspl_ParamsCentripetal::ErrCode_NoError )
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

  int m = this->last_index_knots();
  double* U = NULL;
  if ( m_knotsType == KnotsSelection_Average )
  {
    U = Alloc.Allocate(m + 1, true);

    if ( bspl_KnotsAverage::Calculate(params,
                                      n,
                                      m_iDeg,
                                      m,
                                      bspl_KnotsAverage::Recognize( this->has_start_deriv(),
                                                                    this->has_end_deriv(),
                                                                    this->has_start_deriv2(),
                                                                    this->has_end_deriv2() ),
                                      U) != bspl_KnotsAverage::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectKnots;
      return;
    }
  }
  else
    throw std::exception("NYI knots selection type");

  // TODO: introduce other parameterization techniques

  /* ------------------------------
   *  Perform actual interpolation
   * ------------------------------ */

  if ( !Interp(m_points, n, m_iDeg, params, U, m,
               this->has_start_deriv(),
               this->has_end_deriv(),
               this->has_start_deriv2(),
               this->has_end_deriv2(),
               m_D0, m_Dn, m_D20, m_D2n,
               m_curve) )
  {
    m_errCode = ErrCode_InterpolationFailed;
    return;
  }
}

//! Interpolation kernel.
//! \param points           [in]  reper points.
//! \param n                [in]  index of the last pole (0-based).
//! \param p                [in]  B-spline degree.
//! \param params           [in]  parameters of the reper points.
//! \param pU               [in]  knot vector.
//! \param m                [in]  index of the last knot (0-based).
//! \param has_start_deriv  [in]  indicates whether start derivative D1 is specified.
//! \param has_end_deriv    [in]  indicates whether end derivative D1 is specified.
//! \param has_start_deriv2 [in]  indicates whether start derivative D2 is specified.
//! \param has_end_deriv2   [in]  indicates whether end derivative D2 is specified.
//! \param D0               [in]  derivative D1 at the first point.
//! \param Dn               [in]  derivative D1 at the last point.
//! \param D20              [in]  derivative D2 at the first point.
//! \param D2n              [in]  derivative D2 at the last point.
//! \param crv              [out] result.
//! \return true in case of success, false -- otherwise.
bool mobius::geom_InterpolateCurve::Interp(const std::vector<xyz>& points,
                                           const int               n,
                                           const int               p,
                                           const double*           params,
                                           const double*           pU,
                                           const int               m,
                                           const bool              has_start_deriv,
                                           const bool              has_end_deriv,
                                           const bool              has_start_deriv2,
                                           const bool              has_end_deriv2,
                                           const xyz&              D0,
                                           const xyz&              Dn,
                                           const xyz&              D20,
                                           const xyz&              D2n,
                                           Ptr<bcurve>&            crv)
{
  if ( has_start_deriv2 && !has_start_deriv || has_end_deriv2 && !has_end_deriv )
    throw std::exception("Cannot handle D2 without D1"); // TODO: this limitation can be easily escaped

  std::vector<double> U;
  for ( int i = 0; i <= m; ++i )
    U.push_back(pU[i]);

  /* -----------------------------------
   *  Prepare matrix of B-spline values
   * ----------------------------------- */

  // TODO: use optimized evaluation algorithm for B-splines

  core_HeapAlloc<double> Alloc;

  const int dim = dimension(n, has_start_deriv, has_end_deriv, has_start_deriv2, has_end_deriv2); // Dimension of the problem in 3D
  double* N_values = Alloc.Allocate(dim*dim, true);

  bspl_N Eval;
  for ( int k = 0, kparams = 0; k < dim; ++k ) // Loop over the selected parameters
  {
    if ( (k == 1) && has_start_deriv )
    {
      N_values[dim*k + 0] = -1.0;
      N_values[dim*k + 1] =  1.0;
    }
    else if ( (k == 2) && has_start_deriv2 )
    {
      double coeff = p*(p - 1)/U[p+1];

      N_values[dim*k + 0] =  coeff/U[p+1];                            // P_0
      N_values[dim*k + 1] = -coeff*(U[p+1] + U[p+2])/(U[p+1]*U[p+2]); // P_1
      N_values[dim*k + 2] =  coeff/U[p+2];                            // P_2
    }
    else if ( (k == dim - 2) && has_end_deriv )
    {
      N_values[dim*k + dim - 2] = -1.0;
      N_values[dim*k + dim - 1] =  1.0;
    }
    else if ( (k == dim - 3) && has_end_deriv2 )
    {
      double coeff = p*(p - 1)/(1 - U[m-p-1]);

      N_values[dim*k + dim - 3] =  coeff/(1-U[m-p-2]);                                          // P_{n-2}
      N_values[dim*k + dim - 2] = -coeff*(2-U[m-p-1]-U[m-p-2])/((1 - U[m-p-1])*(1 - U[m-p-2])); // P_{n-1}
      N_values[dim*k + dim - 1] =  coeff/(1-U[m-p-1]);                                          // P_n
    }
    else
    {
      const double u_k = params[kparams];
      for ( int i = 0; i < dim; ++i ) // Loop over the indices of the unknown control points
      {
        const double N = Eval(u_k, U, p, i);
        N_values[dim*k + i] = N;
      }
      kparams++;
    }
  }

#if defined Qr_DEBUG
  core_FileDumper FileDumper(dump_filename_N);
  FileDumper.Dump(N_values, dim, dim, "N");
#endif

  /* -----------------------------------------
   *  Solve linear system for each coordinate
   * ----------------------------------------- */

  // Right-hand side
  double* b = Alloc.Allocate(dim, true);

  // Solution components
  double** pXYZ = new double*[3];
  for ( int c = 0; c < 3; ++c )
  {
    pXYZ[c] = new double[dim];
    memset(pXYZ[c], 0, dim*sizeof(double));
  }

  // Solver for linear system
  core_SolveLinearSystem SolveLinear;

  // TODO: separated solving is a bad idea (!!!)
  // Solve linear system for each target coordinate separately
  for ( int c = 0; c < 3; ++c ) // Loop over coordinates
  {
    for ( int k = 0, kparams = 0; k < dim; ++k )
    {
      if ( (k == 1) && has_start_deriv )
      {
        b[k] = D0.Coord(c) * U[p+1] / p;
      }
      else if ( (k == 2) && has_start_deriv2 )
      {
        b[k] = D20.Coord(c);
      }
      else if ( (k == dim - 2) && has_end_deriv )
      {
        b[k] = Dn.Coord(c) * (1 - U[m-p-1]) / p;
      }
      else if ( (k == dim - 3) && has_end_deriv2 )
      {
        b[k] = D2n.Coord(c);
      }
      else
      {
        b[k] = points.at(kparams).Coord(c);
        kparams++;
      }
    }

#if defined Qr_DEBUG
    std::string fn;
    if ( c == 0 )
      fn = dump_filename_Bx;
    else if ( c == 1 )
      fn = dump_filename_By;
    else
      fn = dump_filename_Bz;

    core_FileDumper FileDumper(fn);
    FileDumper.Dump(b, dim, 1, "B");
#endif

    SolveLinear(N_values, b, pXYZ[c], dim);
  }

  /* -------------------------------
   *  Construct interpolant B-curve
   * ------------------------------- */

  // Pack results
  std::vector<xyz> Poles;
  for ( int k = 0; k < dim; ++k )
  {
    xyz P(pXYZ[0][k], pXYZ[1][k], pXYZ[2][k]);
    Poles.push_back(P);
  }

  // Set result
  crv = new bcurve(Poles, U, p);

  /* ----------------
   *  Release memory
   * ---------------- */

  for ( int c = 0; c < 3; ++c )
    delete[] pXYZ[c];

  delete[] pXYZ;
  return true;
}

//! Returns index of the last pole. Notice that this index is zero-based,
//! so if we have K poles, it will return (K-1).
//! \return index of the last pole.
int mobius::geom_InterpolateCurve::last_index_poles() const
{
  int n = (int) (m_points.size() - 1);
  return n;
}

//! Returns index of the last knot. Notice that this index is zero-based,
//! so if we have K knots, it will return (K-1).
//! \return index of the last knot.
int mobius::geom_InterpolateCurve::last_index_knots() const
{
  int n = this->last_index_poles();
  int m = bspl::M(n, m_iDeg);

  // Each constraint on end derivative introduces one more control point and
  // one more knot so
  if ( this->has_start_deriv() )
    m += 1;
  if ( this->has_end_deriv() )
    m += 1;
  if ( this->has_start_deriv2() )
    m += 1;
  if ( this->has_end_deriv2() )
    m += 1;

  return m;
}

//! Returns true if start derivative D1 is specified, false -- otherwise.
//! \return true/false.
bool mobius::geom_InterpolateCurve::has_start_deriv() const
{
  return !m_D0.IsOrigin();
}

//! Returns true if end derivative D1 is specified, false -- otherwise.
//! \return true/false.
bool mobius::geom_InterpolateCurve::has_end_deriv() const
{
  return !m_Dn.IsOrigin();
}

//! Returns true if start derivative D2 is specified, false -- otherwise.
//! \return true/false.
bool mobius::geom_InterpolateCurve::has_start_deriv2() const
{
  return !m_D20.IsOrigin();
}

//! Returns true if end derivative D2 is specified, false -- otherwise.
//! \return true/false.
bool mobius::geom_InterpolateCurve::has_end_deriv2() const
{
  return !m_D2n.IsOrigin();
}

//! Returns dimension of the problem: number of unknown variables and
//! equations in the linear system to solve.
//! \param n                [in] last pole index.
//! \param has_start_deriv  [in] indicates whether start derivative D1 is specified.
//! \param has_end_deriv    [in] indicates whether end derivative D1 is specified.
//! \param has_start_deriv2 [in] indicates whether start derivative D2 is specified.
//! \param has_end_deriv2   [in] indicates whether end derivative D2 is specified.
//! \return dimension.
int mobius::geom_InterpolateCurve::dimension(const int  n,
                                             const bool has_start_deriv,
                                             const bool has_end_deriv,
                                             const bool has_start_deriv2,
                                             const bool has_end_deriv2)
{
  return n + 1 + (has_start_deriv  ? 1 : 0) + (has_end_deriv  ? 1 : 0)
               + (has_start_deriv2 ? 1 : 0) + (has_end_deriv2 ? 1 : 0);
}
