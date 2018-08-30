//-----------------------------------------------------------------------------
// Created on: 26 October 2013
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

//-----------------------------------------------------------------------------

//! Default constructor.
mobius::geom_InterpolateCurve::geom_InterpolateCurve()
{
  m_errCode = ErrCode_NotInitialized;
}

//-----------------------------------------------------------------------------

//! Complete constructor accepting some strategy of automatic knots selection.
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

//-----------------------------------------------------------------------------

//! Complete constructor accepting the manually defined paraneters and
//! knot vector.
//! \param points  [in] data points to interpolate.
//! \param deg     [in] degree of B-spline functions to use for blending.
//! \param pParams [in] manually defined interpolation parameters.
//! \param n       [in] 0-based index of the last parameter.
//! \param pU      [in] manually defined knot vector.
//! \param m       [in] 0-based index of the last knot in the knot vector.
mobius::geom_InterpolateCurve::geom_InterpolateCurve(const std::vector<xyz>& points,
                                                     const int               deg,
                                                     adouble*                 pParams,
                                                     const int               n,
                                                     adouble*                 pU,
                                                     const int               m)
{
  this->Init(points, deg, pParams, n, pU, m);
}

//-----------------------------------------------------------------------------

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
  m_pParams    = NULL;
  m_iNumParams = 0;
  m_knotsType  = knotsType;
  m_errCode    = ErrCode_NotDone;
  m_pU         = NULL;
  m_iNumKnots  = 0;
}

//-----------------------------------------------------------------------------

//! Initializes interpolation tool.
//! \param points  [in] data points to interpolate.
//! \param deg     [in] degree of B-spline functions to use for blending.
//! \param pParams [in] manually defined interpolation parameters.
//! \param n       [in] 0-based index of the last parameter.
//! \param pU      [in] manually defined knot vector.
//! \param m       [in] 0-based index of the last knot in the knot vector.
void mobius::geom_InterpolateCurve::Init(const std::vector<xyz>& points,
                                         const int               deg,
                                         adouble*                 pParams,
                                         const int               n,
                                         adouble*                 pU,
                                         const int               m)
{
  m_points     = points;
  m_iDeg       = deg;
  m_paramsType = ParamsSelection_Undefined;
  m_pParams    = pParams;
  m_iNumParams = n + 1;
  m_knotsType  = KnotsSelection_Undefined;
  m_pU         = pU;
  m_iNumKnots  = m + 1;
  m_errCode    = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

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
  m_pParams    = NULL;
  m_iNumParams = 0;
  m_knotsType  = knotsType;
  m_pU         = NULL;
  m_iNumKnots  = 0;
  m_errCode    = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

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
  m_pParams    = NULL;
  m_iNumParams = 0;
  m_knotsType  = knotsType;
  m_pU         = NULL;
  m_iNumKnots  = 0;
  m_errCode    = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

//! Performs interpolation.
//! \return true in case of success, false -- otherwise.
void mobius::geom_InterpolateCurve::Perform()
{
  m_errCode = ErrCode_NoError;

  // Heap allocator
  core_HeapAlloc<adouble> Alloc;

  /* -------------------------------
   *  Choose interpolant parameters
   * ------------------------------- */

  int     n      = 0;
  adouble* params = NULL;

  // Now parameterize
  if ( m_paramsType == ParamsSelection_Uniform )
  {
    // There are as many parameters as many data points are passed for interpolation
    n      = this->last_index_poles();
    params = Alloc.Allocate(n + 1, true);

    if ( bspl_ParamsUniform::Calculate(n, params) != bspl_ParamsUniform::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return;
    }
  }
  else if ( m_paramsType == ParamsSelection_ChordLength )
  {
    // There are as many parameters as many data points are passed for interpolation
    n      = this->last_index_poles();
    params = Alloc.Allocate(n + 1, true);

    if ( bspl_ParamsChordLength::Calculate(m_points, params) != bspl_ParamsChordLength::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return;
    }
  }
  else if ( m_paramsType == ParamsSelection_Centripetal )
  {
    // There are as many parameters as many data points are passed for interpolation
    n      = this->last_index_poles();
    params = Alloc.Allocate(n + 1, true);

    if ( bspl_ParamsCentripetal::Calculate(m_points, params) != bspl_ParamsCentripetal::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return;
    }
  }
  else if ( m_paramsType == ParamsSelection_Undefined && m_pParams && m_iNumParams )
  {
    // Accept the externally defined values
    n      = m_iNumParams - 1;
    params = m_pParams;
  }
  else
    throw std::exception("NYI parameterization type");

  // TODO: introduce other parameterization techniques

  /* --------------------------
   *  Choose interpolant knots
   * -------------------------- */

  int     m = 0;
  adouble* U = NULL;

  if ( m_knotsType == KnotsSelection_Average )
  {
    m = this->last_index_knots();
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
  else if ( m_knotsType == KnotsSelection_Undefined && m_pU && m_iNumKnots )
  {
    // Accept the externally defined values
    m = m_iNumKnots - 1;
    U = m_pU;
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

//-----------------------------------------------------------------------------

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
                                           const adouble*           params,
                                           const adouble*           pU,
                                           const int               m,
                                           const bool              has_start_deriv,
                                           const bool              has_end_deriv,
                                           const bool              has_start_deriv2,
                                           const bool              has_end_deriv2,
                                           const xyz&              D0,
                                           const xyz&              Dn,
                                           const xyz&              D20,
                                           const xyz&              D2n,
                                           ptr<bcurve>&            crv)
{
  if ( has_start_deriv2 && !has_start_deriv || has_end_deriv2 && !has_end_deriv )
    throw std::exception("Cannot handle D2 without D1"); // TODO: this limitation can be easily escaped

  std::vector<adouble> U;
  for ( int i = 0; i <= m; ++i )
    U.push_back(pU[i]);

  /* -----------------------------------
   *  Prepare matrix of B-spline values
   * ----------------------------------- */

  // TODO: use optimized evaluation algorithm for B-splines

  core_HeapAlloc<adouble> Alloc;

  const int dim = dimension(n, has_start_deriv, has_end_deriv, has_start_deriv2, has_end_deriv2); // Dimension of the problem in 3D
  adouble* N_values = Alloc.Allocate(dim*dim, true);

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
      adouble coeff = p*(p - 1)/U[p+1];

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
      adouble coeff = p*(p - 1)/(1 - U[m-p-1]);

      N_values[dim*k + dim - 3] =  coeff/(1-U[m-p-2]);                                          // P_{n-2}
      N_values[dim*k + dim - 2] = -coeff*(2-U[m-p-1]-U[m-p-2])/((1 - U[m-p-1])*(1 - U[m-p-2])); // P_{n-1}
      N_values[dim*k + dim - 1] =  coeff/(1-U[m-p-1]);                                          // P_n
    }
    else
    {
      const adouble u_k = params[kparams];
      for ( int i = 0; i < dim; ++i ) // Loop over the indices of the unknown control points
      {
        const adouble N = Eval(u_k, U, p, i);
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
  adouble* b = Alloc.Allocate(dim, true);

  // Solution components
  adouble** pXYZ = new adouble*[3];
  for ( int c = 0; c < 3; ++c )
  {
    pXYZ[c] = new adouble[dim];
    memset(pXYZ[c], 0, dim*sizeof(adouble));
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

//-----------------------------------------------------------------------------

//! Returns index of the last pole. Notice that this index is zero-based,
//! so if we have K poles, it will return (K-1).
//! \return index of the last pole.
int mobius::geom_InterpolateCurve::last_index_poles() const
{
  int n = (int) (m_points.size() - 1);
  return n;
}

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

//! Returns true if start derivative D1 is specified, false -- otherwise.
//! \return true/false.
bool mobius::geom_InterpolateCurve::has_start_deriv() const
{
  return !m_D0.IsOrigin();
}

//-----------------------------------------------------------------------------

//! Returns true if end derivative D1 is specified, false -- otherwise.
//! \return true/false.
bool mobius::geom_InterpolateCurve::has_end_deriv() const
{
  return !m_Dn.IsOrigin();
}

//-----------------------------------------------------------------------------

//! Returns true if start derivative D2 is specified, false -- otherwise.
//! \return true/false.
bool mobius::geom_InterpolateCurve::has_start_deriv2() const
{
  return !m_D20.IsOrigin();
}

//-----------------------------------------------------------------------------

//! Returns true if end derivative D2 is specified, false -- otherwise.
//! \return true/false.
bool mobius::geom_InterpolateCurve::has_end_deriv2() const
{
  return !m_D2n.IsOrigin();
}

//-----------------------------------------------------------------------------

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
