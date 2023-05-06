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
  #define dump_filename_N  "C:/users/serge/desktop/N_interp_log.log"
  #define dump_filename_Bx "C:/users/serge/desktop/N_interp_log_Bx.log"
  #define dump_filename_By "C:/users/serge/desktop/N_interp_log_By.log"
  #define dump_filename_Bz "C:/users/serge/desktop/N_interp_log_Bz.log"
#endif

//-----------------------------------------------------------------------------

mobius::geom_InterpolateCurve::geom_InterpolateCurve(core_ProgressEntry progress,
                                                     core_PlotterEntry  plotter)
: core_OPERATOR (progress, plotter),
  m_iDeg        (0),
  m_errCode     (ErrCode_NoError),
  m_paramsType  (ParamsSelection_Undefined),
  m_pParams     (nullptr),
  m_iNumParams  (0),
  m_knotsType   (KnotsSelection_Undefined),
  m_pU          (nullptr),
  m_iNumKnots   (0)
{
  m_errCode = ErrCode_NotInitialized;
}

//-----------------------------------------------------------------------------

mobius::geom_InterpolateCurve::geom_InterpolateCurve(const std::vector<t_xyz>&  points,
                                                     const int                  deg,
                                                     const bspl_ParamsSelection paramsType,
                                                     const bspl_KnotsSelection  knotsType,
                                                     core_ProgressEntry         progress,
                                                     core_PlotterEntry          plotter)
: core_OPERATOR(progress, plotter)
{
  this->Init(points, deg, paramsType, knotsType);
}

//-----------------------------------------------------------------------------

mobius::geom_InterpolateCurve::geom_InterpolateCurve(const std::vector<t_xyz>& points,
                                                     const int                 deg,
                                                     double*                   pParams,
                                                     const int                 n,
                                                     double*                   pU,
                                                     const int                 m,
                                                     core_ProgressEntry        progress,
                                                     core_PlotterEntry         plotter)
: core_OPERATOR(progress, plotter)
{
  this->Init(points, deg, pParams, n, pU, m);
}

//-----------------------------------------------------------------------------

void mobius::geom_InterpolateCurve::Init(const std::vector<t_xyz>&  points,
                                         const int                  deg,
                                         const bspl_ParamsSelection paramsType,
                                         const bspl_KnotsSelection  knotsType)
{
  m_points     = points;
  m_iDeg       = deg;
  m_paramsType = paramsType;
  m_pParams    = nullptr;
  m_iNumParams = 0;
  m_knotsType  = knotsType;
  m_errCode    = ErrCode_NotDone;
  m_pU         = nullptr;
  m_iNumKnots  = 0;
}

//-----------------------------------------------------------------------------

void mobius::geom_InterpolateCurve::Init(const std::vector<t_xyz>& points,
                                         const int                 deg,
                                         double*                   pParams,
                                         const int                 n,
                                         double*                   pU,
                                         const int                 m)
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

void mobius::geom_InterpolateCurve::Init(const std::vector<t_xyz>&  points,
                                         const t_xyz&               D0,
                                         const t_xyz&               Dn,
                                         const int                  deg,
                                         const bspl_ParamsSelection paramsType,
                                         const bspl_KnotsSelection  knotsType)
{
  m_points     = points;
  m_D0         = D0;
  m_Dn         = Dn;
  m_iDeg       = deg;
  m_paramsType = paramsType;
  m_pParams    = nullptr;
  m_iNumParams = 0;
  m_knotsType  = knotsType;
  m_pU         = nullptr;
  m_iNumKnots  = 0;
  m_errCode    = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

void mobius::geom_InterpolateCurve::Init(const std::vector<t_xyz>&  points,
                                         const t_xyz&               D0,
                                         const t_xyz&               Dn,
                                         const t_xyz&               D20,
                                         const t_xyz&               D2n,
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
  m_pParams    = nullptr;
  m_iNumParams = 0;
  m_knotsType  = knotsType;
  m_pU         = nullptr;
  m_iNumKnots  = 0;
  m_errCode    = ErrCode_NotDone;
}

//-----------------------------------------------------------------------------

bool mobius::geom_InterpolateCurve::Perform()
{
  m_errCode = ErrCode_NoError;

  // Heap allocator
  core_HeapAlloc<double> Alloc;

  /* -------------------------------
   *  Choose interpolant parameters
   * ------------------------------- */

  int     n      = 0;
  double* params = nullptr;

  // Now parameterize
  if ( m_paramsType == ParamsSelection_Uniform )
  {
    // There are as many parameters as many data points are passed for interpolation
    n      = this->last_index_poles();
    params = Alloc.Allocate(n + 1, true);

    if ( bspl_ParamsUniform::Calculate(n, params) != bspl_ParamsUniform::ErrCode_NoError )
    {
      m_errCode = ErrCode_CannotSelectParameters;
      return false;
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
      return false;
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
      return false;
    }
  }
  else if ( m_paramsType == ParamsSelection_Undefined && m_pParams && m_iNumParams )
  {
    // Accept the externally defined values
    n      = m_iNumParams - 1;
    params = m_pParams;
  }
  else
    throw std::runtime_error("NYI parameterization type");

  // TODO: introduce other parameterization techniques

  /* --------------------------
   *  Choose interpolant knots
   * -------------------------- */

  int     m = 0;
  double* U = nullptr;

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
      return false;
    }
  }
  else if ( m_knotsType == KnotsSelection_Undefined && m_pU && m_iNumKnots )
  {
    // Accept the externally defined values
    m = m_iNumKnots - 1;
    U = m_pU;
  }
  else
    throw std::runtime_error("NYI knots selection type");

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
    return false;
  }

  return true; // Success.
}

//-----------------------------------------------------------------------------

bool mobius::geom_InterpolateCurve::Interp(const std::vector<t_xyz>& points,
                                           const int                 n,
                                           const int                 p,
                                           const double*             params,
                                           const double*             pU,
                                           const int                 m,
                                           const bool                has_start_deriv,
                                           const bool                has_end_deriv,
                                           const bool                has_start_deriv2,
                                           const bool                has_end_deriv2,
                                           const t_xyz&              D0,
                                           const t_xyz&              Dn,
                                           const t_xyz&              D20,
                                           const t_xyz&              D2n,
                                           t_ptr<t_bcurve>&          crv)
{
  if ( (has_start_deriv2 && !has_start_deriv) || (has_end_deriv2 && !has_end_deriv) )
    throw std::runtime_error("Cannot handle D2 without D1"); // TODO: this limitation can be easily escaped

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
  std::vector<t_xyz> Poles;
  for ( int k = 0; k < dim; ++k )
  {
    t_xyz P(pXYZ[0][k], pXYZ[1][k], pXYZ[2][k]);
    Poles.push_back(P);
  }

  // Set result
  crv = new t_bcurve(Poles, U, p);

  /* ----------------
   *  Release memory
   * ---------------- */

  for ( int c = 0; c < 3; ++c )
    delete[] pXYZ[c];

  delete[] pXYZ;
  return true;
}

//-----------------------------------------------------------------------------

int mobius::geom_InterpolateCurve::last_index_poles() const
{
  int n = (int) (m_points.size() - 1);
  return n;
}

//-----------------------------------------------------------------------------

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

bool mobius::geom_InterpolateCurve::has_start_deriv() const
{
  return !m_D0.IsOrigin();
}

//-----------------------------------------------------------------------------

bool mobius::geom_InterpolateCurve::has_end_deriv() const
{
  return !m_Dn.IsOrigin();
}

//-----------------------------------------------------------------------------

bool mobius::geom_InterpolateCurve::has_start_deriv2() const
{
  return !m_D20.IsOrigin();
}

//-----------------------------------------------------------------------------

bool mobius::geom_InterpolateCurve::has_end_deriv2() const
{
  return !m_D2n.IsOrigin();
}

//-----------------------------------------------------------------------------

int mobius::geom_InterpolateCurve::dimension(const int  n,
                                             const bool has_start_deriv,
                                             const bool has_end_deriv,
                                             const bool has_start_deriv2,
                                             const bool has_end_deriv2)
{
  return n + 1 + (has_start_deriv  ? 1 : 0) + (has_end_deriv  ? 1 : 0)
               + (has_start_deriv2 ? 1 : 0) + (has_end_deriv2 ? 1 : 0);
}
