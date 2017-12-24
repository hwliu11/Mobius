//-----------------------------------------------------------------------------
// Created on: 23 May 2013
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
#include <mobius/geom_BSplineCurve.h>

// Core includes
#include <mobius/core_HeapAlloc.h>

// BSpl includes
#include <mobius/bspl_EffectiveN.h>
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>
#include <mobius/bspl_InsKnot.h>
#include <mobius/bspl_RefineKnots.h>

// STD includes
#include <algorithm>

//! Auxiliary functions.
namespace BSplCurveProj
{
  double F(const mobius::bcurve* crv,
           const double          u,
           const mobius::xyz&    P)
  {
    mobius::xyz C, d1C;
    crv->Eval(u, C);
    crv->Eval_Dk(u, 1, d1C);
    return (C - P).Dot(d1C);
  }

  double dF(const mobius::bcurve* crv,
            const double          u,
            const mobius::xyz&    P)
  {
    mobius::xyz C, d1C, d2C;
    crv->Eval(u, C);
    crv->Eval_Dk(u, 1, d1C);
    crv->Eval_Dk(u, 2, d2C);
    return d1C.SquaredModulus() + (C - P).Dot(d2C);
  }

  double g(const double u,
           const double u0, const double u1,
           const double s0, const double s1)
  {
    double param = u*(s1 - s0)/(u1 - u0) + (s0*u1 - s1*u0)/(u1 - u0);

    // Round-off errors may break the bounds
    if ( param > s1 )
      param = s1;
    if ( param < s0 )
      param = s0;

    return param;
  }
};

//! Constructor.
//! \param Poles [in] poles for B-spline curve.
//! \param U     [in] knot vector.
//! \param nU    [in] number of knots.
//! \param p     [in] degree.
mobius::geom_BSplineCurve::geom_BSplineCurve(const std::vector<xyz>& Poles,
                                             const double*           U,
                                             const int               nU,
                                             const int               p)
: geom_Curve()
{
  std::vector<double> Uvec;
  for ( int i = 0; i < nU; ++i )
    Uvec.push_back(U[i]);

  this->init(Poles, Uvec, p);
}

//! Constructor.
//! \param Poles [in] poles for B-spline curve.
//! \param U     [in] knot vector.
//! \param p     [in] degree.
mobius::geom_BSplineCurve::geom_BSplineCurve(const std::vector<xyz>&    Poles,
                                             const std::vector<double>& U,
                                             const int                  p)
: geom_Curve()
{
  this->init(Poles, U, p);
}

//! Destructor.
mobius::geom_BSplineCurve::~geom_BSplineCurve()
{}

//! Calculates boundary box for the B-spline curve by its control polygon.
//! Notice that this peculiarity can look weird as control polygon only
//! outlines the B-spline curve, but does not follow its exact shape.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_BSplineCurve::Bounds(double& xMin, double& xMax,
                                       double& yMin, double& yMax,
                                       double& zMin, double& zMax) const
{
  double x_min = DBL_MAX, x_max = -DBL_MAX;
  double y_min = DBL_MAX, y_max = -DBL_MAX;
  double z_min = DBL_MAX, z_max = -DBL_MAX;

  // B-spline curve is fully contained in its control polygon, so we can take
  // it as a rough solution
  for ( int p = 0; p < (int) m_poles.size(); ++p )
  {
    const xyz& P = m_poles.at(p);
    const double x = P.X(), y = P.Y(), z = P.Z();

    if ( x > x_max )
      x_max = x;
    if ( x < x_min )
      x_min = x;
    if ( y > y_max )
      y_max = y;
    if ( y < y_min )
      y_min = y;
    if ( z > z_max )
      z_max = z;
    if ( z < z_min )
      z_min = z;
  }

  // Set results
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}

//! Returns first knot.
//! \return first knot.
double mobius::geom_BSplineCurve::MinParameter() const
{
  return m_U[0];
}

//! Returns last knot.
//! \return last knot.
double mobius::geom_BSplineCurve::MaxParameter() const
{
  return m_U[m_U.size()-1];
}

//! Evaluates B-spline curve for the given parameter.
//! \param u [in]  parameter value to evaluate the curve for.
//! \param P [out] 3D point corresponding to the given parameter on the curve.
void mobius::geom_BSplineCurve::Eval(const double u,
                                     xyz&         P) const
{
  // Heap allocator
  core_HeapAlloc<double> Alloc;

  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_U, m_iDeg);
  const int span = FindSpan(u);

  // Evaluate effective B-spline basis functions
  bspl_EffectiveN EffectiveN;
  double* N = Alloc.Allocate(m_iDeg + 1, false);
  EffectiveN(u, m_U, m_iDeg, span, N);

  // Evaluate curve
  xyz C;
  for ( int i = 0; i <= m_iDeg; ++i )
  {
    C += m_poles[span-m_iDeg+i].Multiplied(N[i]); // See theory for clarifications on indices
  }

  // Set output parameter
  P = C;
}

//! Evaluates derivative of the B-spline curve for the given parameter.
//! \param u   [in]  parameter value to evaluate curve for.
//! \param k   [in]  the desired order of derivative.
//! \param dkC [out] derivative vector.
void mobius::geom_BSplineCurve::Eval_Dk(const double u,
                                        const int    k,
                                        xyz&         dkC) const
{
  core_HeapAlloc2D<double> Alloc;
  double** dN = Alloc.Allocate(m_iDeg + 1, m_iDeg + 1, true);
  //
  this->Eval_Dk(dN, u, k, dkC);
}

//! Evaluates derivative of the B-spline curve for the given parameter.
//! \param dN  [in]  array for results (can be passed for better performance).
//! \param u   [in]  parameter value to evaluate curve for.
//! \param k   [in]  desired order of derivative.
//! \param d1C [out] derivative vector.
void mobius::geom_BSplineCurve::Eval_Dk(double**     dN,
                                        const double u,
                                        const int    k,
                                        xyz&         d1C) const
{
  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_U, m_iDeg);
  const int span = FindSpan(u);

  // Evaluate derivatives of B-spline basis functions
  bspl_EffectiveNDers NDers;
  NDers(u, m_U, m_iDeg, span, m_iDeg, dN);

  // Evaluate curve
  xyz C;
  for ( int i = 0; i <= m_iDeg; ++i )
  {
    C += m_poles[span-m_iDeg+i].Multiplied(dN[k][i]); // See theory for clarifications on indices
  }

  // Set output parameter
  d1C = C;
}

//! Creates a copy of this B-curve.
//! \return copy of B-curve.
mobius::Ptr<mobius::bcurve> mobius::geom_BSplineCurve::Copy() const
{
  return new bcurve(m_poles, m_U, m_iDeg);
}

//! Calculates curvature value at the given parameter.
//! \param u [in] parameter value to evaluate curvature in.
//! \return curvature value.
double mobius::geom_BSplineCurve::K(const double u) const
{
  xyz d1C, d2C;
  this->Eval_Dk(u, 1, d1C);
  this->Eval_Dk(u, 2, d2C);

  const double k1 = d1C.Cross(d2C).Modulus();
  const double k2 = pow(d1C.Modulus(), 3);
  return k1/k2;
}

//! Returns continuity of the curve.
//! \return continuity.
mobius::core_Smoothness mobius::geom_BSplineCurve::Continuity() const
{
  std::vector<int> mults;
  int mult = 1;
  //
  for ( size_t i = 0; i < m_U.size(); ++i )
  {
    if ( m_U[i] == this->MinParameter() )
      continue;

    if ( fabs( m_U[i] - m_U[i - 1] ) < DBL_EPSILON )
      ++mult;
    else if ( m_U[i - 1] != this->MinParameter() )
    {
      mults.push_back(mult);
      mult = 1;

      if ( m_U[i] == this->MaxParameter() )
        break;
    }
  }

  if ( mults.size() )
  {
    const int max_mult = *std::max_element( mults.begin(), mults.end() );
    const int cont     = m_iDeg - max_mult;

    if ( cont <= 0 )
      return Smoothness_C0;
    if ( cont == 1 )
      return Smoothness_C1;
    if ( cont == 2 )
      return Smoothness_C2;
    if ( cont == 3 )
      return Smoothness_C3;
  }

  return Smoothness_CN;
}

//! Calculates parameter value for the given point on curve.
//! \param P     [in]  point to invert.
//! \param prec  [in]  precision to use.
//! \param param [out] parameter on curve.
//! \return true in case of success, false -- otherwise.
bool mobius::geom_BSplineCurve::InvertPoint(const xyz&   P,
                                            double&      param,
                                            const double prec) const
{
  // Working variables
  const int max_iter = 100;
  int       iter     = 0;
  bool      stop     = false;
  double    u        = ( this->MinParameter() + this->MaxParameter() )*0.5;

  // Newton iterations
  do
  {
    if ( u < this->MinParameter() )
      u = this->MaxParameter(); // Try another extremity

    if ( u > this->MaxParameter() )
      u = this->MinParameter(); // Try another extremity

    const double f = BSplCurveProj::F(this, u, P);
    if ( fabs(f) < prec )
    {
      stop = true;
      continue;
    }

    // Continue iterations
    const double df = BSplCurveProj::dF(this, u, P);
    u = u - f / df; // Classic and simplest formulation of Newton iterations
    iter++;
  }
  while ( !stop && iter < max_iter );

  if ( fabs( BSplCurveProj::F(this, u, P) ) > prec )
    return false;

  param = u;
  return true;
}

//! Inserts knot to the knot vector of the curve.
//! \param u         [in] knot to insert.
//! \param num_times [in] how many times to insert. If the passed knot does not
//!                       exist yet, this parameter will become knot's
//!                       multiplicity. Otherwise the resulting knot's
//!                       multiplicity is its original multiplicity plus
//!                       num_times value.
//! \return true in case of success, false -- otherwise.
bool mobius::geom_BSplineCurve::InsertKnot(const double u,
                                           const int    num_times)
{
  int dest_span_idx;
  return this->InsertKnot(u, num_times, dest_span_idx);
}

//! Inserts knot to the knot vector of the curve.
//! \param u             [in]  knot to insert.
//! \param num_times     [in]  how many times to insert. If the passed knot does not
//!                            exist yet, this parameter will become knot's
//!                            multiplicity. Otherwise the resulting knot's
//!                            multiplicity is its original multiplicity plus
//!                            num_times value.
//! \param dest_span_idx [out] span where the knot falls.
//! \return true in case of success, false -- otherwise.
bool mobius::geom_BSplineCurve::InsertKnot(const double u,
                                           const int    num_times,
                                           int&         dest_span_idx)
{
  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_U, m_iDeg);
  const int k = FindSpan(u);
  dest_span_idx = k;

  // Resolve multiplicity
  int s = 0;
  for ( size_t i = 0; i < m_U.size(); ++i )
    if ( m_U[i] == u )
      s++;

  // Working variables
  const int np = (int) (m_poles.size() - 1);
  int       nq = 0;
  int       mq = np + m_iDeg + 1 + num_times;
  //
  std::vector<double> UQ; UQ.reserve(mq + 1);
  std::vector<xyz> Qw;

  // Insert knot
  bspl_InsKnot Insert;
  if ( !Insert(np, m_iDeg, m_U, m_poles, u, k, s, num_times, nq, UQ, Qw) )
    return false;

  // Release resources
  this->init(Qw, UQ, m_iDeg);
  return true;
}

//! Refines knots of the underlying basis functions using the passed vector
//! of new knot values. This vector must contain only new values with
//! repetitions corresponding to multiplicities. E.g. X = [0.2, 0.3, 0.3]
//! will effect in insertion of 0.2 of multiplicity 1 and 0.3 of
//! multiplicity 2.
//! \param X [in] refinement vector.
//! \return true in case of success, false -- otherwise.
bool mobius::geom_BSplineCurve::RefineKnots(const std::vector<double>& X)
{
  if ( !X.size() )
    return true;

  // Working variables
  const int n     = (int) (m_poles.size() - 1);
  const int r     = (int) (X.size() - 1);
  const int m_new = n + m_iDeg + 1 + r + 1;
  //
  double *pX = new double[X.size()];
  //
  for ( size_t i = 0; i < X.size(); ++i )
    pX[i] = X[i];

  std::vector<double> Ubar; Ubar.reserve(m_new + 1);
  std::vector<xyz> Qw;

  // Refine knots
  bspl_RefineKnots Refine;
  if ( !Refine(n, m_iDeg, m_U, m_poles, pX, r, Ubar, Qw) )
    return false;

  // Release resources
  delete[] pX;
  this->init(Qw, Ubar, m_iDeg);
  return true;
}

//! Splits B-curve by two slices with the given parameter.
//! \param u      [in]  parameter to split by.
//! \param slices [out] resulting curve slices.
//! \return true in case of success, false -- otherwise.
bool mobius::geom_BSplineCurve::Split(const double                u,
                                      std::vector< Ptr<bcurve> >& slices) const
{
  // Create a copy of this curve as knot insertion modifies the object
  // (not real geometry)
  Ptr<bcurve> source = this->Copy();

  // Resolve multiplicity
  int s = 0;
  for ( size_t i = 0; i < m_U.size(); ++i )
    if ( m_U[i] == u )
      s++;

  // Insert knot u degree times to obtain two control polygons which can
  // be considered separately
  int k;
  if ( !source->InsertKnot(u, m_iDeg, k) )
    return false;

  /* ==============================================
   *  Prepare slice before the requested parameter
   * ============================================== */

  // Poles
  std::vector<xyz> poles_before;
  for ( int i = 0; i <= k; ++i )
    poles_before.push_back( source->Poles()[i] );

  const size_t nU_before = k + m_iDeg + 2;

  // Knots
  std::vector<double> U_before; U_before.reserve(nU_before);
  //
  for ( size_t i = 0; i < nU_before - 1; ++i )
  {
    U_before[i] = source->m_U[i];
  }
  U_before[nU_before - 1] = u;

  /* =============================================
   *  Prepare slice after the requested parameter
   * ============================================= */

  // Poles
  std::vector<xyz> poles_after;
  for ( size_t i = k; i < source->Poles().size(); ++i )
    poles_after.push_back( source->Poles()[i] );

  std::vector<double> source_knots = source->Knots();
  const int           source_m     = (int) (source_knots.size() - 1);
  const size_t        nU_after     = source_m - k + 1;

  // Knots
  std::vector<double> U_after; U_after.reserve(nU_after);
  for ( int i = k + 1, j = 1; i <= source_m; ++i, ++j )
  {
    U_after[j] = source_knots[i];
  }
  U_after[0] = u;

  /* ============================
   *  Create b-curves and finish
   * ============================ */

  Ptr<bcurve>
    slice_before = new bcurve(poles_before, U_before, m_iDeg);

  Ptr<bcurve>
    slice_after = new bcurve(poles_after, U_after, m_iDeg);

  slices.push_back(slice_before);
  slices.push_back(slice_after);
  return true;
}

//! Performs linear re-parameterization of B-curve: poles remain untouched,
//! while knots are moved according to linear law.
//! \param s_min [in] new min parameter value.
//! \param s_max [in] new max parameter value.
void mobius::geom_BSplineCurve::ReparameterizeLinear(const double s_min,
                                                     const double s_max)
{
  const double u_min = this->MinParameter();
  const double u_max = this->MaxParameter();

  for ( size_t i = 0; i < m_U.size(); ++i )
  {
    m_U[i] = BSplCurveProj::g(m_U[i],
                              u_min,
                              u_max,
                              s_min,
                              s_max);
  }
}

//! Initializes B-spline curve with complete data.
//! \param Poles [in] control points.
//! \param U     [in] knot vector.
//! \param p     [in] degree of the B-spline basis functions.
void mobius::geom_BSplineCurve::init(const std::vector<xyz>&    Poles,
                                     const std::vector<double>& U,
                                     const int                  p)
{
  m_poles = Poles;
  m_U     = U;
  m_iDeg  = p;
}
