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

// Geom includes
#include <mobius/geom_JSON.h>

// Core includes
#include <mobius/core_HeapAlloc.h>
#include <mobius/core_Integral.h>
#include <mobius/core_JSON.h>
#include <mobius/core_UnivariateFunc.h>

// BSpl includes
#include <mobius/bspl_Decompose.h>
#include <mobius/bspl_EffectiveN.h>
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>
#include <mobius/bspl_InsKnot.h>
#include <mobius/bspl_RefineKnots.h>

// STD includes
#include <algorithm>

//-----------------------------------------------------------------------------

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Univariate function representing the squared second derivative of
//! a parametric curve.
class geom_CuuSquared : public core_UnivariateFunc
{
public:

  //! ctor.
  //! \param[in] curve parametric curve in question.
  geom_CuuSquared(const ptr<geom_BSplineCurve>& curve) : core_UnivariateFunc()
  {
    m_curve = curve;
  }

public:

  //! Evaluates the second derivative squared.
  //! \param[in] u parameter value.
  //! \return evaluated function.
  virtual double Eval(const double u) const
  {
    xyz D2;
    m_curve->Eval_Dk(u, 2, D2);

    return D2.Dot(D2);
  }

public:

  //! \return curve in question.
  const ptr<geom_BSplineCurve>& GetCurve() const
  {
    return m_curve;
  }

protected:

  ptr<geom_BSplineCurve> m_curve; //!< Curve.

};

};

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

//! Destructor.
mobius::geom_BSplineCurve::~geom_BSplineCurve()
{}

//-----------------------------------------------------------------------------

//! Constructs B-curve from JSON.
//! \param[in] json JSON string to create a curve from.
//! \return constructed B-curve or null if JSON is of invalid format.
mobius::core_Ptr<mobius::geom_BSplineCurve>
  mobius::geom_BSplineCurve::Instance(const std::string& json)
{
  core_Ptr<bcurve> result;
  if ( !geom_JSON(json).ExtractBCurve(result) )
    return NULL;

  return result;
}

//-----------------------------------------------------------------------------

//! Dumps this B-curve as JSON object.
//! \return JSON string.
std::string mobius::geom_BSplineCurve::DumpJSON() const
{
  geom_JSON dumper;
  dumper.DumpBCurve(this);

  std::string res = dumper.GetJSON();

  return res;
}

//-----------------------------------------------------------------------------

//! Calculates boundary box for the B-spline curve by its control polygon.
//! Notice that this peculiarity can look weird as control polygon only
//! outlines the B-spline curve, but does not follow its exact shape.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_BSplineCurve::GetBounds(double& xMin, double& xMax,
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

//-----------------------------------------------------------------------------

//! Returns first knot.
//! \return first knot.
double mobius::geom_BSplineCurve::GetMinParameter() const
{
  return m_U[0];
}

//-----------------------------------------------------------------------------

//! Returns last knot.
//! \return last knot.
double mobius::geom_BSplineCurve::GetMaxParameter() const
{
  return m_U[m_U.size()-1];
}

//-----------------------------------------------------------------------------

//! Evaluates B-spline curve for the given parameter.
//! This algorithm is essentially the algorithm A3.1 from The NURBS Book.
//!
//! \param u [in]  parameter value to evaluate the curve for.
//! \param P [out] 3D point corresponding to the given parameter on the curve.
void mobius::geom_BSplineCurve::Eval(const double u,
                                     xyz&         P) const
{
  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_U, m_iDeg);
  const int span = FindSpan(u);

  // Evaluate effective B-spline basis functions
  bspl_EffectiveN EffectiveN;
  double N[mobiusBSpl_MaxDegree];
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

//-----------------------------------------------------------------------------

//! Evaluates derivative of the B-spline curve for the given parameter.
//! \param u                [in]  parameter value to evaluate curve for.
//! \param k                [in]  the desired order of derivative.
//! \param dkC              [out] derivative vector.
//! \param alloc            [in]  optional allocator.
//! \param memBlockResult   [in]  index of the memory block for the result.
//! \param memBlockInternal [in]  index of the memory block for internal calculations.
void mobius::geom_BSplineCurve::Eval_Dk(const double u,
                                        const int    k,
                                        xyz&         dkC,
                                        ptr<alloc2d> alloc,
                                        const int    memBlockResult,
                                        const int    memBlockInternal) const
{
  ptr<alloc2d> localAlloc;

  double** dN;
  if ( alloc.IsNull() )
  {
    localAlloc = new alloc2d;
    dN = localAlloc->Allocate(m_iDeg + 1, m_iDeg + 1, true);
  }
  else
    dN = alloc->Access(memBlockResult).Ptr;

  this->Eval_Dk(dN, u, k, dkC, alloc, memBlockInternal);
}

//-----------------------------------------------------------------------------

//! Evaluates derivative of the B-spline curve for the given parameter.
//! \param dN               [in]  array for results (can be passed for better performance).
//! \param u                [in]  parameter value to evaluate curve for.
//! \param k                [in]  desired order of derivative.
//! \param d1C              [out] derivative vector.
//! \param alloc            [in]  optional allocator.
//! \param memBlockInternal [in]  index of the memory block for internal calculations.
void mobius::geom_BSplineCurve::Eval_Dk(double**     dN,
                                        const double u,
                                        const int    k,
                                        xyz&         d1C,
                                        ptr<alloc2d> alloc,
                                        const int    memBlockInternal) const
{
  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_U, m_iDeg);
  const int span = FindSpan(u);

  // Evaluate derivatives of B-spline basis functions
  bspl_EffectiveNDers NDers(alloc, memBlockInternal);
  NDers(u, m_U, m_iDeg, span, k, dN);

  // Evaluate curve
  xyz C;
  for ( int i = 0; i <= m_iDeg; ++i )
  {
    C += m_poles[span-m_iDeg+i].Multiplied(dN[k][i]); // See theory for clarifications on indices
  }

  // Set output parameter
  d1C = C;
}

//-----------------------------------------------------------------------------

//! Creates a copy of this B-curve.
//! \return copy of B-curve.
mobius::ptr<mobius::bcurve> mobius::geom_BSplineCurve::Copy() const
{
  return new bcurve(m_poles, m_U, m_iDeg);
}

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

mobius::core_Continuity
  mobius::geom_BSplineCurve::CheckContinuityByKnots(const std::vector<double>& knots,
                                                    const int                  degree)
{
  std::vector<int> mults;
  int mult = 1;
  //
  for ( size_t i = 0; i < knots.size(); ++i )
  {
    if ( knots[i] == knots[0] )
      continue;

    if ( fabs( knots[i] - knots[i - 1] ) < DBL_EPSILON )
      ++mult;
    else if ( knots[i - 1] != knots[0] )
    {
      mults.push_back(mult);
      mult = 1;

      if ( knots[i] == knots[knots.size() - 1] )
        break;
    }
  }

  if ( mults.size() )
  {
    const int max_mult = *std::max_element( mults.begin(), mults.end() );
    const int cont     = degree - max_mult;

    if ( cont <= 0 )
      return Continuity_C0;
    if ( cont == 1 )
      return Continuity_C1;
    if ( cont == 2 )
      return Continuity_C2;
    if ( cont == 3 )
      return Continuity_C3;
  }

  return Continuity_CN;
}

//-----------------------------------------------------------------------------

//! Returns continuity of the curve.
//! \return continuity.
mobius::core_Continuity
  mobius::geom_BSplineCurve::GetContinuity() const
{
  return CheckContinuityByKnots(m_U, m_iDeg);
}

//-----------------------------------------------------------------------------

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
  double    u        = ( this->GetMinParameter() + this->GetMaxParameter() )*0.5;

  // Newton iterations
  do
  {
    if ( u < this->GetMinParameter() )
      u = this->GetMaxParameter(); // Try another extremity

    if ( u > this->GetMaxParameter() )
      u = this->GetMinParameter(); // Try another extremity

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

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

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
  //
  std::vector<double> UQ;
  std::vector<xyz> Qw;

  // Insert knot
  bspl_InsKnot Insert;
  if ( !Insert(np, m_iDeg, m_U, m_poles, u, k, s, num_times, nq, UQ, Qw) )
    return false;

  // Reinitialize
  this->init(Qw, UQ, m_iDeg);
  return true;
}

//-----------------------------------------------------------------------------

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

  std::vector<double> Ubar;/* Ubar.reserve(m_new + 1);*/
  //
  for ( int k = 0; k < m_new + 1; ++k )
    Ubar.push_back(0.0);

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

//-----------------------------------------------------------------------------

//! Splits B-curve by two slices with the given parameter.
//! \param u      [in]  parameter to split by.
//! \param slices [out] resulting curve slices.
//! \return true in case of success, false -- otherwise.
bool mobius::geom_BSplineCurve::Split(const double                u,
                                      std::vector< ptr<bcurve> >& slices) const
{
  // Create a copy of this curve as knot insertion modifies the object
  // (not real geometry)
  ptr<bcurve> source = this->Copy();

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
    poles_before.push_back( source->GetPoles()[i] );

  const size_t nU_before = k + m_iDeg + 2;

  // Knots
  std::vector<double> U_before; U_before.resize(nU_before);
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
  for ( size_t i = k; i < source->GetPoles().size(); ++i )
    poles_after.push_back( source->GetPoles()[i] );

  std::vector<double> source_knots = source->GetKnots();
  const int           source_m     = (int) (source_knots.size() - 1);
  const size_t        nU_after     = source_m - k + 1;

  // Knots
  std::vector<double> U_after; U_after.resize(nU_after);
  for ( int i = k + 1, j = 1; i <= source_m; ++i, ++j )
  {
    U_after[j] = source_knots[i];
  }
  U_after[0] = u;

  /* ============================
   *  Create b-curves and finish
   * ============================ */

  ptr<bcurve>
    slice_before = new bcurve(poles_before, U_before, m_iDeg);

  ptr<bcurve>
    slice_after = new bcurve(poles_after, U_after, m_iDeg);

  slices.push_back(slice_before);
  slices.push_back(slice_after);
  return true;
}

//-----------------------------------------------------------------------------

//! Performs linear re-parameterization of B-curve: poles remain untouched,
//! while knots are moved according to linear law.
//! \param s_min [in] new min parameter value.
//! \param s_max [in] new max parameter value.
void mobius::geom_BSplineCurve::ReparameterizeLinear(const double s_min,
                                                     const double s_max)
{
  const double u_min = this->GetMinParameter();
  const double u_max = this->GetMaxParameter();

  for ( size_t i = 0; i < m_U.size(); ++i )
  {
    m_U[i] = BSplCurveProj::g(m_U[i],
                              u_min,
                              u_max,
                              s_min,
                              s_max);
  }
}

//-----------------------------------------------------------------------------

double mobius::geom_BSplineCurve::ComputeStrainEnergy() const
{
  const int NUM_GAUSS_PT = 2*m_iDeg - 1; // (2n-1) for max accuracy on polynomial functions.

  geom_CuuSquared Cuu2Func(this);

  double result = 0;
  for ( size_t k = 0; k < m_U.size() - 1; ++k )
  {
    if ( m_U[k] == m_U[k+1] ) continue; // Skip multiple knots.

    // 6-points integration in each knot span.
    const double
      gaussVal = core_Integral::gauss::Compute(&Cuu2Func, m_U[k], m_U[k+1], NUM_GAUSS_PT);
    //
    result += gaussVal;
  }

  return result;
}

//-----------------------------------------------------------------------------

bool mobius::geom_BSplineCurve::SplitToBezier(std::vector< ptr<bcurve> >& segments) const
{
  // Input arguments.
  const int                  n  = this->GetNumOfPoles() - 1;
  const int                  p  = this->GetDegree();
  const std::vector<double>& U  = this->GetKnots();
  const std::vector<xyz>&    Pw = this->GetPoles();

  // Output arguments.
  int nb = 0;
  std::vector< std::vector<xyz> > Qw;
  std::vector<int> breakpoints;

  // Perform curve decomposition.
  bspl_Decompose decomposer;
  //
  if ( !decomposer(n, p, U, Pw, nb, Qw, breakpoints) )
    return false;

  // At least two segments are expected for the decomposition to be
  // successful.
  if ( Qw.size() <= 1 )
    return false;

  // Construct B-curves for the segments.
  for ( size_t k = 0; k < Qw.size(); ++k )
  {
    // Order of the Bezier segment is equal to the number of points in a segment.
    const int bezOrder = int( Qw[k].size() );

    // Prepare min knot.
    double bezUmin;
    //
    if ( k > 0 )
      bezUmin = m_U[ breakpoints[k - 1] ];
    else
      bezUmin = this->GetMinParameter();

    // Prepare max knot.
    const double bezUmax = m_U[ breakpoints[k] ];

    // Prepare knot vector.
    std::vector<double> bezU;
    //
    for ( int ii = 0; ii < bezOrder; ++ii ) bezU.push_back(bezUmin);
    for ( int ii = 0; ii < bezOrder; ++ii ) bezU.push_back(bezUmax);

    // Construct Bezier segment.
    ptr<bcurve> segment = new bcurve(Qw[k], bezU, bezOrder - 1);
    //
    segments.push_back(segment);
  }

  return true;
}

//-----------------------------------------------------------------------------

void mobius::geom_BSplineCurve::init(const std::vector<xyz>&    Poles,
                                     const std::vector<double>& U,
                                     const int                  p)
{
  // Check if max degree is not exceeded.
  if ( p > mobiusBSpl_MaxDegree )
    throw bspl_excMaxDegreeViolation();

  // Check if B-curve can be constructed.
  if ( !bspl::Check(int( Poles.size() ) - 1, int( U.size() ) - 1, p) )
    throw geom_excBCurveCtor();

  // Initialize members.
  m_poles = Poles;
  m_U     = U;
  m_iDeg  = p;
}
