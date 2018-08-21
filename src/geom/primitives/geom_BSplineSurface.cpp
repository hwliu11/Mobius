//-----------------------------------------------------------------------------
// Created on: 07 February 2014
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
#include <mobius/geom_BSplineSurface.h>

// Geom includes
#include <mobius/geom_JSON.h>

// Core includes
#include <mobius/core_HeapAlloc.h>
#include <mobius/core_Integral.h>
#include <mobius/core_JSON.h>
#include <mobius/core_TwovariateFunc.h>

// BSpl includes
#include <mobius/bspl_EffectiveN.h>
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>

//-----------------------------------------------------------------------------

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Twovariate function representing the squared second derivatives of
//! a parametric surface.
class geom_ThinPlateEnergies : public core_TwovariateFunc
{
public:

  //! ctor.
  //! \param[in] surface parametric surface in question.
  geom_ThinPlateEnergies(const ptr<geom_BSplineSurface>& surface) : core_TwovariateFunc()
  {
    m_surface = surface;
  }

public:

  //! Evaluates the sum of second derivatives squared.
  //! \param[in] u U parameter value.
  //! \param[in] v V parameter value.
  //! \return evaluated function.
  virtual double Eval(const double u, const double v) const
  {
    xyz P, dU, dV, d2U, d2V, d2UV;
    m_surface->Eval_D2(u, v, P, dU, dV, d2U, d2V, d2UV);

    const double E = d2U.Dot(d2U) + 2*d2UV.Dot(d2UV) + d2V.Dot(d2V);
    return E;
  }

public:

  //! \return surface in question.
  const ptr<geom_BSplineSurface>& GetSurface() const
  {
    return m_surface;
  }

protected:

  ptr<geom_BSplineSurface> m_surface; //!< Surface.

};

};

//-----------------------------------------------------------------------------

//! Constructor.
//! \param Poles [in] poles for B-spline surface.
//! \param U     [in] knot vector in U dimension.
//! \param V     [in] knot vector in V dimension.
//! \param nU    [in] number of knots in U dimension.
//! \param nV    [in] number of knots in V dimension.
//! \param p     [in] degree in U dimension.
//! \param q     [in] degree in V dimension.
mobius::geom_BSplineSurface::geom_BSplineSurface(const std::vector< std::vector<xyz> >& Poles,
                                                 const double*                          U,
                                                 const double*                          V,
                                                 const int                              nU,
                                                 const int                              nV,
                                                 const int                              p,
                                                 const int                              q)
: geom_Surface()
{
  std::vector<double> Uvec;
  for ( size_t i = 0; i < nU; ++i )
    Uvec.push_back(U[i]);

  std::vector<double> Vvec;
  for ( size_t i = 0; i < nV; ++i )
    Vvec.push_back(V[i]);

  this->init(Poles, Uvec, Vvec, p, q);
}

//-----------------------------------------------------------------------------

//! Constructor.
//! \param Poles [in] poles for B-spline surface.
//! \param U     [in] knot vector in U dimension.
//! \param V     [in] knot vector in V dimension.
//! \param p     [in] degree in U dimension.
//! \param q     [in] degree in V dimension.
mobius::geom_BSplineSurface::geom_BSplineSurface(const std::vector< std::vector<xyz> >& Poles,
                                                 const std::vector<double>&             U,
                                                 const std::vector<double>&             V,
                                                 const int                              p,
                                                 const int                              q)
: geom_Surface()
{
  this->init(Poles, U, V, p, q);
}

//-----------------------------------------------------------------------------

//! Destructor.
mobius::geom_BSplineSurface::~geom_BSplineSurface()
{}

//-----------------------------------------------------------------------------

//! Constructs B-surface from JSON.
//! \param[in] json JSON string to create a surface from.
//! \return constructed B-surface or null if JSON is of invalid format.
mobius::core_Ptr<mobius::geom_BSplineSurface>
  mobius::geom_BSplineSurface::Instance(const std::string& json)
{
  core_Ptr<bsurf> result;
  if ( !geom_JSON(json).ExtractBSurface(result) )
    return NULL;

  return result;
}

//-----------------------------------------------------------------------------

//! Dumps the surface data to string stream.
//! \param stream [in/out] target stream.
void mobius::geom_BSplineSurface::Dump(std::stringstream& stream) const
{
  stream << "B-surface with the following properties:\n"
         << "\t U degree (p) = " << m_iDegU << "\n"
         << "\t V degree (q) = " << m_iDegV << "\n";
}

//-----------------------------------------------------------------------------

//! Calculates boundary box for the B-spline surface by its control polygon.
//! Notice that this peculiarity can look weird as control polygon only
//! outlines the B-spline surface, but does not follow its exact shape.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_BSplineSurface::Bounds(double& xMin, double& xMax,
                                         double& yMin, double& yMax,
                                         double& zMin, double& zMax) const
{
  double x_min = DBL_MAX, x_max = -DBL_MAX;
  double y_min = DBL_MAX, y_max = -DBL_MAX;
  double z_min = DBL_MAX, z_max = -DBL_MAX;

  // B-spline surface is fully contained in its control grid, so we can take
  // it as a rough solution
  for ( int i = 0; i < (int) m_poles.size(); ++i )
  {
    const std::vector<xyz>& uLine = m_poles.at(i);
    for ( int j = 0; j < (int) uLine.size(); ++j )
    {
      const xyz& P = uLine.at(j);
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

//! Returns first knot in U dimension.
//! \return first knot.
double mobius::geom_BSplineSurface::MinParameter_U() const
{
  return m_U[0];
}

//-----------------------------------------------------------------------------

//! Returns last knot in U dimension.
//! \return last knot.
double mobius::geom_BSplineSurface::MaxParameter_U() const
{
  return m_U[m_U.size()-1];
}

//-----------------------------------------------------------------------------

//! Returns first knot in V dimension.
//! \return first knot.
double mobius::geom_BSplineSurface::MinParameter_V() const
{
  return m_V[0];
}

//-----------------------------------------------------------------------------

//! Returns last knot in V dimension.
//! \return last knot.
double mobius::geom_BSplineSurface::MaxParameter_V() const
{
  return m_V[m_V.size()-1];
}

//-----------------------------------------------------------------------------

//! Evaluates B-spline surface for the given pair of (u, v) parameters.
//! This algorithm is essentially the algorithm A3.5 from The NURBS Book.
//!
//! \param u [in]  U parameter value to evaluate surface for.
//! \param v [in]  V parameter value to evaluate surface for.
//! \param S [out] 3D point corresponding to the given parameter pair.
void mobius::geom_BSplineSurface::Eval(const double u,
                                       const double v,
                                       xyz&         S) const
{
  // Find spans the passed u and v fall into
  bspl_FindSpan FindSpanU(m_U, m_iDegU);
  bspl_FindSpan FindSpanV(m_V, m_iDegV);
  //
  const int span_u = FindSpanU(u);
  const int span_v = FindSpanV(v);

  //---------------------------------------------
  // Evaluate effective B-spline basis functions
  //---------------------------------------------

  bspl_EffectiveN EffectiveN;
  //
  double N_u[mobiusBSpl_MaxDegree];
  double N_v[mobiusBSpl_MaxDegree];
  //
  EffectiveN(u, m_U, m_iDegU, span_u, N_u);
  EffectiveN(v, m_V, m_iDegV, span_v, N_v);

  //---------------------------
  // Evaluate B-spline surface
  //---------------------------

  const int u_first_idx = span_u - m_iDegU;
  const int v_first_idx = span_v - m_iDegV;

  xyz Res;
  for ( int i = 0; i <= m_iDegU; ++i )
  {
    xyz temp;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const xyz& P_ij = m_poles.at(u_first_idx + i).at(v_first_idx + j);
      temp += P_ij*N_v[j];
    }
    Res += temp*N_u[i];
  }

  // Set output argument
  S = Res;
}

//-----------------------------------------------------------------------------

void mobius::geom_BSplineSurface::Eval_D1(const double u,
                                          const double v,
                                          xyz&         S,
                                          xyz&         dU,
                                          xyz&         dV) const
{
  // Find spans the passed u and v fall into
  bspl_FindSpan FindSpanU(m_U, m_iDegU);
  bspl_FindSpan FindSpanV(m_V, m_iDegV);
  //
  const int span_u = FindSpanU(u);
  const int span_v = FindSpanV(v);

  ptr<alloc2d> localAlloc = new alloc2d;
  //
  double** dNu = localAlloc->Allocate(2, m_iDegU + 1, true);
  double** dNv = localAlloc->Allocate(2, m_iDegV + 1, true);

  // Evaluate derivatives of B-spline basis functions
  bspl_EffectiveNDers NDers(NULL, -1);
  NDers(u, m_U, m_iDegU, span_u, 1, dNu);
  NDers(v, m_V, m_iDegV, span_v, 1, dNv);

  //---------------------------
  // Evaluate B-spline surface
  //---------------------------

  const int u_first_idx = span_u - m_iDegU;
  const int v_first_idx = span_v - m_iDegV;

  xyz res_S, res_dU, res_dV;
  for ( int i = 0; i <= m_iDegU; ++i )
  {
    xyz temp_S, temp_dU, temp_dV;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const xyz& P_ij = m_poles.at(u_first_idx + i).at(v_first_idx + j);
      temp_S  += P_ij * dNv[0][j]; // Primal (0 index)
      temp_dU += P_ij * dNv[0][j]; // Primal (0 index)
      temp_dV += P_ij * dNv[1][j];
    }
    res_S  += temp_S  * dNu[0][i]; // Primal (0 index)
    res_dU += temp_dU * dNu[1][i];
    res_dV += temp_dV * dNu[0][i]; // Primal (0 index)
  }

  // Set output arguments
  S  = res_S;
  dU = res_dU;
  dV = res_dV;
}

//-----------------------------------------------------------------------------

void mobius::geom_BSplineSurface::Eval_D2(const double u,
                                          const double v,
                                          xyz&         S,
                                          xyz&         dU,
                                          xyz&         dV,
                                          xyz&         d2U,
                                          xyz&         d2V,
                                          xyz&         d2UV,
                                          ptr<alloc2d> alloc,
                                          const int    memBlockResultU,
                                          const int    memBlockResultV,
                                          const int    memBlockInternal) const
{
  ptr<alloc2d> localAlloc;

  double** dNu, **dNv;
  if ( alloc.IsNull() )
  {
    localAlloc = new alloc2d;
    dNu = localAlloc->Allocate(3, m_iDegU + 1, true);
    dNv = localAlloc->Allocate(3, m_iDegV + 1, true);
  }
  else
  {
    dNu = alloc->Access(memBlockResultU).Ptr;
    dNv = alloc->Access(memBlockResultV).Ptr;
  }

  // Find spans the passed u and v fall into
  bspl_FindSpan FindSpanU(m_U, m_iDegU);
  bspl_FindSpan FindSpanV(m_V, m_iDegV);
  //
  const int span_u = FindSpanU(u);
  const int span_v = FindSpanV(v);

  // Evaluate derivatives of B-spline basis functions
  bspl_EffectiveNDers NDers(alloc, memBlockInternal);
  NDers(u, m_U, m_iDegU, span_u, 2, dNu);
  NDers(v, m_V, m_iDegV, span_v, 2, dNv);

  //---------------------------
  // Evaluate B-spline surface
  //---------------------------

  const int u_first_idx = span_u - m_iDegU;
  const int v_first_idx = span_v - m_iDegV;

  xyz res_S, res_dU, res_dV, res_d2U, res_d2V, res_d2UV;
  for ( int i = 0; i <= m_iDegU; ++i )
  {
    xyz temp_S, temp_dU, temp_dV, temp_d2U, temp_d2V, temp_d2UV;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const xyz& P_ij = m_poles.at(u_first_idx + i).at(v_first_idx + j);
      temp_S    += P_ij * dNv[0][j]; // Primal (0 index)
      temp_dU   += P_ij * dNv[0][j]; // Primal (0 index)
      temp_dV   += P_ij * dNv[1][j];
      temp_d2U  += P_ij * dNv[0][j]; // Primal (0 index)
      temp_d2V  += P_ij * dNv[2][j];
      temp_d2UV += P_ij * dNv[1][j];
    }
    res_S    += temp_S    * dNu[0][i]; // Primal (0 index)
    res_dU   += temp_dU   * dNu[1][i];
    res_dV   += temp_dV   * dNu[0][i]; // Primal (0 index)
    res_d2U  += temp_d2U  * dNu[2][i];
    res_d2V  += temp_d2V  * dNu[0][i]; // Primal (0 index)
    res_d2UV += temp_d2UV * dNu[1][i];
  }

  // Set output arguments
  S    = res_S;
  dU   = res_dU;
  dV   = res_dV;
  d2U  = res_d2U;
  d2V  = res_d2V;
  d2UV = res_d2UV;
}

//-----------------------------------------------------------------------------

//! Creates a copy of this B-curve.
//! \return copy of B-curve.
mobius::ptr<mobius::bsurf> mobius::geom_BSplineSurface::Copy() const
{
  return new bsurf(m_poles, m_U, m_V, m_iDegU, m_iDegV);
}

//-----------------------------------------------------------------------------

//! Extracts isoparametric curve corresponding to the passed {u} level.
//! \param u [in] parameter value to extract isoparametric curve for.
//! \return isoline.
mobius::ptr<mobius::bcurve>
  mobius::geom_BSplineSurface::Iso_U(const double u) const
{
  // Heap allocator
  core_HeapAlloc<double> Alloc;

  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_U, m_iDegU);
  const int span_u = FindSpan(u);

  //--------------------------------------------------
  // Calculate control points for isoparametric curve
  //--------------------------------------------------

  // Evaluate effective B-spline basis functions
  bspl_EffectiveN EffectiveN;
  double* N_u = Alloc.Allocate(m_iDegU + 1, true);
  EffectiveN(u, m_U, m_iDegU, span_u, N_u);
  const int u_first_idx = span_u - m_iDegU;

  // Calculate new poles
  std::vector<xyz> Q;
  for ( int j = 0; j < (int) m_poles[0].size(); ++j )
  {
    xyz Q_j;
    for ( int i = 0; i <= m_iDegU; ++i )
    {
      const xyz& P_ij = m_poles[u_first_idx + i][j];
      Q_j += P_ij*N_u[i];
    }
    Q.push_back(Q_j);
  }

  //-----------------------
  // Create B-spline curve
  //-----------------------

  ptr<bcurve> Iso = new bcurve(Q, m_V, m_iDegV);
  return Iso;
}

//-----------------------------------------------------------------------------

//! Extracts isoparametric curve corresponding to the passed {v} level.
//! \param v [in] parameter value to extract isoparametric curve for.
//! \return isoline.
mobius::ptr<mobius::bcurve>
  mobius::geom_BSplineSurface::Iso_V(const double v) const
{
  // Heap allocator
  core_HeapAlloc<double> Alloc;

  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_V, m_iDegV);
  const int span_v = FindSpan(v);

  //--------------------------------------------------
  // Calculate control points for isoparametric curve
  //--------------------------------------------------

  // Evaluate effective B-spline basis functions
  bspl_EffectiveN EffectiveN;
  double* N_v = Alloc.Allocate(m_iDegV + 1, true);
  EffectiveN(v, m_V, m_iDegV, span_v, N_v);
  const int v_first_idx = span_v - m_iDegV;

  // Calculate new poles
  std::vector<xyz> Q;
  for ( int i = 0; i < (int) m_poles.size(); ++i )
  {
    xyz Q_i;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const xyz& P_ij = m_poles[i][v_first_idx + j];
      Q_i += P_ij*N_v[j];
    }
    Q.push_back(Q_i);
  }

  //-----------------------
  // Create B-spline curve
  //-----------------------

  ptr<bcurve> Iso = new bcurve(Q, m_U, m_iDegU);
  return Iso;
}

//-----------------------------------------------------------------------------

double mobius::geom_BSplineSurface::ComputeBendingEnergy() const
{
  geom_ThinPlateEnergies func(this);

  // (2n-1) for max accuracy on polynomial functions.
  const int NUM_GAUSS_PT_U = 2*m_iDegU - 1;
  const int NUM_GAUSS_PT_V = 2*m_iDegV - 1;

  // Integrate in each span individually for better accuracy.
  double result = 0;
  for ( size_t i = 0; i < m_U.size() - 1; ++i )
  {
    if ( m_U[i] == m_U[i+1] ) continue; // Skip multiple knots.

    for ( size_t j = 0; j < m_V.size() - 1; ++j )
    {
      if ( m_V[j] == m_V[j+1] ) continue; // Skip multiple knots.

      // 6-points integration in each knot span.
      const double
        gaussVal = core_Integral::gauss::Compute(&func,
                                                 m_U[i], m_U[i+1],
                                                 m_V[j], m_V[j+1],
                                                 NUM_GAUSS_PT_U, NUM_GAUSS_PT_V);
      //
      result += gaussVal;
    }
  }

  return result;
}

//-----------------------------------------------------------------------------

//! Initializes B-spline surface with complete data.
//! \param Poles [in] control points.
//! \param U     [in] knot vector in U dimension.
//! \param V     [in] knot vector in V dimension.
//! \param p     [in] degree of the B-spline basis functions in U dimension.
//! \param q     [in] degree of the B-spline basis functions in V dimension.
void mobius::geom_BSplineSurface::init(const std::vector< std::vector<xyz> >& Poles,
                                       const std::vector<double>&             U,
                                       const std::vector<double>&             V,
                                       const int                              p,
                                       const int                              q)
{
  // Check degrees.
  if ( p > mobiusBSpl_MaxDegree || q > mobiusBSpl_MaxDegree )
    throw bspl_excMaxDegreeViolation();

  // Check if B-surface can be constructed.
  if ( !bspl::Check( int( Poles.size() ) - 1, int( U.size() ) - 1, p ) )
    throw geom_excBSurfaceCtor();
  //
  if ( !bspl::Check( int( Poles[0].size() ) - 1, int( V.size() ) - 1, q ) )
    throw geom_excBSurfaceCtor();

  m_poles = Poles;
  m_U     = U;
  m_V     = V;
  m_iDegU = p;
  m_iDegV = q;
}
