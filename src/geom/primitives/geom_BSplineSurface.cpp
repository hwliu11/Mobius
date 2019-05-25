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
#include <mobius/core_Newton2x2.h>

// BSpl includes
#include <mobius/bspl_EffectiveN.h>
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>
#include <mobius/bspl_InsKnot.h>

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
  geom_ThinPlateEnergies(const t_ptr<geom_BSplineSurface>& surface) : core_TwovariateFunc()
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
    t_xyz P, dU, dV, d2U, d2V, d2UV;
    m_surface->Eval_D2(u, v, P, dU, dV, d2U, d2V, d2UV);

    const double E = d2U.Dot(d2U) + 2*d2UV.Dot(d2UV) + d2V.Dot(d2V);
    return E;
  }

public:

  //! \return surface in question.
  const t_ptr<geom_BSplineSurface>& GetSurface() const
  {
    return m_surface;
  }

protected:

  t_ptr<geom_BSplineSurface> m_surface; //!< Surface.

};

//! Auxiliary functions for point inversion on surface.
namespace BSplSurfProj
{
  class func_base : public core_TwovariateFuncWithGradient
  {
  public:

    //! Ctor accepting the B-surface and the point to invert.
    //! \param[in] S surface in question.
    //! \param[in] P point to invert.
    func_base(const core_Ptr<t_bsurf>& S,
              const t_xyz&             P)
    : core_TwovariateFuncWithGradient(), m_S(S), m_P(P)
    {}

  protected:

    //! Evaluates residual between `S(u,v)` and point to invert `P`.
    t_xyz eval_r(const double u, const double v) const
    {
      t_xyz S;
      m_S->Eval(u, v, S);
      return S - m_P;
    }

  protected:

    core_Ptr<t_bsurf> m_S; //!< Surface in question.
    t_xyz             m_P; //!< Point to invert.
  };

  class func_f : public func_base
  {
  public:

    //! Ctor accepting the B-surface and the point to invert.
    //! \param[in] S surface in question.
    //! \param[in] P point to invert.
    func_f(const core_Ptr<t_bsurf>& S,
           const t_xyz&             P)
    : func_base(S, P)
    {}

  public:

    //! Evaluates function.
    //! \param[in] u u argument.
    //! \param[in] v v argument.
    //! \return function value.
    virtual double
      Eval(const double u, const double v) const
    {
      const t_xyz r = this->eval_r(u, v);

      // Evaluate dS/dU.
      t_xyz S, dS_dU, dS_dV;
      m_S->Eval_D1(u, v, S, dS_dU, dS_dV);

      // Evaluate dot product.
      const double res = r.Dot(dS_dU);
      return res;
    }

    //! Evaluates function with its gradient.
    //! \param[in]  u     u argument.
    //! \param[in]  v     v argument.
    //! \param[out] F     function value.
    //! \param[out] dF_du first-order partial derivative by u.
    //! \param[out] dF_dv first-order partial derivative by v.
    virtual void
      EvalWithGrad(const double u,
                   const double v,
                   double&      F,
                   double&      dF_du,
                   double&      dF_dv) const
    {
      const t_xyz r = this->eval_r(u, v);

      // Evaluate dS/dU.
      t_xyz S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV;
      m_S->Eval_D2(u, v, S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV);

      // Evaluate F.
      F = r.Dot(dS_dU);

      // Evaluate dF/dU.
      dF_du = dS_dU.Dot(dS_dU) + r.Dot(d2S_dU2);

      // Evaluate dF/dV.
      dF_dv = dS_dU.Dot(dS_dV) + r.Dot(d2S_dUV);
    }
  };

  class func_g : public func_base
  {
  public:

    //! Ctor accepting the B-surface and the point to invert.
    //! \param[in] S surface in question.
    //! \param[in] P point to invert.
    func_g(const core_Ptr<t_bsurf>& S,
           const t_xyz&             P)
    : func_base(S, P)
    {}

  public:

    //! Evaluates function.
    //! \param[in] u u argument.
    //! \param[in] v v argument.
    //! \return function value.
    virtual double
      Eval(const double u, const double v) const
    {
      const t_xyz r = this->eval_r(u, v);

      // Evaluate dS/dU.
      t_xyz S, dS_dU, dS_dV;
      m_S->Eval_D1(u, v, S, dS_dU, dS_dV);

      // Evaluate dot product.
      const double res = r.Dot(dS_dV);
      return res;
    }

    //! Evaluates function with its gradient.
    //! \param[in]  u     u argument.
    //! \param[in]  v     v argument.
    //! \param[out] G     function value.
    //! \param[out] dG_du first-order partial derivative by u.
    //! \param[out] dG_dv first-order partial derivative by v.
    virtual void
      EvalWithGrad(const double u,
                   const double v,
                   double&      G,
                   double&      dG_du,
                   double&      dG_dv) const
    {
      const t_xyz r = this->eval_r(u, v);

      // Evaluate dS/dU.
      t_xyz S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV;
      m_S->Eval_D2(u, v, S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV);

      // Evaluate F.
      G = r.Dot(dS_dV);

      // Evaluate dG/dU.
      dG_du = dS_dU.Dot(dS_dV) + r.Dot(d2S_dUV);

      // Evaluate dG/dV.
      dG_dv = dS_dV.Dot(dS_dV) + r.Dot(d2S_dV2);
    }
  };

  //! Newton method specialization for point inversion.
  class Newton2x2 : public core_Newton2x2
  {
  public:

    //! Ctor.
    //! \param[in] S        surface in question.
    //! \param[in] f        first component of the objective vector function.
    //! \param[in] g        second component of the objective vector function.
    //! \param[in] umin     min value of the first argument.
    //! \param[in] umax     max value of the first argument.
    //! \param[in] vmin     min value of the second argument.
    //! \param[in] vmax     max value of the second argument.
    //! \param[in] progress progress notifier.
    //! \param[in] plotter  imperative plotter.
    Newton2x2(const core_Ptr<geom_BSplineSurface>&             S,
              const core_Ptr<core_TwovariateFuncWithGradient>& f,
              const core_Ptr<core_TwovariateFuncWithGradient>& g,
              const double                                     umin,
              const double                                     umax,
              const double                                     vmin,
              const double                                     vmax,
              core_ProgressEntry                               progress = NULL,
              core_PlotterEntry                                plotter  = NULL)
    //
    : core_Newton2x2 (f, g, umin, umax, vmin, vmax, progress, plotter),
      m_S            (S),
      m_P            (DBL_MAX, 0., 0.)
    {}

  private:

    //! Callback on choosing the next point.
    //! \param[in] u next u.
    //! \param[in] v next v.
    virtual void onNextEvaluation(const double u, const double v)
    {
      // Draw 3D point.
      t_xyz P;
      m_S->Eval(u, v, P);
      //
      m_plotter.DRAW_POINT(P, MobiusColor_Magenta, "onNextStep");

      if ( m_P.X() != DBL_MAX )
        m_plotter.DRAW_LINK(m_P, P, MobiusColor_Magenta, "onNextStep");
      //
      m_P = P;
    }

  private:

    core_Ptr<geom_BSplineSurface> m_S; //!< Surface in question.
    t_xyz                         m_P; //!< Last evaluated point.
  };
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
mobius::geom_BSplineSurface::geom_BSplineSurface(const std::vector< std::vector<t_xyz> >& Poles,
                                                 const double*                            U,
                                                 const double*                            V,
                                                 const int                                nU,
                                                 const int                                nV,
                                                 const int                                p,
                                                 const int                                q)
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
mobius::geom_BSplineSurface::geom_BSplineSurface(const std::vector< std::vector<t_xyz> >& Poles,
                                                 const std::vector<double>&               U,
                                                 const std::vector<double>&               V,
                                                 const int                                p,
                                                 const int                                q)
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
  core_Ptr<t_bsurf> result;
  if ( !geom_JSON(json).ExtractBSurface(result) )
    return NULL;

  return result;
}

//-----------------------------------------------------------------------------

bool mobius::geom_BSplineSurface::Compare(const t_ptr<t_bsurf>& F,
                                          const t_ptr<t_bsurf>& G,
                                          const double          tol2d,
                                          const double          tol3d)
{
  // Compare degrees.
  if ( F->GetDegree_U() != G->GetDegree_U() ||
       F->GetDegree_V() != G->GetDegree_V() )
    return false;

  // Compare number of poles.
  if ( F->GetNumOfPoles_U() != G->GetNumOfPoles_U() ||
       F->GetNumOfPoles_V() != G->GetNumOfPoles_V() )
    return false;

  // Compare number of knots.
  if ( F->GetNumOfKnots_U() != G->GetNumOfKnots_U() ||
       F->GetNumOfKnots_V() != G->GetNumOfKnots_V() )
    return false;

  const int numKnotsU = F->GetNumOfKnots_U();
  const int numKnotsV = F->GetNumOfKnots_V();
  const int numPolesU = F->GetNumOfPoles_U();
  const int numPolesV = F->GetNumOfPoles_V();

  // Compare knots.
  for ( int ii = 0; ii < numKnotsU; ++ii )
  {
    if ( fabs( F->GetKnot_U(ii) - G->GetKnot_U(ii) ) > tol2d )
      return false;
  }
  //
  for ( int ii = 0; ii < numKnotsV; ++ii )
  {
    if ( fabs( F->GetKnot_V(ii) - G->GetKnot_V(ii) ) > tol2d )
      return false;
  }

  // Compare poles.
  for ( int i = 0; i < numPolesU; ++i )
    for ( int j = 0; j < numPolesV; ++j )
    {
      const t_xyz& P = F->GetPole(i, j);
      const t_xyz& Q = G->GetPole(i, j);
      const double d = (P - Q).Modulus();

      if ( d > tol3d )
        return false;
    }

  return true;
}

//-----------------------------------------------------------------------------

//! Dumps the surface data to string stream.
//! \param stream [in,out] target stream.
void mobius::geom_BSplineSurface::Dump(std::ostream* out) const
{
  *out << "B-surface with the following properties:\n"
       << "\t U degree (p) = " << m_iDegU << "\n"
       << "\t V degree (q) = " << m_iDegV << "\n";
}

//-----------------------------------------------------------------------------

//! Dumps this B-surface as JSON object.
//! \return JSON string.
std::string mobius::geom_BSplineSurface::DumpJSON() const
{
  geom_JSON dumper;
  dumper.DumpBSurface(this);

  std::string res = dumper.GetJSON();

  return res;
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
void mobius::geom_BSplineSurface::GetBounds(double& xMin, double& xMax,
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
    const std::vector<t_xyz>& uLine = m_poles.at(i);
    for ( int j = 0; j < (int) uLine.size(); ++j )
    {
      const t_xyz& P = uLine.at(j);
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
double mobius::geom_BSplineSurface::GetMinParameter_U() const
{
  return m_U[0];
}

//-----------------------------------------------------------------------------

//! Returns last knot in U dimension.
//! \return last knot.
double mobius::geom_BSplineSurface::GetMaxParameter_U() const
{
  return m_U[m_U.size()-1];
}

//-----------------------------------------------------------------------------

//! Returns first knot in V dimension.
//! \return first knot.
double mobius::geom_BSplineSurface::GetMinParameter_V() const
{
  return m_V[0];
}

//-----------------------------------------------------------------------------

//! Returns last knot in V dimension.
//! \return last knot.
double mobius::geom_BSplineSurface::GetMaxParameter_V() const
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
                                       t_xyz&       S) const
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

  t_xyz Res;
  for ( int i = 0; i <= m_iDegU; ++i )
  {
    t_xyz temp;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const t_xyz& P_ij = m_poles.at(u_first_idx + i).at(v_first_idx + j);
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
                                          t_xyz&       S,
                                          t_xyz&       dU,
                                          t_xyz&       dV) const
{
  // Find spans the passed u and v fall into
  bspl_FindSpan FindSpanU(m_U, m_iDegU);
  bspl_FindSpan FindSpanV(m_V, m_iDegV);
  //
  const int span_u = FindSpanU(u);
  const int span_v = FindSpanV(v);

  t_ptr<t_alloc2d> localAlloc = new t_alloc2d;
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

  t_xyz res_S, res_dU, res_dV;
  for ( int i = 0; i <= m_iDegU; ++i )
  {
    t_xyz temp_S, temp_dU, temp_dV;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const t_xyz& P_ij = m_poles.at(u_first_idx + i).at(v_first_idx + j);
      //
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

void mobius::geom_BSplineSurface::Eval_D2(const double     u,
                                          const double     v,
                                          t_xyz&           S,
                                          t_xyz&           dU,
                                          t_xyz&           dV,
                                          t_xyz&           d2U,
                                          t_xyz&           d2V,
                                          t_xyz&           d2UV,
                                          t_ptr<t_alloc2d> alloc,
                                          const int        memBlockResultU,
                                          const int        memBlockResultV,
                                          const int        memBlockInternal) const
{
  t_ptr<t_alloc2d> localAlloc;

  double** dNu, **dNv;
  if ( alloc.IsNull() )
  {
    localAlloc = new t_alloc2d;
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

  t_xyz res_S, res_dU, res_dV, res_d2U, res_d2V, res_d2UV;
  for ( int i = 0; i <= m_iDegU; ++i )
  {
    t_xyz temp_S, temp_dU, temp_dV, temp_d2U, temp_d2V, temp_d2UV;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const t_xyz& P_ij = m_poles.at(u_first_idx + i).at(v_first_idx + j);
      //
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

mobius::core_Continuity
  mobius::geom_BSplineSurface::GetContinuity() const
{
  // Get continuity in U direction.
  const core_Continuity
    uCont = geom_BSplineCurve::CheckContinuityByKnots(m_U, m_iDegU);

  // Get continuity in V direction.
  const core_Continuity
    vCont = geom_BSplineCurve::CheckContinuityByKnots(m_V, m_iDegV);

  return min(uCont, vCont);
}

//-----------------------------------------------------------------------------

//! Creates a copy of this B-curve.
//! \return copy of B-curve.
mobius::t_ptr<mobius::t_bsurf> mobius::geom_BSplineSurface::Copy() const
{
  return new t_bsurf(m_poles, m_U, m_V, m_iDegU, m_iDegV);
}

//-----------------------------------------------------------------------------

bool mobius::geom_BSplineSurface::InvertPoint(const t_xyz& P,
                                              t_uv&        params,
                                              const double prec) const
{
  // Create objective functions.
  core_Ptr<core_TwovariateFuncWithGradient> f = new BSplSurfProj::func_f(this, P);
  core_Ptr<core_TwovariateFuncWithGradient> g = new BSplSurfProj::func_g(this, P);

  // Get domain bounds to limit search.
  const double uMin = this->GetMinParameter_U();
  const double uMax = this->GetMaxParameter_U();
  const double vMin = this->GetMinParameter_V();
  const double vMax = this->GetMaxParameter_V();

  // Find the closest control point to P.
  double minD = DBL_MAX;
  int    closest_i = 0, closest_j = 0;
  //
  for ( int iFixedU = 0; iFixedU < int( m_poles.size() ); ++iFixedU )
  {
    for ( int iFixedV = 0; iFixedV < int( m_poles[0].size() ); ++iFixedV )
    {
      const double d = (m_poles[iFixedU][iFixedV] - P).Modulus();
      //
      if ( d < minD )
      {
        minD      = d;
        closest_i = iFixedU;
        closest_j = iFixedV;
      }
    }
  }

  // We can now take the subdomain [u_i, u_{i+p+1}) x [v_j, v_{j+q+1}) where
  // the only non-vanishing basis functions reside. Let's then take the
  // midpoint in this subdomain as the initial guess.
  //
  double usubMin = m_U[closest_i];
  double usubMax = m_U[closest_i + m_iDegU + 1];
  double vsubMin = m_V[closest_j];
  double vsubMax = m_V[closest_j + m_iDegV + 1];
  //
  t_uv initPt = ( t_uv(usubMin, vsubMin) + t_uv(usubMax, vsubMax) )*0.5;

  // Visual dump of the effective subdomain.
  if ( !m_plotter.GetPlotter().IsNull() )
  {
    m_plotter.REDRAW_POINT("initPt2d", initPt, MobiusColor_Magenta);

    // Draw 3D point.
    t_xyz initPt3d;
    this->Eval(initPt.U(), initPt.V(), initPt3d);
    //
    m_plotter.REDRAW_POINT("initPt3d", initPt3d, MobiusColor_Magenta);

    // Draw isos.
    t_ptr<t_bcurve> usubMinIso = this->Iso_U(usubMin);
    t_ptr<t_bcurve> usubMaxIso = this->Iso_U(usubMax);
    t_ptr<t_bcurve> vsubMinIso = this->Iso_V(vsubMin);
    t_ptr<t_bcurve> vsubMaxIso = this->Iso_V(vsubMax);
    //
    m_plotter.REDRAW_CURVE("usubMinIso", usubMinIso, MobiusColor_Magenta);
    m_plotter.REDRAW_CURVE("usubMaxIso", usubMaxIso, MobiusColor_Magenta);
    m_plotter.REDRAW_CURVE("vsubMinIso", vsubMinIso, MobiusColor_Magenta);
    m_plotter.REDRAW_CURVE("vsubMaxIso", vsubMaxIso, MobiusColor_Magenta);
  }

  // Prepare Newton iterations.
  BSplSurfProj::Newton2x2 newton(this, f, g, uMin, uMax, vMin, vMax,
                                 m_progress, m_plotter);

  // Run optimizer.
  t_uv res;
  //
  if ( !newton.Perform(initPt, prec, res) )
    return false;

  params = res;
  return true;
}

//-----------------------------------------------------------------------------

//! Inserts knot in U parametric direction of the B-surface.
//! \param[in] u         knot value to insert.
//! \param[in] num_times how many times to insert.
//! \return true in case of success, false -- otherwise.
bool
  mobius::geom_BSplineSurface::InsertKnot_U(const double u,
                                            const int    num_times)
{
  // Get properties of the input surface.
  const std::vector<double>&               UP = this->GetKnots_U();
  const std::vector<double>&               VP = this->GetKnots_V();
  const int                                p  = this->GetDegree_U();
  const int                                q  = this->GetDegree_V();
  const std::vector< std::vector<t_xyz> >& Pw = this->GetPoles();
  const int                                np = int( Pw.size() ) - 1;
  const int                                mp = int( Pw[0].size() ) - 1;

  // Find span and resolve multiplicity.
  int k = -1, s = 0;
  bspl_FindSpan FindSpan(UP, p);
  k = FindSpan(u);

  // Multiplicity.
  for ( size_t i = 0; i < UP.size(); ++i )
    if ( UP[i] == u )
      s++;

  // Output arguments.
  int nq = 0, mq = 0;
  //
  std::vector<double> UQ, VQ;
  std::vector< std::vector<t_xyz> > Qw;

  // Perform knot insertion algorithm.
  bspl_InsKnot InsKnot;
  //
  try {
    if ( !InsKnot(np, p, UP, mp, q, VP, Pw, ParamDirection_U, u, k, s, num_times, nq, UQ, mq, VQ, Qw) )
      return false;
  }
  catch ( ... )
  {
    return false;
  }

  // Reinitialize.
  this->init(Qw, UQ, VQ, p, q);
  return true;
}

//-----------------------------------------------------------------------------

//! Inserts knot in V parametric direction of the B-surface.
//! \param[in] v         knot value to insert.
//! \param[in] num_times how many times to insert.
//! \return true in case of success, false -- otherwise.
bool
  mobius::geom_BSplineSurface::InsertKnot_V(const double v,
                                            const int    num_times)
{
  // Get properties of the input surface.
  const std::vector<double>&               UP = this->GetKnots_U();
  const std::vector<double>&               VP = this->GetKnots_V();
  const int                                p  = this->GetDegree_U();
  const int                                q  = this->GetDegree_V();
  const std::vector< std::vector<t_xyz> >& Pw = this->GetPoles();
  const int                                np = int( Pw.size() ) - 1;
  const int                                mp = int( Pw[0].size() ) - 1;

  // Find span and resolve multiplicity.
  int k = -1, s = 0;
  bspl_FindSpan FindSpan(VP, q);
  k = FindSpan(v);

  // Multiplicity.
  for ( size_t i = 0; i < VP.size(); ++i )
    if ( VP[i] == v )
      s++;

  // Output arguments.
  int nq = 0, mq = 0;
  //
  std::vector<double> UQ, VQ;
  std::vector< std::vector<t_xyz> > Qw;

  // Perform knot insertion algorithm.
  bspl_InsKnot InsKnot;
  //
  try {
    if ( !InsKnot(np, p, UP, mp, q, VP, Pw, ParamDirection_V, v, k, s, num_times, nq, UQ, mq, VQ, Qw) )
      return false;
  }
  catch ( ... )
  {
    return false;
  }

  // Reinitialize.
  this->init(Qw, UQ, VQ, p, q);
  return true;
}

//-----------------------------------------------------------------------------

//! Extracts isoparametric curve corresponding to the passed {u} level.
//! \param u [in] parameter value to extract isoparametric curve for.
//! \return isoline.
mobius::t_ptr<mobius::t_bcurve>
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
  std::vector<t_xyz> Q;
  for ( int j = 0; j < (int) m_poles[0].size(); ++j )
  {
    t_xyz Q_j;
    for ( int i = 0; i <= m_iDegU; ++i )
    {
      const t_xyz& P_ij = m_poles[u_first_idx + i][j];
      Q_j += P_ij*N_u[i];
    }
    Q.push_back(Q_j);
  }

  //-----------------------
  // Create B-spline curve
  //-----------------------

  t_ptr<t_bcurve> Iso = new t_bcurve(Q, m_V, m_iDegV);
  return Iso;
}

//-----------------------------------------------------------------------------

//! Extracts isoparametric curve corresponding to the passed {v} level.
//! \param v [in] parameter value to extract isoparametric curve for.
//! \return isoline.
mobius::t_ptr<mobius::t_bcurve>
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
  std::vector<t_xyz> Q;
  for ( int i = 0; i < (int) m_poles.size(); ++i )
  {
    t_xyz Q_i;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const t_xyz& P_ij = m_poles[i][v_first_idx + j];
      Q_i += P_ij*N_v[j];
    }
    Q.push_back(Q_i);
  }

  //-----------------------
  // Create B-spline curve
  //-----------------------

  t_ptr<t_bcurve> Iso = new t_bcurve(Q, m_U, m_iDegU);
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

void mobius::geom_BSplineSurface::ExchangeUV()
{
  // Transpose control net.
  std::vector< std::vector<t_xyz> > cNet;
  //
  for ( size_t col = 0; col < m_poles[0].size(); ++col )
  {
    std::vector<t_xyz> newRow;

    for ( size_t row = 0; row < m_poles.size(); ++row )
    {
      newRow.push_back( m_poles[row][col] );
    }

    cNet.push_back(newRow);
  }
  //
  m_poles = cNet;

  // Exchange knot vectors.
  std::vector<double> tmpKnots = m_U;
  m_U = m_V;
  m_V = tmpKnots;

  // Exchange degrees.
  int tmpDeg = m_iDegU;
  m_iDegU = m_iDegV;
  m_iDegV = tmpDeg;
}

//-----------------------------------------------------------------------------

void mobius::geom_BSplineSurface::init(const std::vector< std::vector<t_xyz> >& Poles,
                                       const std::vector<double>&               U,
                                       const std::vector<double>&               V,
                                       const int                                p,
                                       const int                                q)
{
  // Check degrees.
  if ( p > mobiusBSpl_MaxDegree || q > mobiusBSpl_MaxDegree )
    throw bspl_excMaxDegreeViolation();

  // Check relation between m, n and p.
  if ( !bspl::Check( int( Poles.size() ) - 1, int( U.size() ) - 1, p ) )
    throw geom_excBSurfaceCtor();
  //
  if ( !bspl::Check( int( Poles[0].size() ) - 1, int( V.size() ) - 1, q ) )
    throw geom_excBSurfaceCtor();

  // Check that the knot vectors are clamped.
  if ( !bspl::CheckClampedKnots(U, p) )
    throw geom_excBSurfaceCtor();
  //
  if ( !bspl::CheckClampedKnots(V, q) )
    throw geom_excBSurfaceCtor();

  m_poles = Poles;
  m_U     = U;
  m_V     = V;
  m_iDegU = p;
  m_iDegV = q;
}
