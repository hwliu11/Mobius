//-----------------------------------------------------------------------------
// Created on: 15 December 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

#ifndef geom_CoonsSurfaceCubic_HeaderFile
#define geom_CoonsSurfaceCubic_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Four-sided cubic Coons patch.
class geom_CoonsSurfaceCubic : public geom_Surface
{
// Construction & destruction:
public:

  //! This enum allows to evaluate the Coons surface partially.
  enum EvalComponents
  {
    EvalComponent_All,
    EvalComponent_P1S,
    EvalComponent_P2S,
    EvalComponent_P12S
  };

  mobiusGeom_EXPORT
    geom_CoonsSurfaceCubic(const t_ptr<t_curve>& Su0,         // `s(u,0)` boundary curve.
                           const t_ptr<t_curve>& Su1,         // `s(u,1)` boundary curve.
                           const t_ptr<t_curve>& S0v,         // `s(0,v)` boundary curve.
                           const t_ptr<t_curve>& S1v,         // `s(1,v)` boundary curve.
                           const t_ptr<t_curve>& dSu0_dv,     // `s_v(u,0)` boundary derivative.
                           const t_ptr<t_curve>& dSu1_dv,     // `s_v(u,1)` boundary derivative.
                           const t_ptr<t_curve>& dS0v_du,     // `s_u(0,v)` boundary derivative.
                           const t_ptr<t_curve>& dS1v_du,     // `s_u(1,v)` boundary derivative.
                           const t_xyz&          d2S_dudv00,  // Twist vector at corner `(0,0)`.
                           const t_xyz&          d2S_dudv01,  // Twist vector at corner `(0,1)`.
                           const t_xyz&          d2S_dudv10,  // Twist vector at corner `(1,0)`.
                           const t_xyz&          d2S_dudv11); // Twist vector at corner `(1,1)`.

  mobiusGeom_EXPORT virtual
    ~geom_CoonsSurfaceCubic();

// Interface methods:
public:

  //! Calculates boundary box for the surface.
  //! \param xMin [out] min X.
  //! \param xMax [out] max X.
  //! \param yMin [out] min Y.
  //! \param yMax [out] max Y.
  //! \param zMin [out] min Z.
  //! \param zMax [out] max Z.
  mobiusGeom_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const override;

public:

  //! Returns minimal U parameter.
  //! \return parameter value.
  mobiusGeom_EXPORT virtual double
    GetMinParameter_U() const override;

  //! Returns maximal U parameter.
  //! \return parameter value.
  mobiusGeom_EXPORT virtual double
    GetMaxParameter_U() const override;

  //! Returns minimal V parameter.
  //! \return parameter value.
  mobiusGeom_EXPORT virtual double
    GetMinParameter_V() const override;

  //! Returns maximal V parameter.
  //! \return parameter value.
  mobiusGeom_EXPORT virtual double
    GetMaxParameter_V() const override;

  //! Evaluates surface in the given parametric point (u, v).
  //! \param[in]  u first parameter.
  //! \param[in]  v second parameter.
  //! \param[out] S evaluated spatial point S(u, v).
  mobiusGeom_EXPORT virtual void
    Eval(const double u,
         const double v,
         t_xyz&       S) const override;

public:

  mobiusGeom_EXPORT virtual void
    Eval_D1(const double u,
            const double v,
            t_xyz&       dU,
            t_xyz&       dV) const;

public:

  mobiusGeom_EXPORT void
    Eval_P1S(const double u,
             const double v,
             t_xyz&       S) const;

  mobiusGeom_EXPORT void
    Eval_P2S(const double u,
             const double v,
             t_xyz&       S) const;

  mobiusGeom_EXPORT void
    Eval_P12S(const double u,
              const double v,
              t_xyz&       S) const;

  mobiusGeom_EXPORT void
    Eval_D1_P1S(const double u,
                const double v,
                t_xyz&       dU,
                t_xyz&       dV) const;

  mobiusGeom_EXPORT void
    Eval_D1_P2S(const double u,
                const double v,
                t_xyz&       dU,
                t_xyz&       dV) const;

  mobiusGeom_EXPORT void
    Eval_D1_P12S(const double u,
                 const double v,
                 t_xyz&       dU,
                 t_xyz&       dV) const;

public:

  //! Sets component of the Boolean sum to evaluate.
  //! \param[in] comp component to evaluate.
  void SetEvalComponent(const EvalComponents comp)
  {
    m_comp = comp;
  }

protected:

  t_xyz evalS12Row(const int    row,
                   const double v) const;

private:

  /* Inputs */

  t_ptr<t_curve> m_Su0;        //!< Rail curve `s(u,0)`.
  t_ptr<t_curve> m_Su1;        //!< Rail curve `s(u,1)`.
  t_ptr<t_curve> m_S0v;        //!< Rail curve `s(0,v)`.
  t_ptr<t_curve> m_S1v;        //!< Rail curve `s(1,v)`.
  t_ptr<t_curve> m_dS0v_du;    //!< `s_u(0,v)` boundary derivative.
  t_ptr<t_curve> m_dS1v_du;    //!< `s_u(1,v)` boundary derivative.
  t_ptr<t_curve> m_dSu0_dv;    //!< `s_v(u,0)` boundary derivative.
  t_ptr<t_curve> m_dSu1_dv;    //!< `s_v(u,1)` boundary derivative.
  t_xyz          m_d2S_dudv00; //!< Derivative constraint at `(u,v) = (0,0)`.
  t_xyz          m_d2S_dudv01; //!< Derivative constraint at `(u,v) = (0,1)`.
  t_xyz          m_d2S_dudv10; //!< Derivative constraint at `(u,v) = (1,0)`.
  t_xyz          m_d2S_dudv11; //!< Derivative constraint at `(u,v) = (1,1)`.

  //! Component of the Boolean sum to evaluate.
  EvalComponents m_comp;

  /* Cached values */

  t_xyz m_S00; //!< Positional constraint at `(u,v) = (0,0)`.
  t_xyz m_S01; //!< Positional constraint at `(u,v) = (0,1)`.
  t_xyz m_S10; //!< Positional constraint at `(u,v) = (1,0)`.
  t_xyz m_S11; //!< Positional constraint at `(u,v) = (1,1)`.

  t_xyz m_dS00_du; //!< Derivative value `s_u(0,0)`.
  t_xyz m_dS01_du; //!< Derivative value `s_u(0,1)`.
  t_xyz m_dS10_du; //!< Derivative value `s_u(1,0)`.
  t_xyz m_dS11_du; //!< Derivative value `s_u(1,1)`.

  t_xyz m_dS00_dv; //!< Derivative value `s_v(0,0)`.
  t_xyz m_dS01_dv; //!< Derivative value `s_v(0,1)`.
  t_xyz m_dS10_dv; //!< Derivative value `s_v(1,0)`.
  t_xyz m_dS11_dv; //!< Derivative value `s_v(1,1)`.

};

};

#endif
