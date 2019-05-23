//-----------------------------------------------------------------------------
// Created on: 23 May 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

#ifndef geom_PlaneSurface_HeaderFile
#define geom_PlaneSurface_HeaderFile

// Geometry includes
#include <mobius/geom_Line.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Planar surface having the following parameterization:
//!
//! <pre>
//!    x = O_x + D1_x u + D2_x v;
//!    y = O_y + D1_y u + D2_y v;
//!    z = O_z + D1_z u + D2_z v;
//! </pre>
//!
//! Both parameters are not limited for an inifinite plane. At the same time
//! the plane may store additional limits for `u` and `v` as in practice we
//! rarely (if ever) deal with infinite objects.
class geom_PlaneSurface : public geom_Surface
{
// Construction & destruction:
public:

  //! Ctor.
  mobiusGeom_EXPORT
    geom_PlaneSurface();

  //! Complete ctor.
  //! \param[in] O  origin.
  //! \param[in] D1 first direction.
  //! \param[in] D2 second direction.
  mobiusGeom_EXPORT
    geom_PlaneSurface(const xyz& O,
                      const xyz& D1,
                      const xyz& D2);

  //! Dtor.
  mobiusGeom_EXPORT virtual
    ~geom_PlaneSurface();

// Interface methods:
public:

  //! Calculates boundary box for the surface.
  //! \param[out] xMin min X.
  //! \param[out] xMax max X.
  //! \param[out] yMin min Y.
  //! \param[out] yMax max Y.
  //! \param[out] zMin min Z.
  //! \param[out] zMax max Z.
  mobiusGeom_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const override;

public:

  //! \return min U parameter value.
  mobiusGeom_EXPORT virtual double
    GetMinParameter_U() const override;

  //! \return max U parameter value.
  mobiusGeom_EXPORT virtual double
    GetMaxParameter_U() const override;

  //! \return min V parameter value.
  mobiusGeom_EXPORT virtual double
    GetMinParameter_V() const override;

  //! \return max V parameter value.
  mobiusGeom_EXPORT virtual double
    GetMaxParameter_V() const override;

  //! Evaluates surface in the given parametric point `(u,v)`.
  //! \param[in]  u first parameter.
  //! \param[in]  v second parameter.
  //! \param[out] S evaluated spatial point `S = S(u,v)`.
  mobiusGeom_EXPORT virtual void
    Eval(const double u,
         const double v,
         xyz&         S) const override;

  //! Returns isoparametric curve for a fixed `u`.
  //! \param[in] u fixed parameter.
  //! \return isoparametric curve.
  mobiusGeom_EXPORT ptr<geom_Line>
    Iso_U(const double u) const;

  //! Returns isoparametric curve for a fixed `v`.
  //! \param[in] v fixed parameter.
  //! \return isoparametric curve.
  mobiusGeom_EXPORT ptr<geom_Line>
    Iso_V(const double v) const;

public:

  //! Sets the artificial limits for the parametric directions.
  //! \param[in] uMin min U.
  //! \param[in] uMax max U.
  //! \param[in] vMin min V.
  //! \param[in] vMax max V.
  void SetLimits(const double uMin,
                 const double uMax,
                 const double vMin,
                 const double vMax)
  {
    m_fUMin = uMin;
    m_fUMax = uMax;
    m_fVMin = vMin;
    m_fVMax = vMax;
  }

  //! Returns the artificial limits for the parametric directions.
  //! \param[out] uMin min U.
  //! \param[out] uMax max U.
  //! \param[out] vMin min V.
  //! \param[out] vMax max V.
  void GetLimits(double& uMin,
                 double& uMax,
                 double& vMin,
                 double& vMax) const
  {
    uMin = m_fUMin;
    uMax = m_fUMax;
    vMin = m_fVMin;
    vMax = m_fVMax;
  }

  //! Accessor for the origin of a plane.
  //! \return origin `O`.
  const xyz& GetOrigin() const
  {
    return m_origin;
  }

  //! Accessor for the first vector of a plane.
  //! \return vector `D1`.
  const xyz& GetD1() const
  {
    return m_D1;
  }

  //! Accessor for the second vector of a plane.
  //! \return vector `D2`.
  const xyz& GetD2() const
  {
    return m_D2;
  }

protected:

  xyz m_origin; //!< Origin of a plane.
  xyz m_D1;     //!< Vector D1.
  xyz m_D2;     //!< Vector D2.

  double m_fUMin; //!< Min value of U.
  double m_fUMax; //!< Max value of U.
  double m_fVMin; //!< Min value of V.
  double m_fVMax; //!< Max value of V.

};

//! Convenience shortcut.
typedef geom_PlaneSurface plane;

};

#endif
