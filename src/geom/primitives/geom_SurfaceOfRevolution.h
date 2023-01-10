//-----------------------------------------------------------------------------
// Created on: 10 January 2023
//-----------------------------------------------------------------------------
// Copyright (c) 2023-present, Sergey Slyadnev
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

#ifndef geom_SurfaceOfRevolution_HeaderFile
#define geom_SurfaceOfRevolution_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>
#include <mobius/geom_Surface.h>

// Core includes
#include <mobius/core_Axis.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Surface of revolution.
class geom_SurfaceOfRevolution : public geom_Surface
{
// Construction & destruction:
public:

  //! Constructs a surface of revolution for the given generatrix
  //! to be revolved around the passed axis.
  //! \param[in] curve the generatrix curve.
  //! \param[in] axis  the axis of revolution.
  mobiusGeom_EXPORT
    geom_SurfaceOfRevolution(const t_ptr<t_curve>& curve,
                             const t_axis&         axis);

  mobiusGeom_EXPORT virtual
    ~geom_SurfaceOfRevolution();

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
         core_XYZ&    S) const override;

private:

  t_ptr<t_curve> m_c;  //!< Generatrix curve.
  t_axis         m_ax; //!< Axis of revolution.

};

typedef geom_SurfaceOfRevolution t_surfRevol;

}

#endif
