//-----------------------------------------------------------------------------
// Created on: 03 December 2018
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

#ifndef geom_CoonsSurface_HeaderFile
#define geom_CoonsSurface_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Four-sided Coons patch.
class geom_CoonsSurface : public geom_Surface
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_CoonsSurface(const ptr<curve>& c0,
                      const ptr<curve>& c1,
                      const ptr<curve>& b0,
                      const ptr<curve>& b1,
                      const xyz&        p00,
                      const xyz&        p01,
                      const xyz&        p10,
                      const xyz&        p11);

  mobiusGeom_EXPORT virtual
    ~geom_CoonsSurface();

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

  ptr<curve> m_c0;  //!< Rail curve c0.
  ptr<curve> m_c1;  //!< Rail curve c1.
  ptr<curve> m_b0;  //!< Rail curve b0.
  ptr<curve> m_b1;  //!< Rail curve b1.
  xyz        m_p00; //!< Corner point p00.
  xyz        m_p01; //!< Corner point p01.
  xyz        m_p10; //!< Corner point p10.
  xyz        m_p11; //!< Corner point p11.

};

};

#endif
