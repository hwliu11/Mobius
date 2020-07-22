//-----------------------------------------------------------------------------
// Created on: 02 August 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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

#ifndef visu_ActorBSplCurve_HeaderFile
#define visu_ActorBSplCurve_HeaderFile

// core includes
#include <mobius/core_HeapAlloc.h>

// visu includes
#include <mobius/visu_ActorInsensitive.h>
#include <mobius/visu_ColorSelector.h>

// geom includes
#include <mobius/geom_BSplineCurve.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL Actor dedicated to visualization of B-spline
//! 3D curves.
class visu_ActorBSplCurve : public visu_ActorInsensitive
{
public:

  mobiusVisu_EXPORT
    visu_ActorBSplCurve(const t_ptr<t_bcurve>&            Crv,
                          const visu_ColorRGB<GLubyte>& CurveColor = visu_ColorRGB<GLubyte>(255, 0, 0),
                          const bool                      isPoints = false,
                          const bool                      noPolygon = false,
                          const bool                      colorizeDerivatives = false);

  mobiusVisu_EXPORT virtual
    ~visu_ActorBSplCurve();

public:

  mobiusVisu_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

  mobiusVisu_EXPORT virtual void
    GL_Draw();

public:

  //! Sets color of curve.
  //! \param color [in] color to set.
  void SetCurveColor(const visu_ColorRGB<GLubyte>& color)
  {
    m_curveColor = color;
  }

  //! Returns color of curve.
  //! \return color.
  const visu_ColorRGB<GLubyte>& CurveColor() const
  {
    return m_curveColor;
  }

  //! Enables/disables curvature "combs" diagram.
  //! \param on [in] true/false;
  void SetHedgehog(const bool on)
  {
    m_bHedgehog = on;
  }

  //! Enables/disables visualization of control polygon.
  //! \param on [in] true/false;
  void SetPoles(const bool on)
  {
    m_bNoPoly = !on;
  }

private:

  //! Curve to draw.
  t_ptr<t_bcurve> m_curve;

  //! Color.
  visu_ColorRGB<GLubyte> m_curveColor;

  //! Indicates whether to render curve with points instead of polyline.
  bool m_bAsPoints;

  //! Indicates whether to render the control polygon.
  bool m_bNoPoly;

  //! TODO: remove this (testing only).
  t_xyz m_hiliPole;

  //! Indicates whether derivatives are to be colorized or not.
  bool m_bColorizeDerivatives;

  //! Minimal derivative modulus.
  double m_fD1Min;

  //! Maximal derivative modulus.
  double m_fD1Max;

  //! Curve points.
  std::vector<t_xyz> m_points;

  //! Curve derivatives.
  std::vector<t_xyz> m_derivatives;

  //! Curvatures.
  std::vector<t_xyz> m_curvatures;

  //! Allocator.
  core_HeapAlloc2D<double> m_alloc;

  //! Derivatives for B-spline basis functions.
  double** m_pDN;

  //! Trigger for curvature "combs" diagram.
  bool m_bHedgehog;

};

}

#endif
