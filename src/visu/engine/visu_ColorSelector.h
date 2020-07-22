//-----------------------------------------------------------------------------
// Created on: 21 January 2014
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

#ifndef visu_ColorSelector_HeaderFile
#define visu_ColorSelector_HeaderFile

// GL includes
#include <gl/gl.h>
#include <gl/glu.h>

// core includes
#include <mobius/core_Precision.h>

// visu includes
#include <mobius/visu.h>

// geom includes
#include <mobius/geom_BSplineCurve.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Simple structure representing color in RGB format.
template<typename T>
struct visu_ColorRGB
{
  T R; //!< Red.
  T G; //!< Green.
  T B; //!< Blue.

  //! Default constructor.
  visu_ColorRGB() : R(0), G(0), B(0) {}

  //! Complete constructor.
  //! \param r [in] red component.
  //! \param g [in] green component.
  //! \param b [in] blue component.
  visu_ColorRGB(const T r, const T g, const T b) : R(r), G(g), B(b) {}
};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_VISU
//!
//! Utility facilitating color selection.
class visu_ColorSelector
{
public:

  //! Returns normalzed color components for the given index.
  //! \param i [in] 0-based index to select color for.
  //! \param rgb [out] color.
  mobiusVisu_EXPORT static void
    ColorByIndex_f(const GLint i, visu_ColorRGB<GLfloat>& rgb);

  //! Returns integer color components for the given index.
  //! \param i [in] 0-based index to select color for.
  //! \param rgb [out] color.
  mobiusVisu_EXPORT static void
    ColorByIndex_d(const GLint i, visu_ColorRGB<GLubyte>& rgb);

  //! Estimates derivatives for the passed curve finding its min and max
  //! value along the parametric range.
  //! \param crv   [in]  curve to estimate.
  //! \param d1Min [out] minimal derivative.
  //! \param d1Max [out] maximal derivative.
  //! \param d1Avg [out] average derivative.
  mobiusVisu_EXPORT static void
    EstimateDerivative(const t_ptr<t_bcurve>& crv,
                       double&                d1Min,
                       double&                d1Max,
                       double&                d1Avg);

  //! Returns integer color components for the given value of derivative.
  //! \param d [in] derivative value asked for the corresponding color.
  //! \param d1Min [in] minimal derivative.
  //! \param d1Max [in] maximal derivative.
  //! \param rgb [out] color.
  mobiusVisu_EXPORT static void
    ColorByDerivative_d(const double            d,
                        const double            d1Min,
                        const double            d1Max,
                        visu_ColorRGB<GLubyte>& rgb);

};

}

//-----------------------------------------------------------------------------
// Useful macro
//-----------------------------------------------------------------------------

#define QrBlack     visu_ColorRGB<GLubyte>(0,   0,   0)
#define QrWhite     visu_ColorRGB<GLubyte>(255, 255, 255)
#define QrRed       visu_ColorRGB<GLubyte>(255, 0,   0)
#define QrGreen     visu_ColorRGB<GLubyte>(0,   255, 0)
#define QrBlue      visu_ColorRGB<GLubyte>(0,   0,   255)
#define QrLightBlue visu_ColorRGB<GLubyte>(0,   145, 230)
#define QrYellow    visu_ColorRGB<GLubyte>(255, 255, 0)
#define QrGray      visu_ColorRGB<GLubyte>(25,  25,  25)

#endif
