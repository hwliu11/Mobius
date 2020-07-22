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

#include <windows.h>

// Own include
#include <mobius/visu_ColorSelector.h>

//-----------------------------------------------------------------------------

void
  mobius::visu_ColorSelector::ColorByIndex_f(const GLint i, visu_ColorRGB<GLfloat>& rgb)
{
  static visu_ColorRGB<GLfloat>
    ColorBank[] = { visu_ColorRGB<GLfloat>(1.0f, 0.0f, 0.0f),
                    visu_ColorRGB<GLfloat>(0.0f, 1.0f, 0.0f),
                    visu_ColorRGB<GLfloat>(0.0f, 0.0f, 1.0f),
                    visu_ColorRGB<GLfloat>(1.0f, 1.0f, 0.0f),
                    visu_ColorRGB<GLfloat>(0.0f, 1.0f, 1.0f),
                    visu_ColorRGB<GLfloat>(1.0f, 0.0f, 1.0f),
                    visu_ColorRGB<GLfloat>(0.5f, 0.5f, 1.0f),
                    visu_ColorRGB<GLfloat>(1.0f, 0.5f, 0.5f),
                    visu_ColorRGB<GLfloat>(0.5f, 1.0f, 0.5f) };

  const GLint num = (GLint) sizeof(ColorBank)/sizeof(visu_ColorRGB<GLfloat>);
  rgb = ColorBank[i % num];
}

//-----------------------------------------------------------------------------

void
  mobius::visu_ColorSelector::ColorByIndex_d(const GLint i, visu_ColorRGB<GLubyte>& rgb)
{
  static visu_ColorRGB<GLubyte>
    ColorBank[] = { visu_ColorRGB<GLubyte>(0,   0,   255),   // 0
                    visu_ColorRGB<GLubyte>(0,   0,   223),   // 1
                    visu_ColorRGB<GLubyte>(0,   100, 248),   // 2
                    visu_ColorRGB<GLubyte>(0,   155, 255),   // 3
                    visu_ColorRGB<GLubyte>(0,   201, 200),   // 4
                    visu_ColorRGB<GLubyte>(0,   255, 206),   // 5
                    visu_ColorRGB<GLubyte>(0,   147, 57),    // 6
                    visu_ColorRGB<GLubyte>(33,  225, 28),    // 7
                    visu_ColorRGB<GLubyte>(100, 250, 37),    // 8
                    visu_ColorRGB<GLubyte>(218, 254, 0),     // 9
                    visu_ColorRGB<GLubyte>(255, 200, 0),     // 10
                    visu_ColorRGB<GLubyte>(255, 125, 0),     // 11
                    visu_ColorRGB<GLubyte>(255, 0,   0),     // 12
                    visu_ColorRGB<GLubyte>(255, 255, 255) }; // 13

  const GLint num = (GLint) sizeof(ColorBank)/sizeof(visu_ColorRGB<GLubyte>);
  rgb = ColorBank[i % num];
}

//-----------------------------------------------------------------------------

void
  mobius::visu_ColorSelector::EstimateDerivative(const t_ptr<t_bcurve>& crv,
                                                 double&                d1Min,
                                                 double&                d1Max,
                                                 double&                d1Avg)
{
  const int    nSteps = 1000;
  const double uMin   = crv->GetMinParameter();
  const double uMax   = crv->GetMaxParameter();
  const double step   = (uMax - uMin) / nSteps;
  double       u      = uMin;

  d1Min =  DBL_MAX;
  d1Max = -DBL_MAX;
  d1Avg =  0.0;

  int iters = 0;
  while ( u < uMax )
  {
    t_xyz d1C;
    crv->Eval_Dk(u, 1, d1C);

    const double d = d1C.Modulus();
    d1Min = (((d) < (d1Min)) ? (d) : (d1Min));
    d1Max = (((d) < (d1Max)) ? (d) : (d1Max));
    d1Avg += d;

    u += step;
    iters++;
  }

  d1Avg /= iters;
}

//-----------------------------------------------------------------------------

void
  mobius::visu_ColorSelector::ColorByDerivative_d(const double            d,
                                                  const double            d1Min,
                                                  const double            d1Max,
                                                  visu_ColorRGB<GLubyte>& rgb)
{
  static visu_ColorRGB<GLubyte>
    ColorBank[] = { visu_ColorRGB<GLubyte>(255,   0,   0),
                    visu_ColorRGB<GLubyte>(255,  80,   0),
                    visu_ColorRGB<GLubyte>(255, 150,   0),
                    visu_ColorRGB<GLubyte>(255, 255,   0),
                    visu_ColorRGB<GLubyte>(255, 255,   0),
                    visu_ColorRGB<GLubyte>(80,  255,   0),
                    visu_ColorRGB<GLubyte>(0,   255,   0),
                    visu_ColorRGB<GLubyte>(255, 255,   0),
                    visu_ColorRGB<GLubyte>(0,   255, 255),
                    visu_ColorRGB<GLubyte>(0,   125, 255),
                    visu_ColorRGB<GLubyte>(0,   0,   255) };

  int nColors = sizeof(ColorBank)/sizeof(visu_ColorRGB<GLubyte>);
  double dStep = (d1Max - d1Min) / nColors;

  int id;
  if ( fabs(dStep) < core_Precision::Resolution3D() )
    id = 0;
  else
  {
    if ( d < d1Min )
      id = 0;
    else if ( d > d1Max )
      id = nColors - 1;
    else
      id = (int) ((d - d1Min) / dStep);
  }

  rgb = ColorBank[id];
}
