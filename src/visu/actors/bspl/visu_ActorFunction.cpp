//-----------------------------------------------------------------------------
// Created on: 12 October 2015
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

// Own include
#include <mobius/visu_ActorFunction.h>

// visu includes
#include <mobius/visu_ColorSelector.h>

//! Constructor.
//! \param func [in] function to visualize.
mobius::visu_ActorFunction::visu_ActorFunction(const t_ptr<core_TwovariateFunc>& func)
: visu_ActorInsensitive (),
  m_func                (func)
{
  const int stepsNumber = 40;

  /* ==========================================
   *  Fill properties for isoparametric curves
   * ========================================== */

  const double uMin = 0.0;
  const double uMax = 1.0;
  const double uStep = (uMax - uMin) / stepsNumber;

  const double vMin = 0.0;
  const double vMax = 1.0;
  const double vStep = (vMax - vMin) / stepsNumber;

  // Iso U
  {
    double u = uMin;
    bool uStop = false;
    while ( !uStop )
    {
      if ( u > uMax )
      {
        u = uMax;
        uStop = true;
      }

      std::vector<t_xyz> isoU;
      double v = vMin;
      bool vStop = false;
      while ( !vStop )
      {
        if ( v > vMax )
        {
          v = vMax;
          vStop = true;
        }

        double f = m_func->Eval(u, v);
        t_xyz p(u, v, f);
        isoU.push_back(p);

        // Increment V step
        v += vStep;
      }

      m_isoU.push_back(isoU);

      // Increment U step
      u += uStep;
    }
  }

  // Iso V
  {
    double v = vMin;
    bool vStop = false;
    while ( !vStop )
    {
      if ( v > vMax )
      {
        v = vMax;
        vStop = true;
      }

      std::vector<t_xyz> isoV;
      double u = uMin;
      bool uStop = false;
      while ( !uStop )
      {
        if ( u > uMax )
        {
          u = uMax;
          uStop = true;
        }

        double f = m_func->Eval(u, v);
        t_xyz p(u, v, f);
        isoV.push_back(p);

        // Increment U step
        u += uStep;
      }

      m_isoV.push_back(isoV);

      // Increment V step
      v += vStep;
    }
  }
}

//! Destructor.
mobius::visu_ActorFunction::~visu_ActorFunction()
{}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorFunction::GetBounds(double& xMin, double& xMax,
                                           double& yMin, double& yMax,
                                           double& zMin, double& zMax) const
{
  xMin = 0.0;
  xMax = 1.0;
  yMin = 0.0;
  yMax = 1.0;
  zMin = zMax = 0.0;
}

//! Draws function.
void mobius::visu_ActorFunction::GL_Draw()
{
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

  /* ===============================
   *  Render U-isoparametric curves
   * =============================== */

  glLineWidth(1.5f);
  glBegin(GL_LINES);
    glColor3ub(QrWhite.R, QrWhite.G, QrWhite.B);
    for ( size_t iso = 0; iso < m_isoU.size(); ++iso )
    {
      const std::vector<t_xyz>& points = m_isoU[iso];
      t_xyz C_prev = points[0];
      for ( size_t k = 1; k < points.size(); ++k )
      {
        const t_xyz& C = points[k];
        glVertex3f( (GLfloat) C_prev.X(), (GLfloat) C_prev.Y(), (GLfloat) C_prev.Z() );
        glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );
        C_prev = C;
      }
    }
  glEnd();

  /* ===============================
   *  Render V-isoparametric curves
   * =============================== */

  glLineWidth(1.5f);
  glBegin(GL_LINES);
    glColor3ub(QrWhite.R, QrWhite.G, QrWhite.B);
    for ( size_t iso = 0; iso < m_isoV.size(); ++iso )
    {
      const std::vector<t_xyz>& points = m_isoV[iso];
      t_xyz C_prev = points[0];
      for ( size_t k = 1; k < points.size(); ++k )
      {
        const t_xyz& C = points[k];
        glVertex3f( (GLfloat) C_prev.X(), (GLfloat) C_prev.Y(), (GLfloat) C_prev.Z() );
        glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );
        C_prev = C;
      }
    }
  glEnd();
}
