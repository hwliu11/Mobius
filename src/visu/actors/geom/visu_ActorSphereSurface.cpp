//-----------------------------------------------------------------------------
// Created on: 05 September 2014
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
#include <mobius/visu_ActorSphereSurface.h>

//! Constructor.
//! \param Surf [in] surface to draw.
mobius::visu_ActorSphereSurface::visu_ActorSphereSurface(const t_ptr<geom_SphereSurface>& Surf)
: visu_ActorInsensitiveSurface( Surf.Access() )
{
}

//! Destructor.
mobius::visu_ActorSphereSurface::~visu_ActorSphereSurface()
{
}

//! Draws surface.
void mobius::visu_ActorSphereSurface::GL_Draw()
{
  /* ==========================
   *  Render points on surface
   * ========================== */

  //glEnable(GL_POINT_SMOOTH);
  //glPointSize(1);

  //  glColor3f(1.0f, 0.8f, 0.1f);
  //  glEnableClientState(GL_VERTEX_ARRAY);
  //    glVertexPointer(3, GL_FLOAT, 0, m_pFilling);
  //    glDrawArrays(GL_POINTS, 0, m_iNumFillingPoints);
  //  glDisableClientState(GL_VERTEX_ARRAY);

  //glEnd();
  //glDisable(GL_POINT_SMOOTH);

  /* =============================
   *  Render isoparametric curves
   * ============================= */

  const int stepsNumber = 15;

  // Downcast to sphere surface
  t_ptr<geom_SphereSurface>
    sphSurf = t_ptr<geom_SphereSurface>::DownCast(m_surf);

  //-------------------------------
  // Render U-isoparametric curves
  //-------------------------------

  const double uMin = m_surf->GetMinParameter_U();
  const double uMax = m_surf->GetMaxParameter_U();
  const double uStep = (uMax - uMin) / stepsNumber;

  glLineWidth(1);

  glColor3f(0.0f, 0.8f, 1.0f);
  glEnable(GL_LINE_SMOOTH);
  glBegin(GL_LINES);

    double u = uMin;
    while ( u <= uMax )
    {
      t_ptr<geom_Circle> Iso = sphSurf->Iso_U(u);
      {
        const double pMin = Iso->GetMinParameter();
        const double pMax = Iso->GetMaxParameter();
        const double pStep = (pMax - pMin) / 20.0;
        double p = pMin;

        while ( p < pMax )
        {
          double pnext = p + pStep;
          if ( p + pStep > pMax )
            pnext = pMax;

          t_xyz C1, C2;
          Iso->Eval(p, C1);
          Iso->Eval(pnext, C2);
          glVertex3f( (GLfloat) C1.X(), (GLfloat) C1.Y(), (GLfloat) C1.Z() );
          glVertex3f( (GLfloat) C2.X(), (GLfloat) C2.Y(), (GLfloat) C2.Z() );
          p += pStep;
        }
      }

      u += uStep;
    }

  glEnd();
  glDisable(GL_LINE_SMOOTH);

  //-------------------------------
  // Render V-isoparametric curves
  //-------------------------------

  const double vMin = m_surf->GetMinParameter_V();
  const double vMax = m_surf->GetMaxParameter_V();
  const double vStep = (vMax - vMin) / stepsNumber;

  glColor3f(0.0f, 0.8f, 1.0f);
  glEnable(GL_LINE_SMOOTH);
  glBegin(GL_LINES);

    double v = vMin;
    while ( v <= vMax )
    {
      t_ptr<geom_Circle> Iso = sphSurf->Iso_V(v);
      {
        const double pMin = Iso->GetMinParameter();
        const double pMax = Iso->GetMaxParameter();
        const double pStep = (pMax - pMin) / 20.0;
        double p = pMin;

        while ( p < pMax )
        {
          double pnext = p + pStep;
          if ( p + pStep > pMax )
            pnext = pMax;

          t_xyz C1, C2;
          Iso->Eval(p, C1);
          Iso->Eval(pnext, C2);
          glVertex3f( (GLfloat) C1.X(), (GLfloat) C1.Y(), (GLfloat) C1.Z() );
          glVertex3f( (GLfloat) C2.X(), (GLfloat) C2.Y(), (GLfloat) C2.Z() );
          p += pStep;
        }
      }

      v += vStep;
    }

  glEnd();
  glDisable(GL_LINE_SMOOTH);

  /* ===================
   *  Render local axes
   * =================== */

  this->GL_drawAxes();
}
