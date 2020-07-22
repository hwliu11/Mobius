//-----------------------------------------------------------------------------
// Created on: 15 December 2014
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
#include <mobius/visu_ActorKleinBottle.h>

//! Constructor.
//! \param Surf [in] Klein bottle surface to draw.
mobius::visu_ActorKleinBottle::visu_ActorKleinBottle(const t_ptr<geom_KleinBottle>& Surf)
: visu_ActorInsensitiveSurface( Surf.Access() )
{
}

//! Destructor.
mobius::visu_ActorKleinBottle::~visu_ActorKleinBottle()
{
}

//! Draws Klein bottle surface.
void mobius::visu_ActorKleinBottle::GL_Draw()
{
  t_ptr<geom_KleinBottle> surf = t_ptr<geom_KleinBottle>::DownCast(m_surf);

  /* ==========================
   *  Render points on surface
   * ========================== */

  #pragma region Filling
  //glEnable(GL_POINT_SMOOTH);
  //glPointSize(1);

  //  glColor3f(0.0f, 0.0f, 1.0f);
  //  glEnableClientState(GL_VERTEX_ARRAY);
  //    glVertexPointer(3, GL_FLOAT, 0, m_pFilling);
  //    glDrawArrays(GL_POINTS, 0, m_iNumFillingPoints);
  //  glDisableClientState(GL_VERTEX_ARRAY);

  //glEnd();
  //glDisable(GL_POINT_SMOOTH);
  #pragma endregion

  /* ===============================
   *  Render U-isoparametric curves
   * =============================== */

  const int stepsNumber = 60;

  const double uMin  = m_surf->GetMinParameter_U();
  const double uMax  = m_surf->GetMaxParameter_U();
  const double uStep = (uMax - uMin) / stepsNumber;

  const double vMin  = m_surf->GetMinParameter_V();
  const double vMax  = m_surf->GetMaxParameter_V();
  const double vStep = (vMax - vMin) / stepsNumber;

  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glLineWidth(1.2f);

  //glPointSize(2);
  glBegin(GL_LINES);

    glColor3f(0.5f, 0.5f, 0.5f);
    //glColor3f(0.1f, 0.1f, 0.1f);

    double u = uMin;
    while ( u <= uMax )
    {
      t_ptr<geom_KleinIsoCurve> Iso = surf->Iso_U(u);
      {
        const double pMin = Iso->GetMinParameter();
        const double pMax = Iso->GetMaxParameter();
        const double pStep = (pMax - pMin) / 100.0;
        double p = pMin;

        t_xyz C_prev;
        Iso->Eval(p, C_prev);
        while ( p <= pMax )
        {
          p += pStep;
          t_xyz C;
          Iso->Eval(p, C);
          glVertex3f( (GLfloat) C_prev.X(), (GLfloat) C_prev.Y(), (GLfloat) C_prev.Z() );
          glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );
          C_prev = C;
        }
      }

      u += uStep;
    }

  glEnd();

  /* ===============================
   *  Render V-isoparametric curves
   * =============================== */

  //glPointSize(2);
  glBegin(GL_LINES);

    glColor3f(1.0f, 1.0f, 1.0f);
    //glColor3f(0.1f, 0.1f, 0.1f);

    double v = vMin;
    while ( v <= vMax )
    {
      t_ptr<geom_KleinIsoCurve> Iso = surf->Iso_V(v);
      {
        const double pMin = Iso->GetMinParameter();
        const double pMax = Iso->GetMaxParameter();
        const double pStep = (pMax - pMin) / 100.0;
        double p = pMin;

        t_xyz C_prev;
        Iso->Eval(p, C_prev);
        while ( p <= pMax )
        {
          p += pStep;
          t_xyz C;
          Iso->Eval(p, C);
          glVertex3f( (GLfloat) C_prev.X(), (GLfloat) C_prev.Y(), (GLfloat) C_prev.Z() );
          glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );
          C_prev = C;
        }
      }

      v += vStep;
    }

  glEnd();
  glDisable(GL_LINE_SMOOTH);
}
