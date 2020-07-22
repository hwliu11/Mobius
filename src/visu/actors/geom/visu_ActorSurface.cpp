//-----------------------------------------------------------------------------
// Created on: 20 June 2014
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
#include <mobius/visu_ActorSurface.h>

//! Constructor.
//! \param Surf [in] surface to draw.
mobius::visu_ActorSurface::visu_ActorSurface(const t_ptr<geom_Surface>& Surf)
: visu_Actor()
{
  m_surf = Surf;

  //--------------------------
  // Data for surface filling
  //--------------------------

  const int stepsNumber = 200;

  const double uMin = m_surf->GetMinParameter_U();
  const double uMax = m_surf->GetMaxParameter_U();
  const double uStep = (uMax - uMin) / stepsNumber;

  const double vMin = m_surf->GetMinParameter_V();
  const double vMax = m_surf->GetMaxParameter_V();
  const double vStep = (vMax - vMin) / stepsNumber;

  // Number of points in filling
  m_iNumFillingPoints = (stepsNumber+1)*(stepsNumber+1);

  // Initialize array of vertices for filling
  m_pFilling = new GLfloat[m_iNumFillingPoints*3];
  int idx = 0, u_idx = 0;
  while ( u_idx <= stepsNumber )
  {
    double u = uMin + u_idx*uStep;

    int v_idx = 0;
    while ( v_idx <= stepsNumber )
    {
      double v = vMin + v_idx*vStep;

      t_xyz S;
      m_surf->Eval(u, v, S);
      m_pFilling[idx++] = (GLfloat) S.X();
      m_pFilling[idx++] = (GLfloat) S.Y();
      m_pFilling[idx++] = (GLfloat) S.Z();
      v_idx++;
    }
    u_idx++;
  }

  //-----------------------
  // Local axes of surface
  //-----------------------

  double xMin, yMin, zMin, xMax, yMax, zMax;
  this->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);

  t_xyz min_pt(xMin, yMin, zMin);
  t_xyz max_pt(xMax, yMax, zMax);

  const GLfloat diag = (GLfloat) (max_pt - min_pt).Modulus();

  m_axes_OX = core_XYZ::OX()*diag/3.0;
  m_axes_OY = core_XYZ::OY()*diag/3.0;
  m_axes_OZ = core_XYZ::OZ()*diag/3.0;

  // Access transformation for surface
  core_IsoTransformChain T = m_surf->GetTransformChain();
  m_axes_origin = T.Apply(m_axes_origin);
  m_axes_OX     = T.Apply(m_axes_OX);
  m_axes_OY     = T.Apply(m_axes_OY);
  m_axes_OZ     = T.Apply(m_axes_OZ);
}

//! Destructor.
mobius::visu_ActorSurface::~visu_ActorSurface()
{
  delete[] m_pFilling; m_pFilling = NULL;
}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorSurface::GetBounds(double& xMin, double& xMax,
                                          double& yMin, double& yMax,
                                          double& zMin, double& zMax) const
{
  m_surf->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
}

//! Draws local axes of the surface.
void mobius::visu_ActorSurface::GL_drawAxes() const
{
  glEnable(GL_LINE_SMOOTH);
  glBegin(GL_LINES);

    // OX
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f( (GLfloat) m_axes_origin.X(), (GLfloat) m_axes_origin.Y(), (GLfloat) m_axes_origin.Z() );
    glVertex3f( (GLfloat) m_axes_OX.X(),     (GLfloat) m_axes_OX.Y(),     (GLfloat) m_axes_OX.Z() );

    // OY
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f( (GLfloat) m_axes_origin.X(), (GLfloat) m_axes_origin.Y(), (GLfloat) m_axes_origin.Z() );
    glVertex3f( (GLfloat) m_axes_OY.X(),     (GLfloat) m_axes_OY.Y(),     (GLfloat) m_axes_OY.Z() );

    // OZ
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f( (GLfloat) m_axes_origin.X(), (GLfloat) m_axes_origin.Y(), (GLfloat) m_axes_origin.Z() );
    glVertex3f( (GLfloat) m_axes_OZ.X(),     (GLfloat) m_axes_OZ.Y(),     (GLfloat) m_axes_OZ.Z() );

  glEnd();
  glDisable(GL_LINE_SMOOTH);
}
