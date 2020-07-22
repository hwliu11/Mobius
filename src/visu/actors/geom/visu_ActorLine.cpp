//-----------------------------------------------------------------------------
// Created on: 22 May 2013
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

// Windows
#include <windows.h>

// GL includes
#include <gl/gl.h>
#include <gl/glu.h>

// Own include
#include <mobius/visu_ActorLine.h>

//! Constructor.
//! \param Line              [in] line to draw.
//! \param uMin              [in] lower bound to avoid drawing infinite objects.
//! \param uMax              [in] upper bound to avoid drawing infinite objects.
//! \param doDrawExtremities [in] indicates whether to render extremities.
mobius::visu_ActorLine::visu_ActorLine(const t_ptr<geom_Line>& Line,
                                       const double            uMin,
                                       const double            uMax,
                                       const bool              doDrawExtremities)
: visu_ActorInsensitive(),
  m_fUMin(uMin),
  m_fUMax(uMax),
  m_line(Line),
  m_bDrawExtremities(doDrawExtremities)
{
}

//! Destructor.
mobius::visu_ActorLine::~visu_ActorLine()
{
}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorLine::GetBounds(double& xMin, double& xMax,
                                       double& yMin, double& yMax,
                                       double& zMin, double& zMax) const
{
  t_xyz P_min, P_max;

  m_line->Eval(m_fUMin, P_min);
  m_line->Eval(m_fUMax, P_max);

  xMin = P_min.X();
  yMin = P_min.Y();
  zMin = P_min.Z();

  xMax = P_max.X();
  yMax = P_max.Y();
  zMax = P_max.Z();
}

//! Draws geometry.
void mobius::visu_ActorLine::GL_Draw()
{
  // Evaluate line bounds
  t_xyz P_min, P_max;
  m_line->Eval(m_fUMin, P_min);
  m_line->Eval(m_fUMax, P_max);

  // Render line
  #pragma region GL rendering
  glBegin(GL_LINES);
  glLineWidth(1);
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex3f( (GLfloat) P_min.X(), (GLfloat) P_min.Y(), (GLfloat) P_min.Z() );
    glVertex3f( (GLfloat) P_max.X(), (GLfloat) P_max.Y(), (GLfloat) P_max.Z() );
  glEnd();

  // Draw extremity points if requested
  if ( m_bDrawExtremities )
  {
    glEnable(GL_POINT_SMOOTH);
      glPointSize(5);
      glBegin(GL_POINTS);
        glVertex3f( (GLfloat) P_min.X(), (GLfloat) P_min.Y(), (GLfloat) P_min.Z() );
        glVertex3f( (GLfloat) P_max.X(), (GLfloat) P_max.Y(), (GLfloat) P_max.Z() );
      glEnd();
    glDisable(GL_POINT_SMOOTH);
  }
  #pragma endregion
}
