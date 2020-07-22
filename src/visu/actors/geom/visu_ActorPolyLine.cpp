//-----------------------------------------------------------------------------
// Created on: 08 October 2015
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
#include <mobius/visu_ActorPolyLine.h>

//! Constructor.
//! \param PolyLine    [in] geometry to draw.
//! \param Color       [in] color to use.
//! \param doDrawPoles [in] indicates whether to render poles.
mobius::visu_ActorPolyLine::visu_ActorPolyLine(const t_ptr<geom_PolyLine>&   PolyLine,
                                               const visu_ColorRGB<GLubyte>& Color,
                                               const bool                    doDrawPoles)
: visu_ActorInsensitive(),
  m_polyline   (PolyLine),
  m_color      (Color),
  m_bDrawPoles (doDrawPoles)
{
}

//! Destructor.
mobius::visu_ActorPolyLine::~visu_ActorPolyLine()
{
}

//! Calculates boundary box for the polyline.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorPolyLine::GetBounds(double& xMin, double& xMax,
                                           double& yMin, double& yMax,
                                           double& zMin, double& zMax) const
{
  // TODO: NYI
  xMin = 0.0;
  yMin = 0.0;
  zMin = 0.0;
  xMax = 1.0;
  yMax = 1.0;
  zMax = 1.0;
}

//! Draws geometry.
void mobius::visu_ActorPolyLine::GL_Draw()
{
  // Render line
  #pragma region GL rendering
  glBegin(GL_LINES);
  glLineWidth(0.1f);
  glEnable(GL_LINE_SMOOTH);
    glColor3ub(m_color.R, m_color.G, m_color.B);
    for ( int k = 0; k < m_polyline->NumLinks(); ++k )
    {
      const t_ptr<geom_Link>& link = m_polyline->GetLink(k);
      glVertex3f( (GLfloat) link->P1().X(), (GLfloat) link->P1().Y(), (GLfloat) link->P1().Z() );
      glVertex3f( (GLfloat) link->P2().X(), (GLfloat) link->P2().Y(), (GLfloat) link->P2().Z() );
    }
  glDisable(GL_LINE_SMOOTH);
  glEnd();

  // Draw extremity points if requested
  if ( m_bDrawPoles )
  {
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3ub(QrRed.R, QrRed.G, QrRed.B);
      for ( size_t k = 0; k < m_polyline->NumLinks(); ++k )
      {
        const t_ptr<geom_Link>& link = m_polyline->GetLink(k);
        glVertex3f( (GLfloat) link->P1().X(), (GLfloat) link->P1().Y(), (GLfloat) link->P1().Z() );
        glVertex3f( (GLfloat) link->P2().X(), (GLfloat) link->P2().Y(), (GLfloat) link->P2().Z() );
      }
    glEnd();
  }
  #pragma endregion
}
