//-----------------------------------------------------------------------------
// Created on: 13 October 2013
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
#include <mobius/visu_ActorSectionCloud.h>

//! Constructor.
//! \param Cloud [in] point cloud to draw.
//! \param Color [in] pixel color to use for all points in the cloud.
mobius::visu_ActorSectionCloud::visu_ActorSectionCloud(const t_ptr<geom_SectionCloud>& Cloud,
                                                       const visu_ColorRGB<GLubyte>&   Color)
: visu_ActorInsensitive()
{
  m_cloud      = Cloud;
  m_color      = Color;
  m_fPointSize = 5.0f;

  // Calculate number of points
  m_iNumPoints = 0;
  for ( size_t s = 0; s < m_cloud->GetNumberOfSections(); ++s )
    m_iNumPoints += m_cloud->GetSectionByIndex(s)->Pts->GetNumberOfPoints();

  // Initialize array of vertices
  m_pVertices = new GLfloat[m_iNumPoints*3];
  int idx = 0;
  for ( size_t s = 0; s < m_cloud->GetNumberOfSections(); ++s )
  {
    const std::vector<t_xyz>& section = m_cloud->GetSectionByIndex(s)->Pts->GetPoints();
    for ( size_t p = 0; p < section.size(); ++p )
    {
      const t_xyz& point = section[p];
      m_pVertices[idx++] = (GLfloat) point.X();
      m_pVertices[idx++] = (GLfloat) point.Y();
      m_pVertices[idx++] = (GLfloat) point.Z();
    }
  }
}

//! Destructor.
mobius::visu_ActorSectionCloud::~visu_ActorSectionCloud()
{}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorSectionCloud::GetBounds(double& xMin, double& xMax,
                                               double& yMin, double& yMax,
                                               double& zMin, double& zMax) const
{
  m_cloud->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
}

//! Draws point cloud.
void mobius::visu_ActorSectionCloud::GL_Draw()
{
  glEnable(GL_POINT_SMOOTH);

    glPointSize(m_fPointSize);
    glColor3ub(m_color.R, m_color.G, m_color.B);

    glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(3, GL_FLOAT, 0, m_pVertices);
      glDrawArrays( GL_POINTS, 0, (int) m_iNumPoints );
    glDisableClientState(GL_VERTEX_ARRAY);

    glPointSize(1.0f);

  glDisable(GL_POINT_SMOOTH);
}
