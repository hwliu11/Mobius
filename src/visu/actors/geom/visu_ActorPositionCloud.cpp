//-----------------------------------------------------------------------------
// Created on: 29 January 2014
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
#include <mobius/visu_ActorPositionCloud.h>

//! Constructor.
//! \param Cloud [in] point cloud to draw.
//! \param Color [in] pixel color to use for all points in the cloud.
mobius::visu_ActorPositionCloud::visu_ActorPositionCloud(const t_ptr<t_pcloud>&          Cloud,
                                                         const visu_ColorRGB<GLubyte>& Color)
: visu_Actor()
{
  m_cloud = Cloud;
  m_color = Color;
  m_fPointSize = 1.0f;

  // Initialize array of vertices
  m_pVertices = new GLfloat[Cloud->GetNumberOfPoints()*3];
  int idx = 0;
  for ( int p = 0; p < m_cloud->GetNumberOfPoints(); ++p )
  {
    const t_xyz& point = m_cloud->GetPoint(p);
    m_pVertices[idx++] = (GLfloat) point.X();
    m_pVertices[idx++] = (GLfloat) point.Y();
    m_pVertices[idx++] = (GLfloat) point.Z();
  }
}

//! Destructor.
mobius::visu_ActorPositionCloud::~visu_ActorPositionCloud()
{
  delete[] m_pVertices;
}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorPositionCloud::GetBounds(double& xMin, double& xMax,
                                                double& yMin, double& yMax,
                                                double& zMin, double& zMax) const
{
  m_cloud->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
}

//! Draws point cloud.
void mobius::visu_ActorPositionCloud::GL_Draw()
{
  glEnable(GL_POINT_SMOOTH);

    glPointSize(m_fPointSize);
    glColor3ub(m_color.R, m_color.G, m_color.B);

    glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(3, GL_FLOAT, 0, m_pVertices);
      glDrawArrays( GL_POINTS, 0, m_cloud->GetNumberOfPoints() );
    glDisableClientState(GL_VERTEX_ARRAY);
    glPointSize(1.0f);

  glDisable(GL_POINT_SMOOTH);
}

//! Intersects cloud of points with the given ray.
//! \param ray [in] ray to intersect the point cloud with.
//! \return picked portion of data or NULL.
mobius::t_ptr<mobius::visu_DataSet>
  mobius::visu_ActorPositionCloud::IntersectWithLine(const t_ptr<geom_Line>& ray)
{
  // Data for highlighting
  t_ptr<visu_DataPositionCloud> hiliData;

  // Working variables
  const int nPoints   = m_cloud->GetNumberOfPoints();
  double    best_prec = 1.0;
  int       idx       = -1;

  // Now intersect the entire cloud with ray
  for ( int p_idx = 0; p_idx < nPoints; ++p_idx )
  {
    const t_xyz& O = ray->GetOrigin();
    const t_xyz& P = m_cloud->GetPoint(p_idx);

    t_xyz OP = P - O;
    const double ang = OP.Angle( ray->GetDir() );
    const double h = OP.Modulus() * sin(ang);

    if ( h < best_prec )
    {
      idx = (int) p_idx;
      best_prec = h;
    }
  }

  // Fill data set with just one point
  if ( idx != -1 )
  {
    hiliData = new visu_DataPositionCloud();
    hiliData->AddPoint( m_cloud->GetPoint(idx) );
  }

  return hiliData.Access();
}

//! Sets data set to highlight.
//! \param data [in] data set to highlight.
void mobius::visu_ActorPositionCloud::SetHiliData(const t_ptr<visu_DataSet>& data)
{
  m_hiliCloud = t_ptr<visu_DataPositionCloud>::DownCast(data);
}

//! Returns data set used for highlighting.
//! \return portion of data set.
mobius::t_ptr<mobius::visu_DataSet>
  mobius::visu_ActorPositionCloud::GetHiliData() const
{
  if ( !m_hiliCloud.IsNull() )
    return m_hiliCloud.Access();

  return NULL;
}

//! Cleans up data used for highlighting.
void mobius::visu_ActorPositionCloud::ClearHiliData()
{
  if ( !m_hiliCloud.IsNull() )
    m_hiliCloud->Clear();
}

//! Highlights picked portion of data.
void mobius::visu_ActorPositionCloud::GL_Hili() const
{
  if ( m_hiliCloud.IsNull() )
    return;

  const t_ptr<t_pcloud>& pcl = m_hiliCloud->GetCloud();
  if ( pcl.IsNull() )
    return;

  glEnable(GL_POINT_SMOOTH);

    glPointSize(10);
    glColor3ub(255, 0, 0);

    glBegin(GL_POINTS);
      for ( int i = 0; i < pcl->GetNumberOfPoints(); ++i )
      {
        const t_xyz& P = pcl->GetPoint(i);
        glVertex3f( (GLfloat) P.X(), (GLfloat) P.Y(), (GLfloat) P.Z() );
      }
    glEnd();

  glDisable(GL_POINT_SMOOTH);
}
