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
#include <mobius/visu_ActorMesh.h>

//-----------------------------------------------------------------------------

// Auxiliary functions.
namespace {

  //! Enable lighting scheme for the CAD model.
  void __glEnableLighting()
  {
    GLfloat mat_specular[] = {1.0f, 1.0f, 0.0f, 0.1f};
    GLfloat mat_shininess[] = {1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

    GLfloat light_position[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light_ambient[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Setup model color
    glColor4f(0.5f, 0.5f, 0.0f, 1.0f);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glShadeModel(GL_SMOOTH);
  }

  //! Disable lighting scheme for the CAD model.
  void __glDisableLighting()
  {
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
  }
}

//-----------------------------------------------------------------------------

mobius::visu_ActorMesh::visu_ActorMesh(const t_ptr<t_mesh>& mesh)
: visu_Actor (),
  m_mesh     (mesh)
{
  // Fill the vector of points.
  for ( t_mesh::VertexIterator vit(mesh); vit.More(); vit.Next() )
  {
    poly_VertexHandle hv = vit.Current();

    poly_Vertex v;
    mesh->GetVertex(hv, v);

    m_points.push_back( (GLfloat) v.X() );
    m_points.push_back( (GLfloat) v.Y() );
    m_points.push_back( (GLfloat) v.Z() );
  }

  // Fill the vector of indices.
  for ( t_mesh::TriangleIterator tit(mesh); tit.More(); tit.Next() )
  {
  }
}

//-----------------------------------------------------------------------------

mobius::visu_ActorMesh::~visu_ActorMesh()
{}

//-----------------------------------------------------------------------------

void mobius::visu_ActorMesh::GetBounds(double& xMin, double& xMax,
                                       double& yMin, double& yMax,
                                       double& zMin, double& zMax) const
{
  m_mesh->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
}

//-----------------------------------------------------------------------------

void mobius::visu_ActorMesh::GL_Draw()
{
  this->drawShading();
}

//-----------------------------------------------------------------------------

void mobius::visu_ActorMesh::drawShading() const
{
  if ( m_indices.IsNull() || m_points.empty() )
    return;

  __glEnableLighting();

  // Activate and specify pointer to vertex array.
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer( 3, GL_FLOAT, 0, &m_points->Value( m_points->Lower() ) );

  if ( !m_normals.IsNull() )
  {
    // Use normals only if they exists. Notice that without normals
    // shading will be wrong.
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer( GL_FLOAT, 0, &m_normals->Value( m_normals->Lower() ) );
  }

  glDrawElements( GL_TRIANGLES, m_indices->Length(), GL_UNSIGNED_INT, &m_indices->Value( m_indices->Lower() ) );

  // deactivate vertex arrays after drawing.
  if( !m_normals.IsNull() )
    glDisableClientState(GL_NORMAL_ARRAY);

  glDisableClientState(GL_VERTEX_ARRAY);

  __glDisableLighting();
}
