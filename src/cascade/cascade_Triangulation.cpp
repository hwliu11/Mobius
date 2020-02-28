//-----------------------------------------------------------------------------
// Created on: 20 September 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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
#include <mobius/cascade_Triangulation.h>

//-----------------------------------------------------------------------------

mobius::cascade_Triangulation::cascade_Triangulation(const t_ptr<poly_Mesh>& mobiusMesh)
{
  m_mobiusMesh = mobiusMesh;
  m_bIsDone    = false;
}

//-----------------------------------------------------------------------------

mobius::cascade_Triangulation::cascade_Triangulation(const Handle(Poly_Triangulation)& occtMesh)
{
  m_occtMesh = occtMesh;
  m_bIsDone  = false;
}

//-----------------------------------------------------------------------------

mobius::cascade_Triangulation::~cascade_Triangulation()
{}

//-----------------------------------------------------------------------------

void mobius::cascade_Triangulation::DirectConvert()
{
  if ( !m_mobiusMesh.IsNull() )
    this->convertToOpenCascade();
  else if ( !m_occtMesh.IsNull() )
    this->convertToMobius();
}

//-----------------------------------------------------------------------------

const mobius::t_ptr<mobius::poly_Mesh>&
  mobius::cascade_Triangulation::GetMobiusTriangulation() const
{
  return m_mobiusMesh;
}

//-----------------------------------------------------------------------------

const Handle(Poly_Triangulation)&
  mobius::cascade_Triangulation::GetOpenCascadeTriangulation() const
{
  return m_occtMesh;
}

//-----------------------------------------------------------------------------

bool mobius::cascade_Triangulation::IsDone() const
{
  return m_bIsDone;
}

//-----------------------------------------------------------------------------

void mobius::cascade_Triangulation::convertToOpenCascade()
{
  if ( !m_mobiusMesh->GetNumVertices() || !m_mobiusMesh->GetNumTriangles() )
    return;

  TColgp_Array1OfPnt    nodes     ( 1, m_mobiusMesh->GetNumVertices() );
  Poly_Array1OfTriangle triangles ( 1, m_mobiusMesh->GetNumTriangles() );

  // Populate array of nodes.
  for ( int i = 1; i <= nodes.Length(); ++i )
  {
    // Get vertex of Mobius.
    poly_Vertex v;
    poly_VertexHandle vh(i-1);
    //
    m_mobiusMesh->GetVertex(vh, v);

    // Set node of OpenCascade.
    nodes(i) = gp_Pnt( v.X(), v.Y(), v.Z() );
  }

  // Populate array of triangles.
  for ( int i = 1; i <= triangles.Length(); ++i )
  {
    // Get triangle of Mobius.
    poly_Triangle t;
    poly_TriangleHandle th(i-1);
    //
    m_mobiusMesh->GetTriangle(th, t);
    //
    poly_VertexHandle vh0, vh1, vh2;
    t.GetVertices(vh0, vh1, vh2);

    // Set triangle of OpenCascade.
    triangles(i) = Poly_Triangle(vh0.GetIdx() + 1,
                                 vh1.GetIdx() + 1,
                                 vh2.GetIdx() + 1);
  }

  // Construct triangulation.
  m_occtMesh = new Poly_Triangulation(nodes, triangles);

  // Set done.
  m_bIsDone = true;
}

//-----------------------------------------------------------------------------

void mobius::cascade_Triangulation::convertToMobius()
{
  const TColgp_Array1OfPnt&    occtNodes     = m_occtMesh->Nodes();
  const Poly_Array1OfTriangle& occtTriangles = m_occtMesh->Triangles();

  // Construct Mobius mesh.
  m_mobiusMesh = new poly_Mesh;

  // Populate Mobius nodes.
  for ( int i = 1; i <= occtNodes.Length(); ++i )
  {
    const double x = occtNodes(i).X();
    const double y = occtNodes(i).Y();
    const double z = occtNodes(i).Z();

    // Add vertex.
    m_mobiusMesh->AddVertex(x, y, z);
  }

  // Populate Mobius triangles.
  for ( int i = 1; i <= occtTriangles.Length(); ++i )
  {
    int occtN1, occtN2, occtN3;
    occtTriangles(i).Get(occtN1, occtN2, occtN3);

    // Add triangle.
    poly_VertexHandle vh0(occtN1-1);
    poly_VertexHandle vh1(occtN2-1);
    poly_VertexHandle vh2(occtN3-1);
    //
    m_mobiusMesh->AddTriangle(vh0, vh1, vh2);
  }

  m_bIsDone = true;
}
