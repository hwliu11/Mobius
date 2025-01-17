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

#ifndef cascade_Triangulation_HeaderFile
#define cascade_Triangulation_HeaderFile

// Cascade includes
#include <mobius/cascade.h>

// Core includes
#include <mobius/core_Ptr.h>

// Poly includes
#include <mobius/poly_Mesh.h>

// OCCT includes
#include <NCollection_DataMap.hxx>
#include <Poly_CoherentTriangulation.hxx>
#include <Poly_Triangulation.hxx>


namespace mobius {

//! \ingroup MOBIUS_CASCADE
//!
//! Bridge for conversions between Mobius and OCCT triangulations.
template <typename Traits = poly_Traits>
class cascade_Triangulation
{
public:

  //! Ctor.
  cascade_Triangulation(const t_ptr< poly_Mesh<Traits> >& mobiusMesh)
  {
    m_mobiusMesh = mobiusMesh;
    m_bIsDone    = false;
  }

  //! Ctor.
  cascade_Triangulation(const Handle(Poly_Triangulation)& occtMesh)
  {
    m_occtMesh = occtMesh;
    m_bIsDone  = false;
  }

  //! Dtor.
  ~cascade_Triangulation() = default;

public:

  void DirectConvert()
  {
    if ( !m_mobiusMesh.IsNull() )
      this->convertToOpenCascade();
    else if ( !m_occtMesh.IsNull() )
      this->convertToMobius();
  }

public:

  const t_ptr< poly_Mesh<Traits> >& GetMobiusTriangulation() const
  {
    return m_mobiusMesh;
  }

  const Handle(Poly_Triangulation)& GetOpenCascadeTriangulation() const
  {
    return m_occtMesh;
  }

  bool IsDone() const
  {
    return m_bIsDone;
  }

protected:

  void convertToOpenCascade()
  {
    if ( !m_mobiusMesh->GetNumVertices() || !m_mobiusMesh->GetNumTriangles() )
      return;

    Handle(Poly_CoherentTriangulation) cohTris = new Poly_CoherentTriangulation;

    NCollection_DataMap<int, int> nodes;

    // Populate array of nodes.
    for ( typename poly_Mesh<Traits>::VertexIterator vit(m_mobiusMesh); vit.More(); vit.Next() )
    {
      // Get vertex of Mobius.
      poly_Vertex v;
      poly_VertexHandle hv = vit.Current();
      //
      m_mobiusMesh->GetVertex(hv, v);

      // Add node of OpenCascade.
      const int occNodeId = cohTris->SetNode( gp_XYZ( v.X(), v.Y(), v.Z() ) );

      // Register node mapping.
      nodes.Bind( hv.GetIdx(), occNodeId );
    }

    // Populate array of triangles.
    for ( typename poly_Mesh<Traits>::TriangleIterator tit(m_mobiusMesh); tit.More(); tit.Next() )
    {
      // Get triangle of Mobius.
      poly_Triangle<Traits> t;
      poly_TriangleHandle th = tit.Current();
      //
      m_mobiusMesh->GetTriangle(th, t);

      // Skip dead triangles.
      if ( t.IsDeleted() )
        continue;

      // Get vertices.
      poly_VertexHandle vh0, vh1, vh2;
      t.GetVertices(vh0, vh1, vh2);

      // Add triangle of OpenCascade.
      cohTris->AddTriangle( nodes( vh0.GetIdx() ),
                            nodes( vh1.GetIdx() ),
                            nodes( vh2.GetIdx() ) );
    }

    // Construct triangulation.
    m_occtMesh = cohTris->GetTriangulation();

    // Set done.
    m_bIsDone = true;
  }

  void convertToMobius()
  {
    // Construct Mobius mesh.
    m_mobiusMesh = new poly_Mesh<Traits>;

    // Populate Mobius nodes.
    for ( int i = 1; i <= m_occtMesh->NbNodes(); ++i )
    {
      const double x = m_occtMesh->Node(i).X();
      const double y = m_occtMesh->Node(i).Y();
      const double z = m_occtMesh->Node(i).Z();

      // Add vertex.
      m_mobiusMesh->AddVertex(x, y, z);
    }

    // Populate Mobius triangles.
    for ( int i = 1; i <= m_occtMesh->NbTriangles(); ++i )
    {
      int occtN1, occtN2, occtN3;
      m_occtMesh->Triangle(i).Get(occtN1, occtN2, occtN3);

      // Add triangle.
      poly_VertexHandle vh0(occtN1-1);
      poly_VertexHandle vh1(occtN2-1);
      poly_VertexHandle vh2(occtN3-1);
      //
      m_mobiusMesh->AddTriangle(vh0, vh1, vh2);
    }

    m_bIsDone = true;
  }

private:

  //! Mobius data structure.
  t_ptr< poly_Mesh<Traits> > m_mobiusMesh;

  //! OCCT data structure.
  Handle(Poly_Triangulation) m_occtMesh;

  //! Indicates whether conversion is done or not.
  bool m_bIsDone;

};

}

#endif
