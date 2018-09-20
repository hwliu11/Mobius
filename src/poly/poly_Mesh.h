//-----------------------------------------------------------------------------
// Created on: 17 September 2018
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

#ifndef poly_Mesh_HeaderFile
#define poly_Mesh_HeaderFile

// Poly includes
#include <mobius/poly_Edge.h>
#include <mobius/poly_Triangle.h>
#include <mobius/poly_Vertex.h>

// Core includes
#include <mobius/core_OBJECT.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Data structure representing surface triangulation.
//!
//! \sa mobius::poly_ReadSTL
class poly_Mesh : public core_OBJECT
{
// Construction & destruction:
public:

  //! Default ctor.
  mobiusPoly_EXPORT
    poly_Mesh();

public:

  //! Returns vertex by its handle.
  //! \param[in]  h      handle of a vertex to access.
  //! \param[out] vertex vertex.
  //! \return false if there is no such vertex.
  bool GetVertex(const poly_VertexHandle h,
                 poly_Vertex&            vertex)
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > m_vertices.size() ) return false;
    //
    vertex = m_vertices[idx];
    return true;
  }

  //! Returns triangle by its handle.
  //! \param[in]  h        handle of a triangle to access.
  //! \param[out] triangle triangle.
  //! \return false if there is no such triangle.
  bool GetTriangle(const poly_TriangleHandle h,
                   poly_Triangle&            triangle)
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > m_triangles.size() ) return false;
    //
    triangle = m_triangles[idx];
    return true;
  }

  //! Returns edge by its handle.
  //! \param[in]  h    handle of an edge to access.
  //! \param[out] edge edge.
  //! \return false if there is no such edge.
  bool GetEdge(const poly_EdgeHandle h,
               poly_Edge&            edge)
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > m_edges.size() ) return false;
    //
    edge = m_edges[idx];
    return true;
  }

  //! Creates a new vertex and returns its handle.
  //! \return handle of the just added vertex.
  poly_VertexHandle AddVertex()
  {
    return this->AddVertex( core_XYZ::O() );
  }

  //! Creates a new vertex with the given coordinates and returns its handle.
  //! \param[in] x coordinate x of the vertex.
  //! \param[in] y coordinate y of the vertex.
  //! \param[in] z coordinate z of the vertex.
  //! \return handle of the just added vertex.
  poly_VertexHandle AddVertex(const double x,
                              const double y,
                              const double z)
  {
    return this->AddVertex( core_XYZ(x, y, z) );
  }

  //! Creates a new vertex and returns its handle.
  //! \param[in] coords coordinates of the vertex.
  //! \return handle of the just added vertex.
  poly_VertexHandle AddVertex(const core_XYZ& coords)
  {
    m_vertices.push_back( poly_Vertex(coords) );
    poly_VertexHandle hVertex( int( m_vertices.size() ) - 1 );
    return hVertex;
  }

  //! Creates a new invalid edge.
  //! \return handle of the just added edge.
  poly_EdgeHandle AddEdge()
  {
    poly_VertexHandle inv;
    return this->AddEdge(inv, inv);
  }

  //! Creates a new edge between the passed two vertices.
  //! \param[in] hStartV  start vertex.
  //! \param[in] hFinishV finish vertex.
  //! \return handle of the just added edge.
  poly_EdgeHandle AddEdge(const poly_VertexHandle hStartV,
                          const poly_VertexHandle hFinishV)
  {
    m_edges.push_back( poly_Edge(hStartV, hFinishV) );
    poly_EdgeHandle hEdge( int( m_edges.size() ) - 1 );
    return hEdge;
  }

  //! Creates a new invalid triangle.
  //! \return handle of the just added triangle.
  poly_TriangleHandle AddTriangle()
  {
    poly_VertexHandle inv;
    return this->AddTriangle(inv, inv, inv);
  }

  //! Creates a new triangle.
  //! \param[in] hV0 1-st vertex.
  //! \param[in] hV1 2-nd vertex.
  //! \param[in] hV2 3-rd vertex.
  //! \return handle of the just added triangle.
  poly_TriangleHandle AddTriangle(const poly_VertexHandle hV0,
                                  const poly_VertexHandle hV1,
                                  const poly_VertexHandle hV2)
  {
    m_triangles.push_back( poly_Triangle(hV0, hV1, hV2) );
    poly_TriangleHandle hTriangle( int( m_triangles.size() ) - 1 );
    return hTriangle;
  }

  //! \return number of vertices.
  int GetNumVertices() const
  {
    return int( m_vertices.size() );
  }

  //! \return number of edges.
  int GetNumEdges() const
  {
    return int( m_edges.size() );
  }

  //! \return number of triangles.
  int GetNumTriangles() const
  {
    return int( m_triangles.size() );
  }

protected:

  std::vector<poly_Vertex>   m_vertices;  //!< List of vertices.
  std::vector<poly_Edge>     m_edges;     //!< List of edges.
  std::vector<poly_Triangle> m_triangles; //!< List of triangles.

};

//! Convenience shortcuts.
typedef poly_Mesh mesh;

};

#endif
