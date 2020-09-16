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
#include <mobius/poly_Quad.h>
#include <mobius/poly_Triangle.h>
#include <mobius/poly_Vertex.h>

// Core includes
#include <mobius/core_Ptr.h>

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

  //! Calculates axis-aligned boundary box for the mesh.
  //! \param[out] xMin min X.
  //! \param[out] xMax max X.
  //! \param[out] yMin min Y.
  //! \param[out] yMax max Y.
  //! \param[out] zMin min Z.
  //! \param[out] zMax max Z.
  mobiusPoly_EXPORT void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

  //! Refines the triangle of interest by its midpoint.
  //! \param[in]  ht  handle of the triangle to refine.
  //! \param[out] ht0 handle of the first created triangle.
  //! \param[out] ht1 handle of the second created triangle.
  //! \param[out] ht2 handle of the third created triangle.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    RefineByMidpoint(const poly_TriangleHandle ht,
                     poly_TriangleHandle&      ht0,
                     poly_TriangleHandle&      ht1,
                     poly_TriangleHandle&      ht2);

  //! Refines the triangle of interest by its midpoint.
  //! \param[in] ht handle of the triangle to refine.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    RefineByMidpoint(const poly_TriangleHandle ht);

public:

  //! Returns vertex by its handle.
  //! \param[in]  h      handle of a vertex to access.
  //! \param[out] vertex vertex.
  //! \return false if there is no such vertex.
  bool GetVertex(const poly_VertexHandle h,
                 poly_Vertex&            vertex) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( m_vertices.size() ) ) return false;
    //
    vertex = m_vertices[idx];
    return true;
  }

  //! Returns edge by its handle.
  //! \param[in]  h    handle of an edge to access.
  //! \param[out] edge edge.
  //! \return false if there is no such edge.
  bool GetEdge(const poly_EdgeHandle h,
               poly_Edge&            edge) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( m_edges.size() ) ) return false;
    //
    edge = m_edges[idx];
    return true;
  }

  //! Returns triangle by its handle.
  //! \param[in]  h        handle of a triangle to access.
  //! \param[out] triangle triangle.
  //! \return false if there is no such triangle.
  bool GetTriangle(const poly_TriangleHandle h,
                   poly_Triangle&            triangle) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( m_triangles.size() ) ) return false;
    //
    triangle = m_triangles[idx];
    return true;
  }

  //! Returns quad by its handle.
  //! \param[in]  h    handle of a quad to access.
  //! \param[out] quad quad.
  //! \return false if there is no such quad.
  bool GetQuad(const poly_QuadHandle h,
               poly_Quad&            quad) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( m_quads.size() ) ) return false;
    //
    quad = m_quads[idx];
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

  //! Removes the triangle with the given handle.
  //! \param[in] h handle of the triangle to remove.
  //! \return true in case of success, false -- otherwise.
  bool RemoveTriangle(const poly_TriangleHandle h)
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( m_triangles.size() ) ) return false;

    m_triangles[idx].SetDeleted();
    return true;
  }

  //! Creates a new invalid quad.
  //! \return handle of the just added quad.
  poly_QuadHandle AddQuad()
  {
    poly_VertexHandle inv;
    return this->AddQuad(inv, inv, inv, inv);
  }

  //! Creates a new trianglquad.
  //! \param[in] hV0 1-st vertex.
  //! \param[in] hV1 2-nd vertex.
  //! \param[in] hV2 3-rd vertex.
  //! \param[in] hV3 4-th vertex.
  //! \return handle of the just added quad.
  poly_QuadHandle AddQuad(const poly_VertexHandle hV0,
                          const poly_VertexHandle hV1,
                          const poly_VertexHandle hV2,
                          const poly_VertexHandle hV3)
  {
    m_quads.push_back( poly_Quad(hV0, hV1, hV2, hV3) );
    poly_QuadHandle hQuad( int( m_quads.size() ) - 1 );
    return hQuad;
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

  //! \return number of quads.
  int GetNumQuads() const
  {
    return int( m_quads.size() );
  }

public:

  //! Base class for all iterators.
  class BaseIterator
  {
  public:

    //! Ctor accepting mesh.
    BaseIterator(const t_ptr<poly_Mesh>& mesh) : m_mesh(mesh), m_pos(0) {}

  protected:

    t_ptr<poly_Mesh> m_mesh; //!< Mesh to iterate.
    size_t           m_pos;  //!< Current position.

  };

  //! Iterator for vertices.
  class VertexIterator : public BaseIterator
  {
  public:

    //! Ctor accepting mesh.
    VertexIterator(const t_ptr<poly_Mesh>& mesh) : BaseIterator(mesh) {}

  public:

    bool More() const { return m_pos < m_mesh->m_vertices.size(); }

    void Next() { m_pos++; }

    poly_VertexHandle Current() const { return poly_VertexHandle( int(m_pos) ); }

  };

  //! Iterator for edges.
  class EdgeIterator : public BaseIterator
  {
  public:

    //! Ctor accepting mesh.
    EdgeIterator(const t_ptr<poly_Mesh>& mesh) : BaseIterator(mesh) {}

  public:

    bool More() const { return m_pos < m_mesh->m_edges.size(); }

    void Next() { m_pos++; }

    poly_EdgeHandle Current() const { return poly_EdgeHandle( int(m_pos) ); }

  };

  //! Iterator for triangles.
  class TriangleIterator : public BaseIterator
  {
  public:

    //! Ctor accepting mesh.
    TriangleIterator(const t_ptr<poly_Mesh>& mesh) : BaseIterator(mesh) {}

  public:

    bool More() const { return m_pos < m_mesh->m_triangles.size(); }

    void Next() { m_pos++; }

    poly_TriangleHandle Current() const { return poly_TriangleHandle( int(m_pos) ); }

  };

  //! Iterator for quads.
  class QuadIterator : public BaseIterator
  {
  public:

    //! Ctor accepting mesh.
    QuadIterator(const t_ptr<poly_Mesh>& mesh) : BaseIterator(mesh) {}

  public:

    bool More() const { return m_pos < m_mesh->m_quads.size(); }

    void Next() { m_pos++; }

    poly_QuadHandle Current() const { return poly_QuadHandle( int(m_pos) ); }

  };

protected:

  std::vector<poly_Vertex>   m_vertices;  //!< List of vertices.
  std::vector<poly_Edge>     m_edges;     //!< List of edges.
  std::vector<poly_Triangle> m_triangles; //!< List of triangles.
  std::vector<poly_Quad>     m_quads;     //!< List of quads.

};

//! Convenience shortcuts.
typedef poly_Mesh t_mesh;

}

#endif
