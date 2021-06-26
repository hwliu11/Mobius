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

// Standard includes
#include <math.h>
#include <unordered_map>

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

  //! Computes normal vector for the triangle in question.
  //! \param[in]  ht   handle of the triangle in question.
  //! \param[out] norm computed normal vector.
  //! \return true if the normal vector was computed successfully,
  //!         false -- otherwise.
  mobiusPoly_EXPORT bool
    ComputeNormal(const poly_TriangleHandle ht,
                  t_xyz&                    norm) const;

  //! Computes area for the passed triangle.
  //! \param[in] ht handle of the triangle to compute the area for.
  //! \return the computed area.
  mobiusPoly_EXPORT double
    ComputeArea(const poly_TriangleHandle ht) const;

  //! Computes the max length for the edges of the passed triangle.
  //! \param[in] ht handle of the triangle to compute the max length for.
  //! \return the computed max length.
  mobiusPoly_EXPORT double
    ComputeMaxLen(const poly_TriangleHandle ht) const;

  //! Checks if the passed triangle is degenerated w.r.t. the given
  //! precision value.
  //! \param[in] ht   the triangle to check.
  //! \param[in] prec the precision to use.
  //! \return true/false.
  mobiusPoly_EXPORT bool
    IsDegenerated(const poly_TriangleHandle ht,
                  const double              prec) const;

  //! Creates a submesh for the given triangle by subdividing it
  //! at the center points.
  //! WARNING: the resulting mesh is not conformal!
  //! \param[in] ht the triangle to subdivide.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    Subdivide(const poly_TriangleHandle ht);

  //! Computes connectivity information as a set of mesh links
  //! married to the elements they share.
  mobiusPoly_EXPORT void
    ComputeEdges();

  //! Returns handles of the triangles sharing the passed edge.
  //! \param[in]  he  the edge handle to check.
  //! \param[out] hts the output triangles.
  //! \return false if the links were not computed or there is no
  //!         edge with such a handle.
  mobiusPoly_EXPORT bool
    GetTriangles(const poly_EdgeHandle             he,
                 std::vector<poly_TriangleHandle>& hts) const;

  //! Checks if the passed edge can be flipped and returns the pair of
  //! triangles to flip. The links should have been computed before you
  //! call this method.
  //! \param[in]  he          the edge to check.
  //! \param[in]  normDevRad  the allowed absolute normal deviation (radians).
  //! \param[in]  planeDevRad the allowed in-plane deviation (radians).
  //! \param[out] ht0         the first triangle.
  //! \param[out] ht1         the second triangle.
  //! \param[out] a           the opposite vertex on the first triangle.
  //! \param[out] b           the opposite vertex on the second triangle.
  //! \param[out] x           the first vertex on the edge to flip.
  //! \param[out] y           the second vertex on the edge to flip.
  //! \return true/false.
  mobiusPoly_EXPORT bool
    CanFlip(const poly_EdgeHandle he,
            const double          normDevRad,
            const double          planeDevRad,
            poly_TriangleHandle&  ht0,
            poly_TriangleHandle&  ht1,
            poly_VertexHandle&    a,
            poly_VertexHandle&    b,
            poly_VertexHandle&    x,
            poly_VertexHandle&    y) const;

  //! Checks if the passed edge can be flipped. The links should have
  //! been computed before you call this method.
  //! \param[in] he          the edge to check.
  //! \param[in] normDevRad  the allowed absolute normal deviation (radians).
  //! \param[in] planeDevRad the allowed in-plane deviation (radians).
  //! \return true/false.
  mobiusPoly_EXPORT bool
    CanFlip(const poly_EdgeHandle he,
            const double          normDevRad  = 1./180.*M_PI,
            const double          planeDevRad = 15./180.*M_PI) const;

  //! Flips all edges that allow flipping.
  //! \param[in] normDevRad  the allowed absolute normal deviation (radians).
  //! \param[in] planeDevRad the allowed in-plane deviation (radians).
  //! \return the number of flips done.
  mobiusPoly_EXPORT int
    FlipEdges(const double normDevRad  = 1./180.*M_PI,
              const double planeDevRad = 15./180.*M_PI);

  //! Finds the given edge in the precomputed links.
  //! \param[in] e the edge to find.
  //! \return the edge handle.
  mobiusPoly_EXPORT poly_EdgeHandle
    FindEdge(const poly_Edge& e) const;

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

  //! Returns vertex by its handle as a coordinate triple.
  //! \param[in]  h      handle of a vertex to access.
  //! \param[out] vertex vertex.
  //! \return false if there is no such vertex.
  bool GetVertex(const poly_VertexHandle h,
                 t_xyz&                  vertex) const
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

  //! Creates a new triangle with a back reference to a CAD face.
  //! \param[in] hV0 1-st vertex.
  //! \param[in] hV1 2-nd vertex.
  //! \param[in] hV2 3-rd vertex.
  //! \param[in] ref back-reference index.
  //! \return handle of the just added triangle.
  poly_TriangleHandle AddTriangle(const poly_VertexHandle hV0,
                                  const poly_VertexHandle hV1,
                                  const poly_VertexHandle hV2,
                                  const int               ref)
  {
    m_triangles.push_back( poly_Triangle(hV0, hV1, hV2, ref) );
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

  //! Creates a new quad.
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

  //! Returns the number of triangles.
  //! \param[in] includeDeads whether to include dead ones.
  //! \return number of triangles.
  int GetNumTriangles(const bool includeDeads = false) const
  {
    if ( includeDeads )
      return int( m_triangles.size() );

    int count = 0;
    for ( const auto& tri : m_triangles )
      if ( !tri.IsDeleted() )
        count++;

    return count;
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

  //! Edges-to-triangles map.
  std::unordered_map< poly_Edge, std::vector<poly_TriangleHandle> > m_links;

};

//! Convenience shortcuts.
typedef poly_Mesh t_mesh;

}

#endif
