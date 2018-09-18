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
#include <mobius/poly_Elements.h>

// Core includes
#include <mobius/core_OBJECT.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Mesh data structure.
class poly_Mesh : public core_OBJECT
{
// Construction & destruction:
public:

  //! Default ctor.
  mobiusPoly_EXPORT
    poly_Mesh();

public:

  //! Returns handle of a vertex.
  //! \param[in] vertex to access a handle for.
  //! \return handle.
  poly_VertexHandle GetHandle(const poly_Vertex& vertex) const
  {
    return poly_VertexHandle( int( &vertex - &m_vertices.front() ) );
  }

  //! Returns handle of an edge.
  //! \param[in] edge to access a handle for.
  //! \return handle.
  poly_EdgeHandle GetHandle(const poly_Edge& edge) const
  {
    return poly_EdgeHandle( int( &edge - &m_edges.front() ) );
  }

  //! Returns handle of a face.
  //! \param[in] face to access a handle for.
  //! \return handle.
  poly_FaceHandle GetHandle(const poly_Face& face) const
  {
    return poly_FaceHandle( int( &face - &m_faces.front() ) );
  }

  //! Creates a new vertex and returns its handle.
  //! \return handle of the just added vertex.
  poly_VertexHandle CreateVertex()
  {
    m_vertices.push_back( poly_Vertex() );
    //
    return this->GetHandle( m_vertices.back() );
  }

  //! Creates a new edge between the passed two vertices.
  //! \param[in] hStartV  start vertex.
  //! \param[in] hFinishV finish vertex.
  //! \return handle of the just added edge.
  poly_EdgeHandle CreateEdge()
  {
    m_edges.push_back( poly_Edge() );

    // Get handle of the just created edge.
    poly_EdgeHandle hEdge = this->GetHandle( m_edges.back() );
  }

protected:

  std::vector<poly_Vertex> m_vertices; //!< List of vertices.
  std::vector<poly_Edge>   m_edges;    //!< List of edges.
  std::vector<poly_Face>   m_faces;    //!< List of faces.

};

//! Convenience shortcuts.
typedef poly_Mesh mesh;

};

#endif
