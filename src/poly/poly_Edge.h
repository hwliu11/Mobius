//-----------------------------------------------------------------------------
// Created on: 18 September 2018
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

#ifndef poly_Edge_HeaderFile
#define poly_Edge_HeaderFile

// Poly includes
#include <mobius/poly_Handles.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Edge entity.
class poly_Edge
{
public:

  //! Default ctor.
  mobiusPoly_EXPORT
    poly_Edge();

  //! Ctor accepting the nodes of the edge.
  //! \param[in] hv0 first vertex of the edge.
  //! \param[in] hv1 second vertex of the edge.
  mobiusPoly_EXPORT
    poly_Edge(const poly_VertexHandle hv0,
              const poly_VertexHandle hv1);

public:

  //! Returns vertex handles defining the edge.
  //! \param[out] hv0 1-st vertex.
  //! \param[out] hv1 2-nd vertex.
  void GetVertices(poly_VertexHandle& hv0,
                   poly_VertexHandle& hv1) const
  {
    hv0 = m_hVertices[0];
    hv1 = m_hVertices[1];
  }

  //! Compares this edge to the other.
  bool operator==(const poly_Edge& other) const
  {
    return ( (m_hVertices[0] == other.m_hVertices[0]) && (m_hVertices[1] == other.m_hVertices[1]) ) ||
           ( (m_hVertices[0] == other.m_hVertices[1]) && (m_hVertices[1] == other.m_hVertices[0]) );
  }

protected:

  poly_VertexHandle m_hVertices[2]; //!< Handles to the vertices.

};

}

// Std includes.
#include <functional>

namespace std {

//! Hasher for edge.
template <>
struct hash<mobius::poly_Edge>
{
  typedef mobius::poly_Edge argument_type;
  typedef std::size_t       result_type;

  //! Returns hash code of an edge for hashing.
  //! \param[in] e edge to hash.
  //! \return hash code.
  std::size_t operator()(const mobius::poly_Edge& e) const
  {
    mobius::poly_VertexHandle vh[2];
    e.GetVertices(vh[0], vh[1]);

    const int upper = 1000;

    int key = vh[0].GetIdx() + vh[1].GetIdx();
    key += (key << 10);
    key ^= (key >> 6);
    key += (key << 3);
    key ^= (key >> 11);
    return (key & 0x7fffffff) % upper;
  }

};

}

#endif
