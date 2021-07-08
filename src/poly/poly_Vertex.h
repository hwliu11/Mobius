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

#ifndef poly_Vertex_HeaderFile
#define poly_Vertex_HeaderFile

// Poly includes
#include <mobius/poly_Flag.h>
#include <mobius/poly_Handles.h>

// Core includes
#include <mobius/core_XYZ.h>

// STL includes
#include <unordered_set>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Vertex entity.
class poly_Vertex
{
public:

  //! Default ctor.
  mobiusPoly_EXPORT
    poly_Vertex();

  //! Constructor.
  //! \param[in] coords coordinates to set.
  mobiusPoly_EXPORT
    poly_Vertex(const core_XYZ& coords);

  //! Constructor.
  //! \param[in] x coordinate x of the vertex.
  //! \param[in] y coordinate y of the vertex.
  //! \param[in] z coordinate z of the vertex.
  mobiusPoly_EXPORT
    poly_Vertex(const double x,
                const double y,
                const double z);

public:

  double X() const { return m_coords.X(); } //!< \return X coordinate.
  double Y() const { return m_coords.Y(); } //!< \return Y coordinate.
  double Z() const { return m_coords.Z(); } //!< \return Z coordinate.

  //! Type conversion operator.
  operator core_XYZ() const { return m_coords; }

  //! \return non-const reference to the vertex coordinates.
  core_XYZ& ChangeCoords()
  {
    return m_coords;
  }

  //! \return const reference to the stored triangle handles.
  const std::unordered_set<poly_TriangleHandle>& GetTriangleRefs() const
  {
    return m_tris;
  }

  //! \return non-const reference to the stored triangle handles.
  std::unordered_set<poly_TriangleHandle>& ChangeTriangleRefs()
  {
    return m_tris;
  }

  //! Adds another back reference to a triangle containing this vertex.
  //! \param[in] ht the triangle to add as a back reference.
  void AddTriangleRef(const poly_TriangleHandle ht)
  {
    m_tris.insert(ht);
  }

  //! Removes a back reference to a triangle containing this vertex.
  //! \param[in] ht the triangle to remove.
  void RemoveTriangleRef(const poly_TriangleHandle ht)
  {
    m_tris.erase(ht);
  }

  //! Checks if this vertex contains a back reference to the given triangle.
  //! \param[in] ht the triangle to check.
  //! \return true/false.
  bool HasTriangleRef(const poly_TriangleHandle ht) const
  {
    return m_tris.find(ht) != m_tris.end();
  }

  //! \return the associated flags.
  int GetFlags() const
  {
    return m_iFlags;
  }

  //! \return non-const reference to the associated flags.
  int& ChangeFlags()
  {
    return m_iFlags;
  }

  //! Flags this vertex as deleted.
  void SetDeleted()
  {
    m_iFlags |= Flag_Deleted;
  }

  //! \return true if this vertex is marked as deleted.
  bool IsDeleted() const
  {
    return (m_iFlags & Flag_Deleted) > 0;
  }

protected:

  int                                     m_iFlags; //!< Flags.
  core_XYZ                                m_coords; //!< Coordinates of the vertex.
  std::unordered_set<poly_TriangleHandle> m_tris;   //!< Back references to the owner triangles.

};

}

#endif
