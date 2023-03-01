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

#ifndef poly_Triangle_HeaderFile
#define poly_Triangle_HeaderFile

// Poly includes
#include <mobius/poly_Flag.h>
#include <mobius/poly_Handles.h>
#include <mobius/poly_Traits.h>

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Triangle element.
template <typename Traits = poly_Traits>
class poly_Triangle
{
public:

  //! Default ctor.
  poly_Triangle()
  {
    hVertices[0] = hVertices[1] = hVertices[2] = poly_VertexHandle();
    hEdges[0]    = hEdges[1]    = hEdges[2]    = poly_EdgeHandle();
    iFlags       = Flag_None;
    iFaceRef     = Mobius_InvalidHandleIndex;
  }

  //! Ctor accepting the nodes of the triangle. The nodes should be enumerated in
  //! ccw order looking from the outside of the surrounded solid.
  //! \param[in] hv0 first vertex of the triangle.
  //! \param[in] hv1 second vertex of the triangle.
  //! \param[in] hv2 third vertex of the triangle.
  poly_Triangle(const poly_VertexHandle hv0,
                const poly_VertexHandle hv1,
                const poly_VertexHandle hv2)
  {
    hVertices[0] = hv0;
    hVertices[1] = hv1;
    hVertices[2] = hv2;
    iFlags       = Flag_None;
    iFaceRef     = Mobius_InvalidHandleIndex;

    // Edges are empty.
    hEdges[0] = hEdges[1] = hEdges[2] = poly_EdgeHandle();
  }

  //! Ctor accepting the nodes of the triangle and the back reference to a CAD face.
  //! The nodes should be enumerated in ccw order looking from the outside of the
  //! surrounded solid.
  //! \param[in] hv0 first vertex of the triangle.
  //! \param[in] hv1 second vertex of the triangle.
  //! \param[in] hv2 third vertex of the triangle.
  //! \param[in] ref back reference to set.
  poly_Triangle(const poly_VertexHandle hv0,
                const poly_VertexHandle hv1,
                const poly_VertexHandle hv2,
                const int               ref)
  {
    hVertices[0] = hv0;
    hVertices[1] = hv1;
    hVertices[2] = hv2;
    iFlags       = Flag_None;
    iFaceRef     = ref;

    // Edges are empty.
    hEdges[0] = hEdges[1] = hEdges[2] = poly_EdgeHandle();
  }

public:

  //! Makes a copy of this triangle without traits.
  poly_Triangle<> CopyWithoutTraits() const
  {
    poly_Triangle<> res;

    for ( size_t i = 0; i < 3; ++i )
      res.hEdges[i] = this->hEdges[i];

    for ( size_t i = 0; i < 3; ++i )
      res.hVertices[i] = this->hVertices[i];

    res.iFaceRef = this->iFaceRef;
    res.iFlags   = this->iFlags;
    return res;
  }

  //! Returns vertex handles defining the triangle.
  //! \param[out] hv0 1-st vertex.
  //! \param[out] hv1 2-nd vertex.
  //! \param[out] hv2 3-rd vertex.
  void GetVertices(poly_VertexHandle& hv0,
                   poly_VertexHandle& hv1,
                   poly_VertexHandle& hv2) const
  {
    hv0 = hVertices[0];
    hv1 = hVertices[1];
    hv2 = hVertices[2];
  }

  //! Sets the back reference to a CAD face (if any).
  //! \param[in] faceRef the face reference to set.
  void SetFaceRef(const int faceRef)
  {
    iFaceRef = faceRef;
  }

  //! \returns the back reference to a CAD face (if any).
  int GetFaceRef() const
  {
    return iFaceRef;
  }

  //! \return the associated flags.
  int GetFlags() const
  {
    return iFlags;
  }

  //! \return non-const reference to the attributes.
  int& ChangeFlags()
  {
    return iFlags;
  }

  //! Flags this triangle as deleted.
  void SetDeleted()
  {
    iFlags |= Flag_Deleted;
  }

  //! \return true if this triangle is marked as deleted.
  bool IsDeleted() const
  {
    return (iFlags & Flag_Deleted) > 0;
  }

public:

  poly_VertexHandle hVertices[3]; //!< Handles to the vertices.
  poly_EdgeHandle   hEdges[3];    //!< Handles to the edges.
  int               iFlags;       //!< Flags.
  int               iFaceRef;     //!< Back reference to the CAD face.
  Traits            traits;       //!< Traits for customization.

};

}

#endif
