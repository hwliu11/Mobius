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

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Triangle element.
class poly_Triangle
{
public:

  //! Default ctor.
  mobiusPoly_EXPORT
    poly_Triangle();

  //! Ctor accepting the nodes of the triangle. The nodes should be enumerated in
  //! ccw order looking from the outside of the surrounded solid.
  //! \param[in] hv0 first vertex of the triangle.
  //! \param[in] hv1 second vertex of the triangle.
  //! \param[in] hv2 third vertex of the triangle.
  mobiusPoly_EXPORT
    poly_Triangle(const poly_VertexHandle hv0,
                  const poly_VertexHandle hv1,
                  const poly_VertexHandle hv2);

  //! Ctor accepting the nodes of the triangle and the back reference to a CAD face.
  //! The nodes should be enumerated in ccw order looking from the outside of the
  //! surrounded solid.
  //! \param[in] hv0 first vertex of the triangle.
  //! \param[in] hv1 second vertex of the triangle.
  //! \param[in] hv2 third vertex of the triangle.
  //! \param[in] ref back reference to set.
  mobiusPoly_EXPORT
    poly_Triangle(const poly_VertexHandle hv0,
                  const poly_VertexHandle hv1,
                  const poly_VertexHandle hv2,
                  const int               ref);

public:

  //! Returns vertex handles defining the triangle.
  //! \param[out] hv0 1-st vertex.
  //! \param[out] hv1 2-nd vertex.
  //! \param[out] hv2 3-rd vertex.
  void GetVertices(poly_VertexHandle& hv0,
                   poly_VertexHandle& hv1,
                   poly_VertexHandle& hv2)
  {
    hv0 = m_hVertices[0];
    hv1 = m_hVertices[1];
    hv2 = m_hVertices[2];
  }

  //! Sets the back reference to a CAD face (if any).
  //! \param[in] faceRef the face reference to set.
  void SetFaceRef(const int faceRef)
  {
    m_iFaceRef = faceRef;
  }

  //! \returns the back reference to a CAD face (if any).
  int GetFaceRef() const
  {
    return m_iFaceRef;
  }

  //! \return the associated flags.
  int GetFlags() const
  {
    return m_iFlags;
  }

  //! \return non-const reference to the attributes.
  int& ChangeAttributes()
  {
    return m_iFlags;
  }

  //! Flags this triangle as deleted.
  void SetDeleted()
  {
    m_iFlags |= Flag_Deleted;
  }

  //! \return true if this triangle is marked as deleted.
  bool IsDeleted() const
  {
    return (m_iFlags & Flag_Deleted) > 0;
  }

protected:

  poly_VertexHandle m_hVertices[3]; //!< Handles to the vertices.
  int               m_iFlags;       //!< Flags.
  int               m_iFaceRef;     //!< Back reference to the CAD face.

};

}

#endif
