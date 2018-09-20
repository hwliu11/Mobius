//-----------------------------------------------------------------------------
// Created on: 20 September 2018
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

#ifndef poly_ReadSTL_HeaderFile
#define poly_ReadSTL_HeaderFile

// Poly includes
#include <mobius/poly_Mesh.h>

// Core includes
#include <mobius/core_IAlgorithm.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Utility to read STL files. The nodes with equal coordinates are merged
//! automatically on reading.
//!
//! This reader is the slightly adapted version of STL reader taken from
//! OpenCascade 7.3.1 (RWStl package).
class poly_ReadSTL : public core_IAlgorithm
{
public:

  //! Ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  poly_ReadSTL(core_ProgressEntry progress = NULL,
               core_PlotterEntry  plotter  = NULL) : core_IAlgorithm(progress, plotter) {}

public:

  //! \return constructed mesh.
  const ptr<poly_Mesh>& GetResult() const
  {
    return m_mesh;
  }

public:

  //! Reads either ASCII or binary file.
  //!
  //! \param[in] filename file to read.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    Perform(const std::string& filename);

  //! Add a vertex with the given coordinates to the resulting mesh data
  //! structure. This method does not perform any coincidence tests.
  //!
  //! \param[in] P coordinates of the vertex to add.
  //! \return handle of the newly added vertex.
  mobiusPoly_EXPORT poly_VertexHandle
    AddVertex(const core_XYZ& P);

  //! Add a triangle composed of the given vertices to the resulting mesh data
  //! structure.
  //!
  //! \param[in] hv0 handle of the first vertex.
  //! \param[in] hv1 handle of the second vertex.
  //! \param[in] hv2 handle of the third vertex.
  mobiusPoly_EXPORT void
    AddTriangle(const poly_VertexHandle hV0,
                const poly_VertexHandle hV1,
                const poly_VertexHandle hV2);

protected:

  //! Checks whether the stream is an ASCII STL file by analysis of the
  //! first bytes (~200). This function attempts to put back the read
  //! symbols to the stream which thus must support ungetc().
  //!
  //! \param[in] stream stream in question.
  //! \return true if the stream seems to contain ASCII STL.
  mobiusPoly_EXPORT bool
    isAscii(std::istream& stream);

  //! Reads STL data from binary stream. The stream must be opened in binary
  //! mode. Stops after reading the number of triangles recorded in the file header.
  //!
  //! \param[in] stream stream in question.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    readBinary(std::istream& stream);

  //! Reads STL data from ASCII stream. The stream can be opened either in
  //! binary or in ASCII mode. Stops at the position specified by `untilPos`
  //! or when eof is reached or when keyword "endsolid" is found. Empty lines
  //! are not supported and will lead to reading failure. If `untilPos` is
  //! non-zero, reads not further than until the specified position.
  //!
  //! \param[in] stream stream in question.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    readAscii(std::istream&        stream,
              const std::streampos untilPos);

protected:

  ptr<poly_Mesh> m_mesh; //!< Mesh data structure.

};

};

#endif
