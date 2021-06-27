//-----------------------------------------------------------------------------
// Created on: 18 September 2018
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

#ifndef test_Mesh_HeaderFile
#define test_Mesh_HeaderFile

// Test includes
#include <mobius/test_CaseIDs.h>

// TestEngine includes
#include <mobius/testEngine_TestCase.h>

// Poly includes
#include <mobius/poly_Mesh.h>

namespace mobius {

//! Test functions for mesh data structure.
class test_Mesh : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_Poly_Mesh;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_Mesh";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "structure";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param[out] functions output collection of pointers.
  static void Functions(MobiusTestFunctions& functions)
  {
    functions << &testCreateVertex
              << &testReadSTL01
              << &testReadSTL02
              << &testReadSTL03
              << &testReadSTL04
              << &testReadPLY01
              << &testReadPLY02
              << &refineTriangleByMidpoint
              << &computeLinks
              << &flipEdges01
              << &flipEdges02
              << &flipEdges03
              << &flipEdges04
              << &flipEdges05
    ; // Put semicolon here for convenient adding new functions above ;)
  }

private:

  static bool verifyMeshContents(const t_ptr<poly_Mesh>& mesh,
                                 const int               refNumVertices,
                                 const int               refNumEdges,
                                 const int               refNumTriangles,
                                 const int               refNumQuads);

  static t_ptr<poly_Mesh>
    readSTL(const char* filenameShort);

  static outcome testReadSTL(const int   funcID,
                             const char* filenameShort,
                             const int   refNumVertices,
                             const int   refNumEdges,
                             const int   refNumTriangles);

  static outcome testReadPLY(const int   funcID,
                             const char* filenameShort,
                             const int   refNumVertices,
                             const int   refNumEdges,
                             const int   refNumTriangles,
                             const int   refNumQuads);

private:

  static outcome testCreateVertex         (const int funcID);
  static outcome testReadSTL01            (const int funcID);
  static outcome testReadSTL02            (const int funcID);
  static outcome testReadSTL03            (const int funcID);
  static outcome testReadSTL04            (const int funcID);
  static outcome testReadPLY01            (const int funcID);
  static outcome testReadPLY02            (const int funcID);
  static outcome refineTriangleByMidpoint (const int funcID);
  static outcome computeLinks             (const int funcID);
  static outcome flipEdges01              (const int funcID);
  static outcome flipEdges02              (const int funcID);
  static outcome flipEdges03              (const int funcID);
  static outcome flipEdges04              (const int funcID);
  static outcome flipEdges05              (const int funcID);

};

};

#endif
