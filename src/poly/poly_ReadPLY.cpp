//-----------------------------------------------------------------------------
// Created on: 05 October 2018
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

// Own include
#include <mobius/poly_ReadPLY.h>

// Standard includes
#include <fstream>
#include <iterator>

//-----------------------------------------------------------------------------

bool mobius::poly_ReadPLY::Perform(const std::string& filename)
{
  std::ifstream FILE(filename);
  //
  if ( !FILE.is_open() )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot open PLY file.");
    return false;
  }

  // Create container for mesh.
  m_mesh = new poly_Mesh;

  // Working variables.
  int nVertices = 0, nFaces = 0;
  bool isHeaderPassed = false, areVerticesPassed = false, areFacesPassed = false;

  // Loop over the file.
  while ( !FILE.eof() )
  {
    char str[1024];
    FILE.getline(str, 1024);

    // Save tokens to vector.
    std::vector<std::string> tokens;
    std::istringstream iss(str);
    std::copy( std::istream_iterator<std::string>(iss),
               std::istream_iterator<std::string>(),
               std::back_inserter< std::vector<std::string> >(tokens) );

    if ( !tokens.size() )
      continue;

    // Skip header.
    if ( !isHeaderPassed && (tokens[0] == "end_header") )
    {
      isHeaderPassed = true;
      continue;
    }
    else if ( !isHeaderPassed )
    {
      // ...
      // Process header
      // ...

      if ( tokens.size() == 3 )
      {
        int* iTarget = NULL;
        if ( tokens[1] == "vertex" )
          iTarget = &nVertices;
        else if ( tokens[1] == "face" )
          iTarget = &nFaces;

        if ( iTarget )
        {
          *iTarget = atoi( tokens[2].c_str() );
          iTarget = NULL;
        }
      }
    }
    else if ( isHeaderPassed )
    {
      // ...
      // Process vertices
      // ...

      if ( !areVerticesPassed && tokens.size() >= 3 )
      {
        const double xyz[3] = { atof( tokens[0].c_str() ),
                                atof( tokens[1].c_str() ),
                                atof( tokens[2].c_str() ) };

        m_mesh->AddVertex( core_XYZ(xyz[0], xyz[1], xyz[2]) );

        if ( m_mesh->GetNumVertices() == nVertices )
        {
          areVerticesPassed = true;
          continue;
        }
      }

      // ...
      // Process faces
      // ...

      if ( areVerticesPassed && !areFacesPassed && tokens.size() > 3)
      {
        int nNodes = atoi( tokens[0].c_str() );

        if ( nNodes == 3 )
          m_mesh->AddTriangle( poly_VertexHandle(atoi( tokens[1].c_str() ) + 1),
                               poly_VertexHandle(atoi( tokens[2].c_str() ) + 1),
                               poly_VertexHandle(atoi( tokens[3].c_str() ) + 1) );
        else if ( nNodes == 4 )
          m_mesh->AddQuad( poly_VertexHandle(atoi( tokens[1].c_str() ) + 1),
                           poly_VertexHandle(atoi( tokens[2].c_str() ) + 1),
                           poly_VertexHandle(atoi( tokens[3].c_str() ) + 1),
                           poly_VertexHandle(atoi( tokens[4].c_str() ) + 1) );
      }
    }
  } // Until EOF.

  FILE.close();
  return true;
}
