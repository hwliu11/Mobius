//-----------------------------------------------------------------------------
// Created on: 29 August 2019
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
#include <mobius/poly_ReadOBJ.h>

// Standard includes
#include <fstream>

//-----------------------------------------------------------------------------

bool mobius::poly_ReadOBJ::Perform(const std::string& filename)
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

  // Loop over the file.
  while ( !FILE.eof() )
  {
    char str[1024];
    FILE.getline(str, 1024);

    // Split.
    std::vector<std::string> tokensRaw, tokens;
    core::str::split(str, " ", tokensRaw);

    // Remove empty.
    for ( size_t k = 0; k < tokensRaw.size(); ++k )
      if ( !tokensRaw[k].empty() )
        tokens.push_back(tokensRaw[k]);

    if ( !tokens.size() )
      continue;

    // Add vertex.
    if ( tokens[0] == "v" )
    {
      const double xyz[3] = { atof( tokens[1].c_str() ),
                              atof( tokens[2].c_str() ),
                              atof( tokens[3].c_str() ) };

      m_mesh->AddVertex( core_XYZ(xyz[0], xyz[1], xyz[2]) );
    }

    // Add face.
    if ( tokens[0] == "f" )
    {
      const int nTokens = int( tokens.size() );
      std::vector<int> nodes;

      for ( int k = 1; k < nTokens; ++k )
      {
        // Split.
        std::vector<std::string> nodeTokens;
        core::str::split(tokens[k], "//", nodeTokens);

        nodes.push_back( atoi( nodeTokens[0].c_str() ) - 1 );
      }

      // Add element.
      if ( nTokens == 4 )
        m_mesh->AddTriangle( poly_VertexHandle(nodes[0]),
                             poly_VertexHandle(nodes[1]),
                             poly_VertexHandle(nodes[2]) );
      else if ( nTokens == 5 )
        m_mesh->AddQuad( poly_VertexHandle(nodes[0]),
                         poly_VertexHandle(nodes[1]),
                         poly_VertexHandle(nodes[2]),
                         poly_VertexHandle(nodes[3]) );
    }
  } // Until EOF.

  FILE.close();
  return true;
}
