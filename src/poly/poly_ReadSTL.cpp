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

// Own include
#include <mobius/poly_ReadSTL.h>

// Core includes
#include <mobius/core_Precision.h>

// Standard includes
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <math.h>
#include <unordered_map>

//-----------------------------------------------------------------------------

// Macro to get 64-bit position of the file from streampos.
#ifdef _WIN32
  #define GETPOS(pos) pos.seekpos()
#else
  #define GETPOS(pos) ( (int64_t) pos )
#endif

//-----------------------------------------------------------------------------

namespace mobius
{
  // For binary STL.
  static const size_t STL_HEADER_SIZE   = 84;
  static const size_t STL_SIZEOF_FACET  = 50;
  static const size_t STL_MIN_FILE_SIZE = STL_HEADER_SIZE + STL_SIZEOF_FACET;

  //! Auxiliary tool for merging nodes during STL reading.
  class NodeMerger
  {
  public:

    //! Hasher for map.
    struct hasher
    {
      //! Computes hash code for the given coordinates.
      //! \param[in] coords coordinates in question.
      //! \return hash code.
      size_t operator()(const core_XYZ& coords) const
      {
        const double tmp = coords.X()*M_LN10 + coords.Y()*M_PI + coords.Z()*M_E;
        return core::hasher::HashCode(tmp, 1024);
      }
    };

    //! Equality checker for map.
    struct compare
    {
      //! Checks if the two passed points are equal.
      bool operator()(const core_XYZ& P1, const core_XYZ& P2) const 
      {
        return (P1 - P2).SquaredModulus() < core_Precision::SquaredResolution3D();
      }
    };

    //! Map storing coordinates for coincidence check.
    typedef std::unordered_map<core_XYZ,
                               poly_VertexHandle,
                               NodeMerger::hasher,
                               NodeMerger::compare> t_xyzVertexMap;

  public:

    //! Ctor.
    //! \param[in] pReader reader tool.
    NodeMerger(poly_ReadSTL* pReader) : m_pReader(pReader) {}

    //! Adds vertex trying to find if such coordinates have been already used.
    //! Coincidence check is perform by hasher.
    //! \param[in] x coordinate x.
    //! \param[in] y coordinate y.
    //! \param[in] z coordinate z.
    //! \return vertex handle (new or reused one).
    poly_VertexHandle AddVertex(const double x,
                                const double y,
                                const double z)
    {
      core_XYZ P(x, y, z);

      // Try to find such coordinates in a map.
      t_xyzVertexMap::const_iterator it = m_map.find(P);
      //
      if ( it != m_map.end() )
        return it->second;

      // Create a new vertex.
      poly_VertexHandle vh = m_pReader->AddVertex(P);
      m_map.insert( std::pair<core_XYZ, poly_VertexHandle>(P, vh) );
      return vh;
    }

  private:

    //! STL reader.
    poly_ReadSTL* m_pReader;

    //! Map of registered vertex coordinates.
    t_xyzVertexMap m_map;

  };

  //! Reads a Little Endian 32 bits float.
  static float readStlFloat(const char* pData)
  {
    // On little-endian platform, use plain cast.
    return *reinterpret_cast<const float*>(pData);
  }

  //! Reads a Little Endian 32 bits float.
  static core_XYZ readStlFloatVec3(const char* pData)
  {
    return core_XYZ( readStlFloat( pData ),
                     readStlFloat( pData + sizeof(float) ),
                     readStlFloat( pData + sizeof(float)*2 ) );
  }

}

//-----------------------------------------------------------------------------

bool mobius::poly_ReadSTL::Perform(const std::string& filename)
{
  // Open file for reading.
  std::filebuf fileBuff;
  fileBuff.open(filename, std::ios::in | std::ios::binary);
  //
  if ( !fileBuff.is_open() )
  {
    m_progress.SendLogMessage( MobiusErr(Normal) << "Cannot open file %1 for reading."
                                                 << filename.c_str() );
    return false;
  }

  // Create mesh data structure.
  m_mesh = new poly_Mesh;

  // Prepare buffer.
  std::istream buffer(&fileBuff);

  // Get length of file.
  buffer.seekg(0, buffer.end);
  std::streampos endPos = buffer.tellg();
  buffer.seekg(0, buffer.beg);

  // Binary STL files cannot be shorter than 134 bytes (80 bytes header +
  // 4 bytes facet count + 50 bytes for one facet); thus we assume the files
  // shorter than 134 are the ASCII files without probing (probing may bring
  // stream to fail state if eof is reached).
  const bool isAsciiFile = ( (size_t) endPos < STL_MIN_FILE_SIZE || this->isAscii(buffer) );

  while ( buffer.good() )
  {
    if ( isAsciiFile )
    {
      if ( !this->readAscii(buffer, endPos) )
        break;
    }
    else
    {
      if ( !this->readBinary(buffer) )
        break;
    }
    buffer >> std::ws; // Skip any white spaces.
  }
  return !buffer.fail();
}

//-----------------------------------------------------------------------------

mobius::poly_VertexHandle
  mobius::poly_ReadSTL::AddVertex(const core_XYZ& P)
{
  return m_mesh->AddVertex(P);
}

//-----------------------------------------------------------------------------

void mobius::poly_ReadSTL::AddTriangle(const poly_VertexHandle hV0,
                                       const poly_VertexHandle hV1,
                                       const poly_VertexHandle hV2)
{
  m_mesh->AddTriangle(hV0, hV1, hV2);
}

//-----------------------------------------------------------------------------

bool mobius::poly_ReadSTL::isAscii(std::istream& stream)
{
  // Read first 134 bytes to recognize file format.
  char buffer[STL_MIN_FILE_SIZE];
  std::streamsize nbRead = stream.read(buffer, STL_MIN_FILE_SIZE).gcount();

  // Put back the read symbols.
  for ( std::streamsize byteIter = nbRead; byteIter > 0; --byteIter )
    stream.unget();

  // If file is shorter than size of binary file with 1 facet, it must be ASCII.
  if ( nbRead < std::streamsize(STL_MIN_FILE_SIZE) )
    return true;

  // Otherwise, detect binary format by presence of non-ASCII symbols in the
  // first 128 bytes (note that binary STL file may start with the same bytes
  // "solid " as ASCII one).
  for ( int byteIter = 0; byteIter < nbRead; ++byteIter )
  {
    if ( (unsigned char) buffer[byteIter] > (unsigned char) '~' )
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------

static inline bool str_starts_with (const char* pStr, const char* pWord, int n)
{
  while ( isspace (*pStr) && *pStr != '\0' ) pStr++;
  return !::strncmp(pStr, pWord, n);
}

//-----------------------------------------------------------------------------

static bool ReadVertex(const char* pStr, double& x, double& y, double& z)
{
  const char *localStr = pStr;

  // Skip 'vertex'.
  while ( isspace( (unsigned char) *localStr) || isalpha( (unsigned char) *localStr) )
    ++localStr;

  // Read values.
  char *localEnd;
  x = strtod(localStr, &localEnd);
  y = strtod(localStr = localEnd, &localEnd);
  z = strtod(localStr = localEnd, &localEnd);

  return localEnd != localStr;
}

//-----------------------------------------------------------------------------

#pragma warning(disable : 4127) // This is for "while (1)" below...

bool mobius::poly_ReadSTL::readAscii(std::istream&        stream,
                                     const std::streampos untilPos)
{
  // Use seekpos() to get true 64-bit offset to enable
  // handling of large files (VS 2010 64-bit).
  const int64_t startPos = GETPOS( stream.tellg() );

  // Note: 1 is added to `untilPos` to be sure to read the last
  // symbol (relevant for files without eol at the end).
  const int64_t endPos = (untilPos > 0 ? 1 + GETPOS(untilPos) : INT64_MAX);

  // Skip header "solid ..."
  stream.ignore( (std::streamsize) (endPos - startPos), '\n' );
  //
  if ( !stream )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Premature end of file.");
    return false;
  }

  NodeMerger MergeTool(this);

  const int64_t LINELEN = 1024;
  int           nbLine  = 1;
  char          line1[LINELEN], line2[LINELEN], line3[LINELEN];
  //
  while ( 1 )
  {
    char facet[LINELEN], outer[LINELEN];

    stream.getline(facet, (std::streamsize) std::min(LINELEN, endPos - GETPOS(stream.tellg()))); // "facet normal nx ny nz"
    //
    if ( str_starts_with(facet, "endsolid", 8) )
    {
      // End of STL code.
      break;
    }

    stream.getline(outer, (std::streamsize) std::min(LINELEN, endPos - GETPOS(stream.tellg()))); // "outer loop"
    //
    if ( !str_starts_with(facet, "facet", 5) || !str_starts_with(outer, "outer", 5) )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Unexpected format of facet at line %1."
                                                  << (nbLine + 1));
      return false;
    }

    stream.getline(line1, (std::streamsize) std::min(LINELEN, endPos - GETPOS(stream.tellg())));
    stream.getline(line2, (std::streamsize) std::min(LINELEN, endPos - GETPOS(stream.tellg())));
    stream.getline(line3, (std::streamsize) std::min(LINELEN, endPos - GETPOS(stream.tellg())));

    // Stop reading if eof is reached; note that well-formatted file never
    // ends by the vertex line.
    if ( stream.eof() || GETPOS(stream.tellg()) >= endPos )
      break;

    if ( !stream )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Premature end of file.");
      return false;
    }
    nbLine += 5;

    double x1, y1, z1, x2, y2, z2, x3, y3, z3;
    //
    if ( !ReadVertex(line1, x1, y1, z1) ||
         !ReadVertex(line2, x2, y2, z2) ||
         !ReadVertex(line3, x3, y3, z3) )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot read vertex coordinates at line %1."
                                                  << nbLine);
      return false;
    }

    // Add triangle to mesh data structure.
    const poly_VertexHandle hv0 = ( m_bMergeNodes ? MergeTool.AddVertex(x1, y1, z1) : this->AddVertex( t_xyz(x1, y1, z1) ) );
    const poly_VertexHandle hv1 = ( m_bMergeNodes ? MergeTool.AddVertex(x2, y2, z2) : this->AddVertex( t_xyz(x2, y2, z2) ) );
    const poly_VertexHandle hv2 = ( m_bMergeNodes ? MergeTool.AddVertex(x3, y3, z3) : this->AddVertex( t_xyz(x3, y3, z3) ) );
    //
    if ( hv0 != hv1 && hv1 != hv2 && hv2 != hv0 )
      this->AddTriangle(hv0, hv1, hv2);

    stream.ignore( (std::streamsize) ( endPos - GETPOS( stream.tellg() ) ), '\n' ); // Skip "endloop".
    stream.ignore( (std::streamsize) ( endPos - GETPOS( stream.tellg() ) ), '\n' ); // Skip "endfacet".

    nbLine += 2;
  }

  return true;
}

#pragma warning(default : 4127)

//-----------------------------------------------------------------------------

bool mobius::poly_ReadSTL::readBinary(std::istream& stream)
{
  // Read file header first.
  char header[STL_HEADER_SIZE + 1];
  if ( stream.read(header, STL_HEADER_SIZE).gcount() != std::streamsize(STL_HEADER_SIZE) )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Corrupted binary file.");
    return false;
  }

  // Nnumber of facets is stored as 32-bit integer at position 80.
  const int nbFacets = *(int32_t*)(header + 80);

  NodeMerger MergeTool(this);

  // Do not trust the number of triangles which is coded in the file.
  // Sometimes the number is wrong, and with this technique we do not need
  // to swap endians for integer.
  int nbRead = 0;

  // Allocate buffer for 80 triangles.
  const int CHUNK_NBFACETS = 80;
  char buffer[STL_SIZEOF_FACET * CHUNK_NBFACETS];

  // Normal + 3 nodes + 2 extra bytes.
  const size_t vec3Size        = sizeof(float) * 3;
  const size_t faceDataLen     = vec3Size * 4 + 2;
  const char*  bufferPtr       = buffer;
  int          nbFacesInBuffer = 0;
  //
  for ( int nbFacetRead = 0;
        nbFacetRead < nbFacets;
        ++nbFacetRead, ++nbRead, --nbFacesInBuffer, bufferPtr += faceDataLen )
  {
    // Read more data.
    if ( nbFacesInBuffer <= 0 )
    {
      nbFacesInBuffer = std::min(CHUNK_NBFACETS, nbFacets - nbFacetRead);

      const std::streamsize dataToRead = nbFacesInBuffer * faceDataLen;
      //
      if ( stream.read(buffer, dataToRead).gcount() != dataToRead )
      {
        m_progress.SendLogMessage(MobiusErr(Normal) << "Reader failed.");
        return false;
      }
      bufferPtr = buffer;
    }

    // Get points from buffer.
    core_XYZ P1 = readStlFloatVec3(bufferPtr + vec3Size);
    core_XYZ P2 = readStlFloatVec3(bufferPtr + vec3Size * 2);
    core_XYZ P3 = readStlFloatVec3(bufferPtr + vec3Size * 3);

    // Add triangle to mesh data structure.
    const poly_VertexHandle hv0 = ( m_bMergeNodes ? MergeTool.AddVertex( P1.X(), P1.Y(), P1.Z() ) : this->AddVertex(P1) );
    const poly_VertexHandle hv1 = ( m_bMergeNodes ? MergeTool.AddVertex( P2.X(), P2.Y(), P2.Z() ) : this->AddVertex(P2) );
    const poly_VertexHandle hv2 = ( m_bMergeNodes ? MergeTool.AddVertex( P3.X(), P3.Y(), P3.Z() ) : this->AddVertex(P3) );
    //
    if ( hv0 != hv1 && hv1 != hv2 && hv2 != hv0 )
      this->AddTriangle(hv0, hv1, hv2);
  }

  return true;
}
