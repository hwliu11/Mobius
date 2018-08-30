//-----------------------------------------------------------------------------
// Created on: 17 June 2018
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
#include <mobius/core_JSON.h>

//-----------------------------------------------------------------------------

//! Constructor accepting JSON string to process.
//! \param[in] json string representing JSON to process.
mobius::core_JSON::core_JSON(const std::string& json)
: m_json(json)
{}

//-----------------------------------------------------------------------------

//! Destructor.
mobius::core_JSON::~core_JSON()
{
}

//-----------------------------------------------------------------------------

bool mobius::core_JSON::ExtractBlockForKey(const std::string& key,
                                           std::string&       block) const
{
  // Find position of the key word.
  size_t pos  = 0;
  bool   stop = false;
  //
  while ( !stop )
  {
    pos = m_json.find(key, pos); // Locate the key.

    if ( pos == std::string::npos )
      return false; // Key not found.

    // We need a whole word match, so we check special characters here.
    if ( pos > 0 && pos < m_json.length() )
      if ( m_json[pos-1] != ' '  &&
           m_json[pos-1] != '\t' &&
           m_json[pos+1] != ' '  &&
           m_json[pos-1] != '\t' &&
           m_json[pos-1] != ':' )
      {
        pos++;
        continue;
      }

    stop = true;
    pos += key.length();
  }

  // Take the string after the found key.
  std::string tmpBlock = m_json.substr(pos);

  // Check if there is a sub-structure.
  size_t subStructureBegin = 0,
         subStructureEnd   = 0,
         firstSeparatorPos = 0,
         firstCommaPos     = 0,
         subStructureCount = 0;
  //
  for ( size_t k = 0; k < tmpBlock.size(); ++k )
  {
    // Skip all characters which can occur between the key and curled bracket.
    // ...

    if ( tmpBlock[k] == ' '  ||
         tmpBlock[k] == '\n' ||
         tmpBlock[k] == '\r' ||
         tmpBlock[k] == '\t' )
      continue;

    if ( tmpBlock[k] == ':' )
    {
      if ( !firstSeparatorPos )
        firstSeparatorPos = k;

      continue;
    }

    if ( tmpBlock[k] == ',' )
    {
      if ( !firstCommaPos )
        firstCommaPos = k;

      if ( !subStructureBegin )
        break; // Do not proceed if comma is found while sub-structure never began.

      continue;
    }

    if ( tmpBlock[k] == '{' || tmpBlock[k] == '[' )
    {
      if ( !subStructureBegin )
        subStructureBegin = k;

      subStructureCount++;
      continue;
    }

    if ( tmpBlock[k] == '}' || tmpBlock[k] == ']' )
    {
      subStructureEnd = k;
      subStructureCount--;

      if ( !subStructureCount ) // To process nested brackets.
        break;
    }
  }

  if ( subStructureEnd > subStructureBegin ) // Extract sub-structure.
  {
    block = tmpBlock.substr(subStructureBegin + 1, subStructureEnd - subStructureBegin - 1);
  }
  else if ( firstCommaPos > firstSeparatorPos ) // Extract inline data.
  {
    block = tmpBlock.substr(firstSeparatorPos + 1, firstCommaPos - firstSeparatorPos - 1);
  }
  else
    return false;

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::core_JSON::ExtractVector1d(const std::string&   keyword,
                                        std::vector<adouble>& vector) const
{
  std::string block;
  this->ExtractBlockForKey(keyword, block);

  std::vector<std::string> chunks;
  core::str::split(block, ",", chunks);

  for ( size_t k = 0; k < chunks.size(); ++k )
  {
    if ( !core::str::is_number(chunks[k]) )
      return false;

    vector.push_back( core::str::to_number<adouble>(chunks[k]) );
  }

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::core_JSON::ExtractVector3d(const std::string& keyword,
                                        std::vector<xyz>&  vector) const
{
  std::string block;
  this->ExtractBlockForKey(keyword, block);

  std::vector<std::string> chunks;
  core::str::split(block, "][", chunks);

  // Extract 3-coordinate tuples.
  for ( size_t k = 0; k < chunks.size(); ++k )
  {
    if ( !core::str::is_number(chunks[k]) )
      continue;

    // Extract coordinates.
    const std::string& coordStr = chunks[k];
    std::vector<std::string> coordChunks;
    core::str::split(coordStr, ",", coordChunks);

    if ( coordChunks.size() != 3 )
      return false;

    xyz P( core::str::to_number<adouble>(coordChunks[0], 0),
           core::str::to_number<adouble>(coordChunks[1], 0),
           core::str::to_number<adouble>(coordChunks[2], 0) );
    //
    vector.push_back(P);
  }

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::core_JSON::ExtractGrid3d(const std::string&               keyword,
                                      std::vector< std::vector<xyz> >& vector) const
{
  std::string block;
  this->ExtractBlockForKey(keyword, block);

  std::vector<std::string> rowChunks;
  core::str::split(block, ":", rowChunks);

  for ( size_t k = 0; k < rowChunks.size(); ++k )
  {
    std::vector<std::string> tupleChunks;
    core::str::split(rowChunks[k], "][", tupleChunks);

    if ( tupleChunks.size() < 3 )
      continue;

    std::vector<xyz> row;
    for ( size_t j = 0; j < tupleChunks.size(); ++j )
    {
      std::vector<std::string> coordChunks;
      core::str::split(tupleChunks[j], ",", coordChunks);

      if ( coordChunks.size() != 3 )
        continue;

      xyz P( core::str::to_number<adouble>(coordChunks[0], 0),
             core::str::to_number<adouble>(coordChunks[1], 0),
             core::str::to_number<adouble>(coordChunks[2], 0) );
      //
      row.push_back(P);
    }
    //
    vector.push_back(row);
  }

  return true;
}
