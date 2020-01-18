//-----------------------------------------------------------------------------
// Created on: 13 October 2013
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
#include <mobius/core.h>

#define BUFSIZE 1000

//-----------------------------------------------------------------------------

//! Returns value of MOBIUS_TEST_DATA environment variable. This variable is used to
//! refer to the directory containing all data files playing as inputs for
//! unit tests.
//! \return value of MOBIUS_TEST_DATA variable.
std::string mobius::core::env::MobiusTestData()
{
  return std::getenv(MOBIUS_TEST_DATA);
}

//-----------------------------------------------------------------------------

//! Returns value of MOBIUS_TEST_DUMPING environment variable. This variable is
//! used to refer to the directory containing all temporary files utilized by
//! unit tests.
//! \return value of MOBIUS_TEST_DUMPING variable.
std::string mobius::core::env::MobiusTestDumping()
{
  return std::getenv(MOBIUS_TEST_DUMPING);
}

//-----------------------------------------------------------------------------

//! Returns value of MOBIUS_TEST_DESCR environment variable. This variable is used
//! to refer to the directory containing all temporary files utilized by unit
//! tests.
//! \return value of MOBIUS_TEST_DESCR variable.
std::string mobius::core::env::MobiusTestDescr()
{
  return std::getenv(MOBIUS_TEST_DESCR);
}

//-----------------------------------------------------------------------------

//! Returns value of the environment variable with the passed name.
//! \param VarName [in] variable name.
//! \return value of the variable.
std::string
  mobius::core::env::GetVariable(const char* VarName)
{
  return std::getenv(VarName);
}

//-----------------------------------------------------------------------------

std::string
  mobius::core::str::slashed(const std::string& strIN)
{
  if ( !strIN.length() )
    return strIN;

  char c = strIN.at(strIN.length() - 1);
  if ( c == *SLASH_STR )
    return strIN;

  std::string strOUT(strIN);
  strOUT.append(SLASH_STR);
  return strOUT;
}

//-----------------------------------------------------------------------------

int mobius::core::str::extract_int(const std::string& str)
{
  std::string temp;
  int number = 0;

  for ( unsigned int i = 0; i < str.size(); ++i )
  {
    // Iterate the string to find the first "number" character.
    // If found, start another loop to extract it
    // and then break the current one
    // thus extracting the first encountered numeric block
    if ( isdigit(str[i]) )
    {
      for ( unsigned int a = i; a < str.size(); ++a )
      {
        temp += str[a];
      }
      break; // The first numeric block is extracted
    }
  }

  std::istringstream stream(temp);
  stream >> number;
  return number;
}

//-----------------------------------------------------------------------------

bool mobius::core::str::are_equal(const std::string& str_1,
                                  const std::string& str_2)
{
  return !str_1.compare(str_2);
}

//-----------------------------------------------------------------------------

bool mobius::core::str::is_number(const std::string& str)
{
  char* cnv;
  strtod(str.c_str(), &cnv);
  return cnv != str.data();
}

//-----------------------------------------------------------------------------

void mobius::core::str::replace_all(std::string&       str,
                                    const std::string& what,
                                    const std::string& with)
{
  for ( size_t pos = 0; ; pos += with.length() )
  {
    pos = str.find(what, pos); // Locate the substring to replace
    if ( pos == std::string::npos )
      break; // Not found

    // Replace by erasing and inserting
    str.erase( pos, what.length() );
    str.insert(pos, with);
  }
}

//-----------------------------------------------------------------------------

void mobius::core::str::split(const std::string&        source_str,
                              const std::string&        delim_str,
                              std::vector<std::string>& result)
{
  // Initialize collection of strings to split
  std::vector<std::string> chunks;
  chunks.push_back(source_str);

  // Split by each delimiter consequently
  for ( size_t delim_idx = 0; delim_idx < delim_str.length(); ++delim_idx )
  {
    std::vector<std::string> new_chunks;
    const char delim = delim_str[delim_idx];

    // Split each chunk
    for ( size_t chunk_idx = 0; chunk_idx < chunks.size(); ++chunk_idx )
    {
      const std::string& source = chunks[chunk_idx];
      std::string::size_type currPos = 0, prevPos = 0;
      while ( (currPos = source.find(delim, prevPos) ) != std::string::npos )
      {
        std::string item = source.substr(prevPos, currPos - prevPos);
        if ( item.size() > 0 )
        {
          new_chunks.push_back(item);
        }
        prevPos = currPos + 1;
      }
      new_chunks.push_back( source.substr(prevPos) );
    }

    // Set new collection of chunks for splitting by the next delimiter
    chunks = new_chunks;
  }

  // Set result
  result = chunks;
}

//-----------------------------------------------------------------------------

void mobius::core::str::join(const std::vector<std::string>& vector,
                             std::string&                    result,
                             const int                       from,
                             const int                       until)
{
  std::string res;
  const int last = ( (until == -1) ? (int) vector.size() : until );
  for ( int idx = from; idx < last; ++idx )
    res += vector[idx];

  result = res;
}

//-----------------------------------------------------------------------------

std::string
  mobius::core::str::substr(const std::string& source,
                            const int          idx_F,
                            const int          length)
{
  return source.substr(idx_F, length);
}

//-----------------------------------------------------------------------------

bool mobius::core::str::getKeyValue(const std::string& source,
                                    const std::string& delim,
                                    const std::string& key,
                                    std::string&       value)
{
  std::vector<std::string> chunks;
  split(source, delim, chunks);

  // Try to find key
  if ( chunks.size() != 2 )
    return false;

  const size_t pos = chunks[0].find(key, 0); // Locate the substring to replace.
  if ( pos == std::string::npos )
    return false; // Key not found.

  value = chunks[1];
  return true;
}

//-----------------------------------------------------------------------------

int mobius::core::hasher::HashCode(const int val, const int upper)
{
  return ( (val & 0x7fffffff) % upper ) + 1;
}

//-----------------------------------------------------------------------------

int mobius::core::hasher::HashCode(const double val, const int upper)
{
  union {
    double R;
    int    I[2];
  } U;
  U.R = val;
  return HashCode( (U.I[0] ^ U.I[1]), upper );
}
