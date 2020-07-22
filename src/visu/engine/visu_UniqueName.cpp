//-----------------------------------------------------------------------------
// Created on: 01 September 2014
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
#include <mobius/visu_UniqueName.h>

// STD includes
#include <algorithm>

//! Generates unique name trying to choose its numerical index by
//! consulting the provided collection of already occupied names. If other
//! names have some numerical indices at the ends, the maximal one is chosen
//! and incremented. Otherwise, the generated name gets index 1.
//! \param names_occupied [in] occupied names.
//! \param base_name [in] base name.
//! \return new unique name.
std::string
  mobius::visu_UniqueName::Generate(const std::vector<std::string>& names_occupied,
                                    const std::string&              base_name)
{
  int idxMax = -INT_MAX;
  const size_t numOccupied = names_occupied.size();
  for ( size_t r = 0; r < numOccupied; ++r )
  {
    const std::string& nextOccupied = names_occupied[r];

    // Split name by a predefined separator
    std::vector<std::string> chunks;
    core::str::split(nextOccupied, "_", chunks);

    if ( chunks.size() < 2 )
      continue; // We are searching for <name>_<index> format

    // Get base name
    std::string next_base_name;
    core::str::join(chunks, next_base_name, 0, (int) (chunks.size() - 1));

    if ( next_base_name != base_name )
      continue; // Skip different names

    // Extract next index
    const int idx = core::str::to_number<int>(chunks[chunks.size() - 1]) + 1;
    idxMax = std::max(idxMax, idx);
  }

  if ( idxMax == -INT_MAX )
    idxMax = 1;

  return base_name + "_" + core::str::to_string<int>(idxMax);
}
