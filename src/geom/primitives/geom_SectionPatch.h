//-----------------------------------------------------------------------------
// Created on: 03 March 2015
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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

#ifndef geom_SectionPatch_HeaderFile
#define geom_SectionPatch_HeaderFile

// Geometry includes
#include <mobius/geom_Surface.h>
#include <mobius/geom_VectorField.h>

// STL includes
#include <map>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Surface and constraints.
class geom_SectionPatch : public core_OBJECT
{
public:

  geom_SectionPatch() : core_OBJECT(), ID(-1) {}

  int                                ID;   //!< ID of the patch.
  std::map< int, ptr<vector_field> > D1;   //!< D1 by sections.
  std::map< int, ptr<vector_field> > D2;   //!< D2 by sections.
  ptr<geom_Surface>                  Surf; //!< Reconstructed surface.

  void Add_D1(const int sct_ID, ptr<vector_field> D1_vectors)
  {
    D1.insert( std::pair< int, ptr<vector_field> >(sct_ID, D1_vectors) );
  }

  void Add_D2(const int sct_ID, ptr<vector_field> D2_vectors)
  {
    D2.insert( std::pair< int, ptr<vector_field> >(sct_ID, D2_vectors) );
  }

  ptr<vector_field> D1_sct(const int sct_ID)
  {
    std::map< int, ptr<vector_field> >::iterator it = D1.find(sct_ID);
    if ( it == D1.end() )
      return NULL;

    return it->second;
  }

  ptr<vector_field> D2_sct(const int sct_ID)
  {
    std::map< int, ptr<vector_field> >::iterator it = D2.find(sct_ID);
    if ( it == D2.end() )
      return NULL;

    return it->second;
  }

};

//! Handy shortcut for section patch type name.
typedef geom_SectionPatch spatch;

};

#endif
