//-----------------------------------------------------------------------------
// Created on: 03 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_SectionPatch_HeaderFile
#define geom_SectionPatch_HeaderFile

// Geometry includes
#include <mobius/geom_Surface.h>
#include <mobius/geom_VectorField.h>

// STL includes
#include <map>

namespace mobius {

//! Surface and constraints.
class geom_SectionPatch : public core_OBJECT
{
public:

  geom_SectionPatch() : core_OBJECT(), ID(-1) {}

  int                                ID;   //!< ID of the patch.
  std::map< int, Ptr<vector_field> > D1;   //!< D1 by sections.
  std::map< int, Ptr<vector_field> > D2;   //!< D2 by sections.
  Ptr<geom_Surface>                  Surf; //!< Reconstructed surface.

  void Add_D1(const int sct_ID, Ptr<vector_field> D1_vectors)
  {
    D1.insert( std::pair< int, Ptr<vector_field> >(sct_ID, D1_vectors) );
  }

  void Add_D2(const int sct_ID, Ptr<vector_field> D2_vectors)
  {
    D2.insert( std::pair< int, Ptr<vector_field> >(sct_ID, D2_vectors) );
  }

  Ptr<vector_field> D1_sct(const int sct_ID)
  {
    std::map< int, Ptr<vector_field> >::iterator it = D1.find(sct_ID);
    if ( it == D1.end() )
      return NULL;

    return it->second;
  }

  Ptr<vector_field> D2_sct(const int sct_ID)
  {
    std::map< int, Ptr<vector_field> >::iterator it = D2.find(sct_ID);
    if ( it == D2.end() )
      return NULL;

    return it->second;
  }

};

//! Handy shortcut for section patch type name.
typedef geom_SectionPatch spatch;

};

#endif
