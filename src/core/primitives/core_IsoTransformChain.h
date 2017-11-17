//-----------------------------------------------------------------------------
// Created on: 05 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef core_IsoTransformChain_HeaderFile
#define core_IsoTransformChain_HeaderFile

// core includes
#include <mobius/core_IsoTransform.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Utility class representing a series of isometric transformations.
class core_IsoTransformChain
{
// Construction & destruction:
public:

  mobiusCore_EXPORT
    core_IsoTransformChain();

  mobiusCore_EXPORT
    core_IsoTransformChain(const core_IsoTransform& transform);

public:

  mobiusCore_EXPORT core_IsoTransformChain&
    operator<<(const core_IsoTransform& transform);

  mobiusCore_EXPORT core_IsoTransformChain&
    operator<<(const core_IsoTransformChain& transformChain);

  mobiusCore_EXPORT core_XYZ
    Apply(const core_XYZ& coord) const;

public:

  //! List of transformations.
  std::vector<core_IsoTransform> m_tr_list;

};

};

#endif
