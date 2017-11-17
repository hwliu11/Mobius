//-----------------------------------------------------------------------------
// Created on: 03 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef core_Smoothness_HeaderFile
#define core_Smoothness_HeaderFile

// core includes
#include <mobius/core.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Order of smoothness.
enum core_Smoothness
{
  Smoothness_C0 = 0, //!< C0.
  Smoothness_C1,     //!< C1.
  Smoothness_C2,     //!< C2.
  Smoothness_C3,     //!< C3.
  Smoothness_CN      //!< CN.
};

};

#endif
