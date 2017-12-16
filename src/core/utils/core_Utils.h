//-----------------------------------------------------------------------------
// Created on: 13 October 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef core_Utils_HeaderFile
#define core_Utils_HeaderFile

// core includes
#include <mobius/core.h>

namespace mobius {

//! Auxiliary functions.
namespace core_Utils
{
  //! Functions for working with environment.
  namespace Env
  {
    mobiusCore_EXPORT
      std::string QrData();

    mobiusCore_EXPORT
      std::string QrDumping();

    mobiusCore_EXPORT
      std::string QrDescr();

    mobiusCore_EXPORT
      std::string GetVariable(const char* VarName);
  }

  //! Functions for working with strings.
  namespace Str
  {
    mobiusCore_EXPORT
      std::string Slashed(const std::string& strIN);
  }

};

};

#endif
