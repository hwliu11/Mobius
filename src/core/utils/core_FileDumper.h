//-----------------------------------------------------------------------------
// Created on: 04 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef core_FileDumper_HeaderFile
#define core_FileDumper_HeaderFile

// core includes
#include <mobius/core.h>

// STD includes
#include <fstream>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Utility class providing functionality for dumping of algorithmic data
//! to ASCII files in different manners.
class core_FileDumper
{
public:

  mobiusCore_EXPORT
    core_FileDumper();

  mobiusCore_EXPORT
    core_FileDumper(const std::string& filename);

  mobiusCore_EXPORT
    ~core_FileDumper();

public:

  mobiusCore_EXPORT bool
    Open(const std::string& filename);

  mobiusCore_EXPORT void
    Dump(const std::string& msg);

  mobiusCore_EXPORT void
    Dump(const int val,
         const std::string& msg = "");

  mobiusCore_EXPORT void
    Dump(const double val,
         const std::string& msg = "");

  mobiusCore_EXPORT void
    Dump(const bool val,
         const std::string& msg = "");

  mobiusCore_EXPORT void
    Dump(const double* arr,
         const int numElems,
         const std::string& msg = "");

  mobiusCore_EXPORT void
    Dump(const double* mx,
         const int numRows,
         const int numCols,
         const std::string& msg = "");

private:

  std::ofstream m_FILE; //!< File stream.

};

};

#endif
