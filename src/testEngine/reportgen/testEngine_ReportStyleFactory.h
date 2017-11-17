//-----------------------------------------------------------------------------
// Created on: 26 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef testEngine_ReportStyleFactory_HeaderFile
#define testEngine_ReportStyleFactory_HeaderFile

// testEngine includes
#include <mobius/testEngine_ReportStyle.h>

namespace mobius {

//! Factory for commonly used CSS styles.
class testEngine_ReportStyleFactory
{
public:

  mobiusTestEngine_EXPORT static testEngine_ReportStyle
    Bad();

  mobiusTestEngine_EXPORT static testEngine_ReportStyle
    Good();

private:

  testEngine_ReportStyleFactory() {}
  testEngine_ReportStyleFactory(const testEngine_ReportStyleFactory&) {}
  void operator=(const testEngine_ReportStyleFactory&) {}

};

};

#endif
