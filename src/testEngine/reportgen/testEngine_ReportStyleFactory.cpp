//-----------------------------------------------------------------------------
// Created on: 26 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// testEngine includes
#include <mobius/testEngine_ReportStyleFactory.h>

//! CSS style with background color suitable for highlighting of "bad" data.
//! \return requested style.
mobius::testEngine_ReportStyle mobius::testEngine_ReportStyleFactory::Bad()
{
  testEngine_ReportStyle ResStyle;
  ResStyle.SetBgColor( testEngine_ReportStyle::Color(240, 100, 100) );
  return ResStyle;
}

//! CSS style with background color suitable for highlighting of "good" data.
//! \return requested style.
mobius::testEngine_ReportStyle mobius::testEngine_ReportStyleFactory::Good()
{
  testEngine_ReportStyle ResStyle;
  ResStyle.SetBgColor( testEngine_ReportStyle::Color(190, 235, 100) );
  return ResStyle;
}
