//-----------------------------------------------------------------------------
// Created on: 26 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef testEngine_ReportTagFactory_HeaderFile
#define testEngine_ReportTagFactory_HeaderFile

// testEngine includes
#include <mobius/testEngine_ReportTag.h>

namespace mobius {

//! Mark-up factory for reports. Encapsulates routines connected with
//! preparation of tags. As rendering process is kind of streaming, only
//! one tag is accessible at each moment. The tag's working copy is
//! initialized by attributes & styles required in the current report
//! rendering context.
class testEngine_ReportTagFactory
{
public:

  mobiusTestEngine_EXPORT
    testEngine_ReportTagFactory();

  mobiusTestEngine_EXPORT
    ~testEngine_ReportTagFactory();

public:

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Html(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Head(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Meta();

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Body(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    P(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    B(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Sub(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Br();

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Table(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    TBody(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Tr(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Td(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Th(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Font(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    H1(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    H2(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    H3(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    H4(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Hr();

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Style(const bool isOpen = true);

  mobiusTestEngine_EXPORT testEngine_ReportTag&
    Center(const bool isOpen = true);

private:

  static void setPairedTag(testEngine_ReportTag& tag,
                           const char* baseName,
                           const bool isOpen);

  static void setUnpairedTag(testEngine_ReportTag& tag,
                             const char* baseName);

private:

  testEngine_ReportTag m_tag; //!< Simple tag for streaming.

};

};

#endif
