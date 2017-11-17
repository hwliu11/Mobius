//-----------------------------------------------------------------------------
// Created on: 26 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef testEngine_ReportTag_HeaderFile
#define testEngine_ReportTag_HeaderFile

// testEngine includes
#include <mobius/testEngine.h>

// Core includes
#include <mobius/core.h>

// STD includes
#include <vector>

namespace mobius {

//! Base class representing HTML tag for test report.
class testEngine_ReportTag
{
public:

  mobiusTestEngine_EXPORT
    testEngine_ReportTag();

  mobiusTestEngine_EXPORT
    ~testEngine_ReportTag();

public:

  mobiusTestEngine_EXPORT void
    Release();

  mobiusTestEngine_EXPORT void
    SetIsPaired(const bool isPaired);

  mobiusTestEngine_EXPORT bool
    IsPaired() const;

  mobiusTestEngine_EXPORT void
    SetIsClosed(const bool isClosed);

  mobiusTestEngine_EXPORT bool
    IsClosed() const;

  mobiusTestEngine_EXPORT void
    SetBase(const char* base);

  mobiusTestEngine_EXPORT void
    SetBase(const std::string& base);

  mobiusTestEngine_EXPORT void
    AddStyle(const char* styleName,
             const char* styleValue);

  mobiusTestEngine_EXPORT void
    AddStyle(const std::string& styleName,
             const std::string& styleValue);

  mobiusTestEngine_EXPORT void
    AddAttribute(const char* attrName,
                 const char* attrValue);

  mobiusTestEngine_EXPORT void
    AddAttribute(const std::string& attrName,
                 const std::string& attrValue);

  mobiusTestEngine_EXPORT std::string
    Result() const;

private:

  static void eraseString(std::string& string);
  static void eraseStrings(std::vector<std::string>& stringList);

private:

  std::string              m_base;           //!< Base name.
  std::string              m_style;          //!< Associated style.
  std::vector<std::string> m_attributes;     //!< Associated attributes.
  bool                     m_bIsInitialized; //!< Indicates whether this tag is initialized or not.
  bool                     m_bIsPaired;      //!< Indicates whether this tag is paired or not.
  bool                     m_bIsStyled;      //!< Indicates whether this tag is styled or not.
  bool                     m_bIsClosed;      //!< Indicates whether this tag is closed or not.

};

};

#endif
