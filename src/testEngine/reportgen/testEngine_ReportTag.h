//-----------------------------------------------------------------------------
// Created on: 26 November 2013
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

}

#endif
