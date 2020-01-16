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

#ifndef testEngine_ReportRenderer_HeaderFile
#define testEngine_ReportRenderer_HeaderFile

// testEngine includes
#include <mobius/testEngine_ReportStyle.h>
#include <mobius/testEngine_ReportTagFactory.h>

// Core includes
#include <mobius/core_OBJECT.h>

// Standard includes
#include <string>

namespace mobius {

//! Utility for generating reports i  n HTML format. Supplies a predefined set
//! of atomic layout portions to be streamed one-by-one, producing the
//! resulting one-string report's content so.
//!
//! HTML 4.01 in XHTML form is supported.
//!
//! This class provides a mechanism to apply CSS on the HTML tags being
//! generated.
class testEngine_ReportRenderer : public core_OBJECT
{
public:

  //! Convenient shortcut for this.
  typedef testEngine_ReportRenderer* THAT;

  //! Rendering log.
  struct RenderLog
  {
    int NbTables;     //!< Number of rendered tables
    int NbTableRows;  //!< Number of rendered table rows
    int NbTableCells; //!< Number of rendered table cells

    //! Constructor.
    RenderLog()
    {
      this->CleanLog();
    }

    //! Nullifies the logged statistics.
    void CleanLog()
    {
      NbTables = NbTableRows = NbTableCells = 0;
    }
  };

public:

  mobiusTestEngine_EXPORT
    testEngine_ReportRenderer();

  mobiusTestEngine_EXPORT
    ~testEngine_ReportRenderer();

public:

  mobiusTestEngine_EXPORT std::string
    Flush(const bool isLogDropped = false);

  mobiusTestEngine_EXPORT void
    Reset(const bool isLogDropped = false);

  mobiusTestEngine_EXPORT bool
    DumpToFile(const std::string& filename);

  mobiusTestEngine_EXPORT const RenderLog&
    GetRenderLog() const;

// ADD functions:
public:

  mobiusTestEngine_EXPORT THAT
    AddDoctype();

  mobiusTestEngine_EXPORT THAT
    AddSymboledText(const char* text,
                    const bool isScaled = true);

  mobiusTestEngine_EXPORT THAT
    AddErrorText(const char* text);

  mobiusTestEngine_EXPORT THAT
    AddImportantText(const char* text);

  mobiusTestEngine_EXPORT THAT
    AddImportantText(const std::string& text);

  mobiusTestEngine_EXPORT THAT
    AddImportantText(const int num);

  mobiusTestEngine_EXPORT THAT
    AddImportantText(const double num);

  mobiusTestEngine_EXPORT THAT
    AddText(const char* text,
            const bool doRemoveNewLines = true);

  mobiusTestEngine_EXPORT THAT
    AddText(const std::string& text,
            const bool doRemoveNewLines = true);

  mobiusTestEngine_EXPORT THAT
    AddText(const int num);

  mobiusTestEngine_EXPORT THAT
    AddText(const double num);

  mobiusTestEngine_EXPORT THAT
    AddEquality(const char* var,
                const char* value,
                const char* units = 0);

  mobiusTestEngine_EXPORT THAT
    AddTextWithSubs(const char* text,
                    const char* subs,
                    const bool isTextSymboled = false,
                    const bool isSubsSymboled = false);

  mobiusTestEngine_EXPORT THAT
    AddMeta();

  mobiusTestEngine_EXPORT THAT
    AddEmptyTableCell();

  mobiusTestEngine_EXPORT THAT
    AddEmptyTableHCell();

  mobiusTestEngine_EXPORT THAT
    AddTableHCell(const char* text);

  mobiusTestEngine_EXPORT THAT
    AddTableCell(const char* text);

  mobiusTestEngine_EXPORT THAT
    AddParagraph(const char* text);

  mobiusTestEngine_EXPORT THAT
    AddHr();

  mobiusTestEngine_EXPORT THAT
    AddClass(const std::string& dotName,
             const testEngine_ReportStyle& style);

// START/END functions:
public:

  mobiusTestEngine_EXPORT THAT
    StartHtml();

  mobiusTestEngine_EXPORT THAT
    EndHtml();

  mobiusTestEngine_EXPORT THAT
    StartHeader();

  mobiusTestEngine_EXPORT THAT
    EndHeader();

  mobiusTestEngine_EXPORT THAT
    StartBody(const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartBody(const std::string& className);

  mobiusTestEngine_EXPORT THAT
    EndBody();

  mobiusTestEngine_EXPORT THAT
    StartTable(const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartTable(const std::string& className);

  mobiusTestEngine_EXPORT THAT
    EndTable();

  mobiusTestEngine_EXPORT THAT
    StartTableRow(const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartTableRow(const std::string& className);

  mobiusTestEngine_EXPORT THAT
    EndTableRow();

  mobiusTestEngine_EXPORT THAT
    StartTableCell(const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartTableCell(const std::string& className);

  mobiusTestEngine_EXPORT THAT
    StartColSpanTableCell(const int colSpan = 1,
                          const testEngine_ReportStyle& theStyle = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartColSpanTableCell(const int colSpan,
                          const std::string& className);

  mobiusTestEngine_EXPORT THAT
    StartColSpanTableHCell(const int colSpan = 1,
                           const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartColSpanTableHCell(const int colSpan,
                           const std::string& className);

  mobiusTestEngine_EXPORT THAT
    StartRowSpanTableCell(const int rowSpan = 1,
                          const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartRowSpanTableCell(const int rowSpan,
                          const std::string& className);

  mobiusTestEngine_EXPORT THAT
    StartRowSpanTableHCell(const int rowSpan = 1,
                           const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartRowSpanTableHCell(const int rowSpan,
                           const std::string& className);

  mobiusTestEngine_EXPORT THAT
    EndTableCell();

  mobiusTestEngine_EXPORT THAT
    StartTableHCell(const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartTableHCell(const std::string& className);

  mobiusTestEngine_EXPORT THAT
    EndTableHCell();

  mobiusTestEngine_EXPORT THAT
    StartParagraph(const testEngine_ReportStyle& style = nullptr);

  mobiusTestEngine_EXPORT THAT
    StartParagraph(const std::string& className);

  mobiusTestEngine_EXPORT THAT
    EndParagraph();

  mobiusTestEngine_EXPORT THAT
    StartSubscript();

  mobiusTestEngine_EXPORT THAT
    EndSubscript();

  mobiusTestEngine_EXPORT THAT
    StartH1();

  mobiusTestEngine_EXPORT THAT
    EndH1();

  mobiusTestEngine_EXPORT THAT
    StartStyle();

  mobiusTestEngine_EXPORT THAT
    EndStyle();

  mobiusTestEngine_EXPORT THAT
    StartCenter();

  mobiusTestEngine_EXPORT THAT
    EndCenter();

// Additional functions:
public:

  mobiusTestEngine_EXPORT THAT
    BreakRow();

private:

  static void
    removeInconsistent(std::string& str);

private:

  //! Tag factory to generate HTML mark-up.
  testEngine_ReportTagFactory m_tagFactory;

  //! Rendering buffer.
  std::string m_buffer;

  //! Rendering log.
  RenderLog m_log;

};

};

#endif
