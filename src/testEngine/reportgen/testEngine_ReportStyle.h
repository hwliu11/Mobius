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

#ifndef testEngine_ReportStyle_HeaderFile
#define testEngine_ReportStyle_HeaderFile

// testEngine includes
#include <mobius/testEngine_ReportTag.h>

namespace mobius {

//! Structure embodying HTML styling support.
class testEngine_ReportStyle
{
public:

  //! Color components.
  struct Color
  {
    int R; //!< Red.
    int G; //!< Green.
    int B; //!< Blue.

    //! Default constructor.
    Color() : R(0), G(0), B(0) {}

    //! Complete constructor.
    //! \param r [in] red component in range [0;255].
    //! \param g [in] green component in range [0;255].
    //! \param b [in] blue component in range [0;255].
    Color(const int r, const int g, const int b) : R(r), G(g), B(b) {}

    //! Copy constructor.
    //! \param color [in] another color to copy data from.
    Color(const Color& color) {this->operator=(color);}

    //! Assignment operator.
    //! \param color [in] another color to copy data from.
    //! \return this.
    Color& operator=(const Color& color) {R = color.R; G = color.G; B = color.B; return *this;}
  };

public:

  //! Enumerates supported font weights.
  enum FontWeight
  {
    FW_Bold,
    FW_Normal
  };

  //! Enumerates supported font styles.
  enum FontStyle
  {
    FS_Normal,
    FS_Italic
  };

  //! Enumerates supported text alignment.
  enum TextAlign
  {
    TA_Center,
    TA_Justify,
    TA_Left,
    TA_Right
  };

  //! Enumerates supported vertical alignment.
  enum VertAlign
  {
    VA_Bottom,
    VA_Middle,
    VA_Top
  };

public:

  mobiusTestEngine_EXPORT
    testEngine_ReportStyle();

  mobiusTestEngine_EXPORT
    testEngine_ReportStyle(std::nullptr_t);

  mobiusTestEngine_EXPORT
    testEngine_ReportStyle(const testEngine_ReportStyle& copy);

  mobiusTestEngine_EXPORT virtual
    ~testEngine_ReportStyle();

public:

  mobiusTestEngine_EXPORT testEngine_ReportStyle&
    operator=(const testEngine_ReportStyle& other);

  mobiusTestEngine_EXPORT bool
    IsNull() const;

public:

  mobiusTestEngine_EXPORT void
    SetBorder(const int px);

  mobiusTestEngine_EXPORT void
    SetBgColor(const Color& color);

  mobiusTestEngine_EXPORT void
    SetColor(const Color& color);

  mobiusTestEngine_EXPORT void
    SetFontFamily(const std::string& family);

  mobiusTestEngine_EXPORT void
    SetFontSize(const int px);

  mobiusTestEngine_EXPORT void
    SetFontWeight(const FontWeight weight);

  mobiusTestEngine_EXPORT void
    SetFontStyle(const FontStyle style);

  mobiusTestEngine_EXPORT void
    SetTextAlignment(const TextAlign align);

  mobiusTestEngine_EXPORT void
    SetVerticalAlignment(const VertAlign align);

  mobiusTestEngine_EXPORT void
    SetPadding(const int px);

  mobiusTestEngine_EXPORT void
    SetWidth(const std::string& width);

public:

  mobiusTestEngine_EXPORT virtual void
    ApplyStyles(testEngine_ReportTag& tag) const;

  mobiusTestEngine_EXPORT virtual std::string
    MakeDescriptor() const;

protected:

  mobiusTestEngine_EXPORT const char*
    getFontStyle(const FontStyle style) const;

  mobiusTestEngine_EXPORT const char*
    getFontWeight(const FontWeight weight) const;

  mobiusTestEngine_EXPORT const char*
    getTextAlignCSS(const TextAlign align) const;

  mobiusTestEngine_EXPORT const char*
    getVertAlignCSS(const VertAlign align) const;

  mobiusTestEngine_EXPORT void
    nullify(const bool release = false);

  mobiusTestEngine_EXPORT void
    deepCopy(const testEngine_ReportStyle& other);

private:

  //! Style descriptor.
  struct TProps
  {
    bool IsBorder()     const { return (pPxBorder   != nullptr); }
    bool IsBgColor()    const { return (pBgColor    != nullptr); }
    bool IsColor()      const { return (pColor      != nullptr); }
    bool IsFontFamily() const { return (pFontFamily != nullptr); }
    bool IsFontSize()   const { return (pPxFont     != nullptr); }
    bool IsFontWeight() const { return (pFontWeight != nullptr); }
    bool IsFontStyle()  const { return (pFontStyle  != nullptr); }
    bool IsTextAlign()  const { return (pTextAlign  != nullptr); }
    bool IsVertAlign()  const { return (pVertAlign  != nullptr); }
    bool IsPadding()    const { return (pPxPadding  != nullptr); }
    bool IsWidth()      const { return (pWidth      != nullptr); }

    //! Default constructor.
    TProps() : pPxBorder   (nullptr),
               pPxPadding  (nullptr),
               pPxFont     (nullptr),
               pBgColor    (nullptr),
               pColor      (nullptr),
               pFontFamily (nullptr),
               pFontWeight (nullptr),
               pFontStyle  (nullptr),
               pTextAlign  (nullptr),
               pVertAlign  (nullptr),
               pWidth      (nullptr) {}

    //! Returns true if all properties are nulls.
    //! \return true/false.
    bool IsNull() const
    {
      if ( !pPxBorder &&
           !pPxPadding &&
           !pPxFont &&
           !pBgColor &&
           !pColor &&
           !pFontFamily &&
           !pFontWeight &&
           !pFontStyle &&
           !pTextAlign &&
           !pVertAlign &&
           !pWidth )
        return true;

      return false;
    }

    int*         pPxBorder;
    int*         pPxPadding;
    int*         pPxFont;
    Color*       pBgColor;
    Color*       pColor;
    std::string* pFontFamily;
    FontWeight*  pFontWeight;
    FontStyle*   pFontStyle;
    TextAlign*   pTextAlign;
    VertAlign*   pVertAlign;
    std::string* pWidth;
  };

  //! Actual properties.
  TProps m_props;
};

};

#endif
