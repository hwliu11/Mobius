//-----------------------------------------------------------------------------
// Created on: 26 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
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
    testEngine_ReportStyle(int);

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
    bool IsBorder()     const { return (pPxBorder   != NULL); }
    bool IsBgColor()    const { return (pBgColor    != NULL); }
    bool IsColor()      const { return (pColor      != NULL); }
    bool IsFontFamily() const { return (pFontFamily != NULL); }
    bool IsFontSize()   const { return (pPxFont     != NULL); }
    bool IsFontWeight() const { return (pFontWeight != NULL); }
    bool IsFontStyle()  const { return (pFontStyle  != NULL); }
    bool IsTextAlign()  const { return (pTextAlign  != NULL); }
    bool IsVertAlign()  const { return (pVertAlign  != NULL); }
    bool IsPadding()    const { return (pPxPadding  != NULL); }
    bool IsWidth()      const { return (pWidth      != NULL); }

    //! Default constructor.
    TProps() : pPxBorder(NULL),
               pPxPadding(NULL),
               pPxFont(NULL),
               pBgColor(NULL),
               pColor(NULL),
               pFontFamily(NULL),
               pFontWeight(NULL),
               pFontStyle(NULL),
               pTextAlign(NULL),
               pVertAlign(NULL),
               pWidth(NULL) {}

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
