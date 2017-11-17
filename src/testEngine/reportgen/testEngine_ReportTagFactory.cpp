//-----------------------------------------------------------------------------
// Created on: 26 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/testEngine_ReportTagFactory.h>

//! Default constructor.
mobius::testEngine_ReportTagFactory::testEngine_ReportTagFactory()
{
}

//! Destructor.
mobius::testEngine_ReportTagFactory::~testEngine_ReportTagFactory()
{
}

//! Returns HTML tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Html(const bool isOpen)
{
  setPairedTag(m_tag, "html", isOpen);
  return m_tag;
}

//! Returns HEAD tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Head(const bool isOpen)
{
  setPairedTag(m_tag, "head", isOpen);
  return m_tag;
}

//! Returns META tag.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Meta()
{
  setUnpairedTag(m_tag, "meta");
  return m_tag;
}

//! Returns BODY tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Body(const bool isOpen)
{
  setPairedTag(m_tag, "body", isOpen);
  return m_tag;
}

//! Returns P tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::P(const bool isOpen)
{
  setPairedTag(m_tag, "p", isOpen);
  return m_tag;
}

//! Returns B tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::B(const bool isOpen)
{
  setPairedTag(m_tag, "b", isOpen);
  return m_tag;
}

//! Returns SUB tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Sub(const bool isOpen)
{
  setPairedTag(m_tag, "sub", isOpen);
  return m_tag;
}

//! Returns BR tag.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Br()
{
  setUnpairedTag(m_tag, "br");
  return m_tag;
}

//! Returns TABLE tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Table(const bool isOpen)
{
  setPairedTag(m_tag, "table", isOpen);
  return m_tag;
}

//! Returns TBODY tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::TBody(const bool isOpen)
{
  setPairedTag(m_tag, "tbody", isOpen);
  return m_tag;
}

//! Returns TR tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Tr(const bool isOpen)
{
  setPairedTag(m_tag, "tr", isOpen);
  return m_tag;
}

//! Returns TD tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Td(const bool isOpen)
{
  setPairedTag(m_tag, "td", isOpen);
  return m_tag;
}

//! Returns TH tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Th(const bool isOpen)
{
  setPairedTag(m_tag, "th", isOpen);
  return m_tag;
}

//! Returns FONT tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Font(const bool isOpen)
{
  setPairedTag(m_tag, "font", isOpen);
  return m_tag;
}

//! Returns H1 tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::H1(const bool isOpen)
{
  setPairedTag(m_tag, "h1", isOpen);
  return m_tag;
}

//! Returns H2 tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::H2(const bool isOpen)
{
  setPairedTag(m_tag, "h2", isOpen);
  return m_tag;
}

//! Returns H3 tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::H3(const bool isOpen)
{
  setPairedTag(m_tag, "h3", isOpen);
  return m_tag;
}

//! Returns H4 tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::H4(const bool isOpen)
{
  setPairedTag(m_tag, "h4", isOpen);
  return m_tag;
}

//! Returns HR tag.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Hr()
{
  setUnpairedTag(m_tag, "hr");
  return m_tag;
}

//! Returns Style tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Style(const bool isOpen)
{
  setPairedTag(m_tag, "style", isOpen);
  return m_tag;
}

//! Returns Center tag, opening or closing, depending on the passed
//! parameter.
//! \param isOpen [in] indicates whether the resulting tag is opening or
//!        closing.
//! \return requested tag.
mobius::testEngine_ReportTag& mobius::testEngine_ReportTagFactory::Center(const bool isOpen)
{
  setPairedTag(m_tag, "center", isOpen);
  return m_tag;
}

//! Prepares the passed tag as a paired one with the given name and
//! openness indicator.
//! \param tag [in/out] tag to prepare.
//! \param baseName [in] base name to set for the tag being prepared.
//! \param isOpen [in] indicates whether the tag being prepared is opening
//!        or closing.
void mobius::testEngine_ReportTagFactory::setPairedTag(testEngine_ReportTag& tag,
                                                       const char* baseName,
                                                       const bool isOpen)
{
  tag.Release();
  tag.SetIsPaired(true);
  tag.SetIsClosed(!isOpen);
  tag.SetBase(baseName);
}

//! Prepares the passed tag as single (non-paired) one with the given name.
//! \param tag [in/out] tag to prepare.
//! \param baseName [in] base name to set for the tag being prepared.
void mobius::testEngine_ReportTagFactory::setUnpairedTag(testEngine_ReportTag& tag,
                                                         const char* baseName)
{
  tag.Release();
  tag.SetIsPaired(false);
  tag.SetBase(baseName);
}
