//-----------------------------------------------------------------------------
// Created on: 15 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef core_StringBuffer_HeaderFile
#define core_StringBuffer_HeaderFile

// core includes
#include <mobius/core_Ptr.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Convenient class for wrapping standard string with streaming
//! functionality.
class core_StringBuffer : public OBJECT
{
public:

  std::string String; //!< Internal string.

public:

  //! Streaming operator performing concatenation of two strings.
  //! \param str [in] string to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBuffer& operator<<(const std::string& str)
  {
    String += str;
    return *this;
  }

  //! Streaming operator performing concatenation of the stored string with
  //! the passed character.
  //! \param chr [in] character to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBuffer& operator<<(const char chr)
  {
    String += chr;
    return *this;
  }

  //! Streaming operator performing concatenation of two strings.
  //! \param str [in] string to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBuffer& operator<<(const char* str)
  {
    String += str;
    return *this;
  }

  //! Streaming operator performing concatenation of buffered string with
  //! the passed number (also converted to string).
  //! \param val [in] value to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBuffer& operator<<(const int val)
  {
    String += core::to_string<int>(val);
    return *this;
  }

  //! Streaming operator performing concatenation of buffered string with
  //! the passed size value (also converted to string).
  //! \param val [in] value to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBuffer& operator<<(const size_t val)
  {
    String += core::to_string<int>( (int) val );
    return *this;
  }

  //! Streaming operator performing concatenation of buffered string with
  //! the passed number (also converted to string).
  //! \param val [in] value to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBuffer& operator<<(const double val)
  {
    String += core::to_string<double>(val);
    return *this;
  }

  //! Cleans up the buffer.
  void Clear()
  {
    String.clear();
  }

  //! Conversion operator.
  //! \return wrapped string.
  operator std::string()
  {
    return String;
  }

  //! Returns length of the currently buffered string.
  //! \return buffered length.
  size_t Length() const
  {
    return String.length();
  }

};

//-----------------------------------------------------------------------------
// Null-safe wrapper for tool for dumping
//-----------------------------------------------------------------------------

//! Null-safe wrapper for String Buffer.
class core_StringBufferSentry
{
public:

  core_Ptr<core_StringBuffer> Buffer; //!< Wrapped String Buffer.

public:

  //! Constructor which allows assignments like
  //! 'core_StringBufferSentry e = 0'.
  core_StringBufferSentry(const int = 0)
  {}

  //! Constructor accepting String Buffer.
  //! \param Buff [in] String Buffer to wrap.
  core_StringBufferSentry(const core_Ptr<core_StringBuffer>& Buff)
  {
    Buffer = Buff;
  }

  //! Constructor accepting String Buffer.
  //! \param Buff [in] String Buffer to wrap.
  core_StringBufferSentry(const core_StringBuffer& Buff)
  {
    Buffer = new core_StringBuffer();
    Buffer->String = Buff.String;
  }

public:

  //! Streaming operator performing concatenation of two strings.
  //! \param str [in] string to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBufferSentry& operator<<(const std::string& str)
  {
    if ( !Buffer.IsNull() )
      Buffer->operator<<(str);

    return *this;
  }

  //! Streaming operator performing concatenation of two strings.
  //! \param str [in] string to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBufferSentry& operator<<(const char* str)
  {
    if ( !Buffer.IsNull() )
      Buffer->operator<<(str);

    return *this;
  }

  //! Streaming operator performing concatenation of buffered string with
  //! the passed number (also converted to string).
  //! \param val [in] value to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBufferSentry& operator<<(const int val)
  {
    if ( !Buffer.IsNull() )
      Buffer->operator<<(val);

    return *this;
  }

  //! Streaming operator performing concatenation of buffered string with
  //! the passed number (also converted to string).
  //! \param val [in] value to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBufferSentry& operator<<(const double val)
  {
    if ( !Buffer.IsNull() )
      Buffer->operator<<(val);

    return *this;
  }

  //! Cleans up the buffer.
  void Clear()
  {
    if ( !Buffer.IsNull() )
      Buffer->Clear();
  }

  //! Conversion operator.
  //! \return string.
  operator std::string()
  {
    if ( !Buffer.IsNull() )
      return Buffer->operator std::string();

    return "";
  }

  //! Returns length of the currently buffered string.
  //! \return buffered length.
  size_t Length() const
  {
    if ( !Buffer.IsNull() )
      return Buffer->Length();

    return 0;
  }


  //! Returns true if wrapped buffer is null.
  //! \return true/false.
  bool IsNull() const
  {
    return Buffer.IsNull();
  }

};

};

#endif
