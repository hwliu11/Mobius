//-----------------------------------------------------------------------------
// Created on: 15 September 2014
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

#ifndef core_StringBuffer_HeaderFile
#define core_StringBuffer_HeaderFile

// core includes
#include <mobius/core_Ptr.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Convenient class for wrapping standard string with streaming
//! functionality.
class core_StringBuffer : public core_OBJECT
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
    String += core::str::to_string<int>(val);
    return *this;
  }

  //! Streaming operator performing concatenation of buffered string with
  //! the passed size value (also converted to string).
  //! \param val [in] value to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBuffer& operator<<(const size_t val)
  {
    String += core::str::to_string<int>( (int) val );
    return *this;
  }

  //! Streaming operator performing concatenation of buffered string with
  //! the passed number (also converted to string).
  //! \param val [in] value to append to the buffer.
  //! \return this instance for subsequent streaming.
  core_StringBuffer& operator<<(const double val)
  {
    String += core::str::to_string<double>(val);
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

}

#endif
