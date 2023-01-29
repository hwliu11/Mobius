//-----------------------------------------------------------------------------
// Created on: 28 Aug 2012
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

#ifndef core_OBJECT_HeaderFile
#define core_OBJECT_HeaderFile

#if defined(WIN32) || defined(_WIN32)
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  //
  #include <windows.h>
#endif

// core includes
#include <mobius/core.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Base class for all Mobius objects manipulated by Mobius smart pointers --
//! the Handles. These smart pointers are intrusive, meaning that in order
//! to have your own object being controlled via Handle, you need to extend
//! this Mobius-specific class. All such objects must be allocated in a heap
//! memory.
//!
//! The entire mechanism is based on reference counting. Each object has its
//! own counter incremented each time a new Handle appears, and decremented
//! once a Handle dies. Eventually, when the last Handle goes out of scope
//! (Handles must be allocated in the automatic memory) and reference counter
//! gets value of zero, the object is deleted.
//!
//! Note that this mechanism has no protection against cyclic dependencies
//! between Handles.
//!
//! \todo core_OBJECT class is incomplete
//! \todo unit tests
class core_OBJECT
{
// Construction & destruction:
public:

  mobiusCore_EXPORT core_OBJECT();
  mobiusCore_EXPORT virtual ~core_OBJECT();

// Reference counting:
public:

  mobiusCore_EXPORT void IncRef();
  mobiusCore_EXPORT void DecRef();
  mobiusCore_EXPORT int  NbRefs() const;

public:

  mobiusCore_EXPORT virtual void
    Dump(std::ostream* out) const;

  mobiusCore_EXPORT const std::string&
    GetName() const;

  mobiusCore_EXPORT void
    SetName(const std::string& name);

  mobiusCore_EXPORT bool
    HasName() const;

private:

#if defined(WIN32) || defined(_WIN32)
  volatile LONG m_iRefCount;
#else
  int m_iRefCount;
#endif

  std::string m_name; //!< Optional name.

};

//-----------------------------------------------------------------------------
// Handy shortcuts
//-----------------------------------------------------------------------------

//! shortcut for Mobius shared pointer.
typedef core_OBJECT OBJECT;

}

#endif
