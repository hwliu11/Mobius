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

// Own include
#include <mobius/core_OBJECT.h>

//-----------------------------------------------------------------------------

//! Default constructor.
mobius::core_OBJECT::core_OBJECT() : m_iRefCount(0)
{}

//-----------------------------------------------------------------------------

//! Destructor.
mobius::core_OBJECT::~core_OBJECT()
{}

//-----------------------------------------------------------------------------

//! Increments reference counter.
void mobius::core_OBJECT::IncRef()
{
#if defined(WIN32) || defined(_WIN32)
  InterlockedIncrement(&m_iRefCount);
#else
  static tbb::spin_mutex MUTEX;
  MUTEX.lock();
  ++m_iRefCount;
  MUTEX.unlock();
#endif
}

//-----------------------------------------------------------------------------

//! Decrements reference counter.
void mobius::core_OBJECT::DecRef()
{
#if defined(WIN32) || defined(_WIN32)
  LONG aRefCount = (LONG) m_iRefCount;
  m_iRefCount = (int) InterlockedDecrement(&aRefCount);
#else
  static tbb::spin_mutex MUTEX;
  MUTEX.lock();
  --m_iRefCount;
  MUTEX.unlock();
#endif
}

//-----------------------------------------------------------------------------

//! Returns the current number of references to the object.
//! \return number of references.
int mobius::core_OBJECT::NbRefs() const
{
  return m_iRefCount;
}

//-----------------------------------------------------------------------------

//! Dumps this object to the passed string stream.
//! \param stream [in/out] target stream.
void mobius::core_OBJECT::Dump(std::ostream* out) const
{
  *out << "core_OBJECT";
}
