//-----------------------------------------------------------------------------
// Created on: 28 Aug 2012
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef core_OBJECT_HeaderFile
#define core_OBJECT_HeaderFile

#if defined(WIN32) || defined(_WIN32)
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
    Dump(std::stringstream& stream) const;

private:

#if defined(WIN32) || defined(_WIN32)
  volatile LONG m_iRefCount;
#else
  int m_iRefCount;
#endif

};

//-----------------------------------------------------------------------------
// Handy shortcuts
//-----------------------------------------------------------------------------

//! shortcut for Mobius shared pointer.
typedef core_OBJECT OBJECT;

};

#endif
