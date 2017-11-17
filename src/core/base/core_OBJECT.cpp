//-----------------------------------------------------------------------------
// Created on: 28 Aug 2012
// Created by: Sergey SLYADNEV
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
void mobius::core_OBJECT::Dump(std::stringstream& stream) const
{
  stream << "core_OBJECT";
}
