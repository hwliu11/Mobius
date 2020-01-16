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

#ifndef core_Ptr_HeaderFile
#define core_Ptr_HeaderFile

// core includes
#include <mobius/core_OBJECT.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! \todo provide description here
template <class SharedType>
class core_Ptr
{
// Construction & destruction:
public:

  //! Default constructor.
  core_Ptr<SharedType>()
  {
    m_pRef = nullptr;
  }

  //! Constructor accepting integer.
  core_Ptr<SharedType>(int)
  {
    m_pRef = nullptr;
  }

  //! Constructor accepting null pointer.
  core_Ptr<SharedType>(std::nullptr_t)
  {
    m_pRef = nullptr;
  }

  //! Constructor accepting the raw reference to the target object to wrap with
  //! a smart pointer.
  //! \param theRef [in] reference to the target object.
  core_Ptr<SharedType>(SharedType* theRef)
  {
    m_pRef = theRef;
    this->beginScope();
  }

  //! Constructor accepting the raw const reference to the target object
  //! to wrap with a smart pointer.
  //! \param theRef [in] reference to the target object.
  core_Ptr<SharedType>(const SharedType* theRef)
  {
    m_pRef = const_cast<SharedType*>(theRef);
    this->beginScope();
  }

  //! Constructor accepting the smart reference to the target object to share.
  //! \param theSmartRef [in] smart reference to the target object.
  core_Ptr<SharedType>(const core_Ptr<SharedType>& theSmartRef)
  {
    m_pRef = nullptr;
    this->operator=(theSmartRef);
  }

  //! Destructor.
  virtual ~core_Ptr()
  {
    this->endScope();
  }

public:

  //! Assignment operator accepting raw pointer.
  //! \param theRef [in] raw pointer.
  //! \return this instance.
  core_Ptr<SharedType>& operator=(SharedType* theRef)
  {
    this->endScope();
    m_pRef = theRef;
    this->beginScope();
    return *this;
  }

  //! Assignment operator accepting smart pointer.
  //! \param theSmartRef [in] smart pointer.
  //! \return this instance.
  core_Ptr<SharedType>& operator=(const core_Ptr<SharedType>& theSmartRef)
  {
    this->endScope();
    m_pRef = theSmartRef.m_pRef;
    this->beginScope();
    return *this;
  }

  //! Conversion operator.
  //! \return raw pointer.
  operator SharedType*()
  {
    return m_pRef;
  }

  //! Conversion operator.
  //! \return raw pointer.
  operator const SharedType*() const
  {
    return m_pRef;
  }

  //! Conversion operator from this smart pointer to the smart pointer
  //! holding a reference to the base type. This operator allows to pass
  //! smart pointers to derived classes in functions accepting smart
  //! pointers to base classes.
  template <class T2, typename = typename std::enable_if<std::is_base_of<T2, SharedType>::value>::type>
  operator const core_Ptr<T2>& () const
  {
    return reinterpret_cast<const core_Ptr<T2>&>(*this);
  }

  //! Nullifies smart pointer.
  //! \return true/false.
  void Nullify()
  {
    this->endScope();
  }

  //! Checks whether the actually wrapper pointer is null or not.
  //! \return true/false.
  bool IsNull() const
  {
    return m_pRef == nullptr;
  }

  //! Accessor for the wrapped pointer.
  //! \return wrapped pointer.
  SharedType* Access() const
  {
    return m_pRef;
  }

  //! Dereferencing operator.
  //! \return wrapped pointer.
  SharedType* operator->() const
  {
    return this->Access();
  }

  //! Dereferencing operator.
  SharedType& operator*() const
  {
    return *this->Access();
  }

public:

  //! Casts the passed pointer to the necessary type.
  //! \param theSmartRef [in] pointer to cast.
  //! \return casted pointer.
  static core_Ptr<SharedType> DownCast(const core_Ptr<core_OBJECT>& theSmartRef)
  {
    return core_Ptr<SharedType>( theSmartRef.IsNull() ? nullptr : dynamic_cast<SharedType*>( theSmartRef.Access() ) );
  }

  //! Casts the passed pointer to the necessary type.
  //! \param theSmartRef [in] pointer to cast.
  //! \return casted pointer.
  template <class OtherType>
  static core_Ptr<SharedType> DownCast(const core_Ptr<OtherType>& theSmartRef)
  {
    return core_Ptr<SharedType>( theSmartRef.IsNull() ? nullptr : dynamic_cast<SharedType*>( theSmartRef.Access() ) );
  }

private:

  //! Increments reference counter in the handled object.
  void beginScope()
  {
    if ( m_pRef )
      m_pRef->IncRef();
  }

  //! Decrements reference counter in the handled object.
  //! If counter's value goes to 0, the object is deleted.
  void endScope()
  {
    if ( m_pRef )
    {
      m_pRef->DecRef();
      if ( !m_pRef->NbRefs() )
      {
        delete m_pRef;
        m_pRef = nullptr;
      }
    }
  }

private:

  SharedType* m_pRef; //!< Actual reference to the wrapped object.

};

//-----------------------------------------------------------------------------
// Handy shortcuts
//-----------------------------------------------------------------------------

//! Shortcut for Mobius shared pointer.
#define t_ptr core_Ptr

};

#endif
