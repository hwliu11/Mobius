//-----------------------------------------------------------------------------
// Created on: 20 November 2013
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

#ifndef core_HeapAlloc_HeaderFile
#define core_HeapAlloc_HeaderFile

// core includes
#include <mobius/core.h>

// STD includes
#include <vector>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

namespace mobius {

//-----------------------------------------------------------------------------
// Allocator for 1-dimensional arrays
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Simple convenience class for allocation/deallocation of heap arrays. This
//! allocator stores raw pointers to the acquired memory in its internal
//! collection. When allocator is destructed, all acquired memory is released.
template <typename ElemType>
class core_HeapAlloc
{
public:

  //! Raw pointer with size.
  struct THeapPtr
  {
    THeapPtr() : Ptr(NULL), Num(0) {} //!< Default constructor.

    ElemType* Ptr; //!< Pointer to the allocated memory.
    size_t    Num; //!< Number of elements.
  };

public:

  //! Template class allocating memory for array of the given type having
  //! the passed number of elements.
  //! \param numElems  [in] number of elements.
  //! \param doNullify [in] indicates whether to nullify the allocated
  //!                       memory or not.
  //! \return pointer to the allocated memory.
  ElemType* Allocate(const size_t numElems,
                     const bool   doNullify = false)
  {
    // Allocate
    ElemType* ptr = new ElemType[numElems];
    THeapPtr HeapPtr;
    HeapPtr.Ptr = ptr;
    HeapPtr.Num = numElems;

    // Nullify
    if ( doNullify )
    {
#if defined COUT_DEBUG
      std::cout << "*** core_HeapAlloc::Allocate> sizeof(ElemType) = " << numElems*sizeof(ElemType) << std::endl;
#endif
      memset( ptr, 0, numElems*sizeof(ElemType) );
    }

    // Store pointer in the internal collection for deallocation
    m_ptrVector.push_back(HeapPtr);

    // Return
    return ptr;
  }

  //! Accessor for the raw array by its internal index in the allocator.
  //! \param arr_idx [in] 0-based index of array in the allocator.
  //! \return pointer to the target array.
  const THeapPtr& Access(const size_t arr_idx) const
  {
    return m_ptrVector.at(arr_idx);
  }

public:

  //! Default constructor.
  core_HeapAlloc()
  {}

  //! Deallocates all consumed memory.
  ~core_HeapAlloc()
  {
    for ( int i = 0; i < (int) m_ptrVector.size(); ++i )
    {
      THeapPtr& HeapPtr = m_ptrVector.at(i);
      delete[] HeapPtr.Ptr;
      HeapPtr.Ptr = NULL;
      HeapPtr.Num = 0;
    }
  }

private:

  //! Pointers to allocated memory in order of allocation.
  std::vector<THeapPtr> m_ptrVector;

};

//-----------------------------------------------------------------------------
// Allocator for 2-dimensional arrays
//-----------------------------------------------------------------------------

//! Simple convenience class for allocation/deallocation of heap matrices. This
//! allocator stores raw pointers to the acquired memory in its internal
//! collection. When allocator is destructed, all acquired memory is released.
template <typename ElemType>
class core_HeapAlloc2D
{
public:

  //! Raw pointer with size.
  struct THeapPtr
  {
    THeapPtr() : Ptr(NULL), NumRows(0), NumCols(0) {} //!< Default constructor.

    ElemType** Ptr;     //!< Pointer to the allocated memory.
    int        NumRows; //!< Number of rows.
    int        NumCols; //!< Number of columns.
  };

public:

  //! Template class allocating memory for matrix of the given type having
  //! the passed number of elements.
  //! \param numRows   [in] number of rows.
  //! \param numCols   [in] number of columns.
  //! \param doNullify [in] indicates whether to nullify the allocated
  //!                       memory or not.
  //! \return pointer to the allocated memory.
  ElemType** Allocate(const int  numRows,
                      const int  numCols,
                      const bool doNullify = false)
  {
    // Allocate
    ElemType** ptr = new ElemType*[numRows];
    for ( int i = 0; i < numRows; ++i )
    {
      ptr[i] = new ElemType[numCols];

      // Nullify
      if ( doNullify )
        memset(ptr[i], 0, numCols*sizeof(ElemType));
    }

    THeapPtr HeapPtr;
    HeapPtr.Ptr     = reinterpret_cast<ElemType**>(ptr);
    HeapPtr.NumRows = numRows;
    HeapPtr.NumCols = numCols;

    // Store pointer in the internal collection for deallocation
    m_ptrVector.push_back(HeapPtr);

    // Return
    return ptr;
  }

public:

  //! Default constructor.
  core_HeapAlloc2D()
  {}

  //! Deallocates all consumed memory.
  ~core_HeapAlloc2D()
  {
    for ( int i = 0; i < (int) m_ptrVector.size(); ++i )
    {
      THeapPtr& HeapPtr = m_ptrVector.at(i);
      for ( int i = 0; i < HeapPtr.NumRows; ++i )
      {
        delete[] HeapPtr.Ptr[i];
        HeapPtr.Ptr[i] = NULL;
      }
      HeapPtr.NumRows = 0;
      HeapPtr.NumCols = 0;
    }
  }

private:

  //! Pointers to allocated memory in order of allocation.
  std::vector<THeapPtr> m_ptrVector;

};

};

#endif
