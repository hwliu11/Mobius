//-----------------------------------------------------------------------------
// Created on: 18 September 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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

#ifndef poly_Handles_HeaderFile
#define poly_Handles_HeaderFile

// Poly includes
#include <mobius/poly.h>

#define Mobius_InvalidHandleIndex -1

namespace mobius {

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_POLY
//!
//! Base class for all handles used to address mesh elements. This class is
//! intended to substitute raw indices which are less descriptive and more
//! error-prone to operate with.
class poly_BaseHandle
{
public:

  //! Ctor accepting the optional index.
  //! \param[in] _idx 0-based index to set for the handle. Use "-1" to
  //!                 invalidate the handle.
  explicit poly_BaseHandle(const int _idx = Mobius_InvalidHandleIndex) : iIdx(_idx) {}

public:

  //! \return underlying index of this handle.
  int GetIdx() const { return iIdx; }

  //! The handle is valid iff the index is not negative.
  bool IsValid() const { return iIdx >= 0; }

  //! Invalidates this handle.
  void Invalidate() { iIdx = Mobius_InvalidHandleIndex; }

  //! Compares this handle with the passed one.
  //! \param[in] _rhs another handle to compare this handle with.
  //! \return true in case of equality, false -- otherwise.
  bool operator==(const poly_BaseHandle& _rhs) const
  {
    return this->iIdx == _rhs.iIdx;
  }

  //! Checks for inequality.
  //! \param[in] _rhs another handle to compare this handle with.
  //! \return true in case of inequality, false -- otherwise.
  bool operator!=(const poly_BaseHandle& _rhs) const
  {
    return this->iIdx != _rhs.iIdx;
  }

  //! Checks if this handle is less than the passed one.
  //! \param[in] _rhs another handle to compare this handle with.
  //! \return true if this handle is less than the passed one, false -- otherwise.
  bool operator<(const poly_BaseHandle& _rhs) const
  {
    return this->iIdx < _rhs.iIdx;
  }

protected:

  void increment() { ++iIdx; } //!< Increments the underlying index.
  void decrement() { --iIdx; } //!< Decrements the underlying index.

  //! Increments the underlying index by the passed value.
  //! \param[in] amount increment value to use.
  void increment(const int amount) { iIdx += amount; }

  //! Decrements the underlying index by the passed value.
  //! \param[in] amount decrement value to use.
  void decrement(const int amount) { iIdx -= amount; }

public:

  int iIdx; //!< 0-based index of the handle.

};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_POLY
//!
//! Handle for vertex.
struct poly_VertexHandle : public poly_BaseHandle
{
  //! Ctor accepting the optional index.
  //! \param[in] _idx 0-based index to set for the handle. Use "-1" to
  //!                 invalidate the handle.
  explicit poly_VertexHandle(const int _idx = Mobius_InvalidHandleIndex) : poly_BaseHandle(_idx) {}
};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_POLY
//!
//! Handle for edge.
struct poly_EdgeHandle : public poly_BaseHandle
{
  //! Ctor accepting the optional index.
  //! \param[in] _idx 0-based index to set for the handle. Use "-1" to
  //!                 invalidate the handle.
  explicit poly_EdgeHandle(const int _idx = Mobius_InvalidHandleIndex) : poly_BaseHandle(_idx) {}

  //! Ctor accepting the size_t index.
  //! \param[in] _idx 0-based index to set for the handle.
  explicit poly_EdgeHandle(const size_t _idx) : poly_BaseHandle( int(_idx) ) {}
};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_POLY
//!
//! Handle for triangle.
struct poly_TriangleHandle : public poly_BaseHandle
{
  //! Ctor accepting the optional index.
  //! \param[in] _idx 0-based index to set for the handle. Use "-1" to
  //!                 invalidate the handle.
  explicit poly_TriangleHandle(const int _idx = Mobius_InvalidHandleIndex) : poly_BaseHandle(_idx) {}
};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_POLY
//!
//! Handle for quad.
struct poly_QuadHandle : public poly_BaseHandle
{
  //! Ctor accepting the optional index.
  //! \param[in] _idx 0-based index to set for the handle. Use "-1" to
  //!                 invalidate the handle.
  explicit poly_QuadHandle(const int _idx = Mobius_InvalidHandleIndex) : poly_BaseHandle(_idx) {}
};

}

//-----------------------------------------------------------------------------
// Hashers
//-----------------------------------------------------------------------------

// Std includes.
#include <functional>

namespace std {

//! Hasher for base handle.
template <>
struct hash<mobius::poly_BaseHandle>
{
  typedef mobius::poly_BaseHandle argument_type;
  typedef std::size_t             result_type;

  //! Returns index of a handle for hashing.
  //! \param[in] h handle.
  //! \return index.
  std::size_t operator()(const mobius::poly_BaseHandle& h) const
  {
    return h.GetIdx();
  }
};

//! Hasher for vertex handle.
template <>
struct hash<mobius::poly_VertexHandle>
{
  typedef mobius::poly_VertexHandle argument_type;
  typedef std::size_t               result_type;

  //! Returns index of a handle for hashing.
  //! \param[in] h handle.
  //! \return index.
  std::size_t operator()(const mobius::poly_VertexHandle& h) const
  {
    return h.GetIdx();
  }
};

//! Hasher for edge handle.
template <>
struct hash<mobius::poly_EdgeHandle>
{
  typedef mobius::poly_EdgeHandle argument_type;
  typedef std::size_t             result_type;

  //! Returns index of a handle for hashing.
  //! \param[in] h handle.
  //! \return index.
  std::size_t operator()(const mobius::poly_EdgeHandle& h) const
  {
    return h.GetIdx();
  }
};

//! Hasher for triangle handle.
template <>
struct hash<mobius::poly_TriangleHandle>
{
  typedef mobius::poly_TriangleHandle argument_type;
  typedef std::size_t                 result_type;

  //! Returns index of a handle for hashing.
  //! \param[in] h handle.
  //! \return index.
  std::size_t operator()(const mobius::poly_TriangleHandle& h) const
  {
    return h.GetIdx();
  }
};

//! Hasher for quad handle.
template <>
struct hash<mobius::poly_QuadHandle>
{
  typedef mobius::poly_QuadHandle argument_type;
  typedef std::size_t             result_type;

  //! Returns index of a handle for hashing.
  //! \param[in] h handle.
  //! \return index.
  std::size_t operator()(const mobius::poly_QuadHandle& h) const
  {
    return h.GetIdx();
  }
};

}

#endif
