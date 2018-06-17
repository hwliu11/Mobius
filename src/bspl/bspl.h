//-----------------------------------------------------------------------------
// Created on: 19 July 2017
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

#ifndef bspl_HeaderFile
#define bspl_HeaderFile

// core includes
#include <mobius/core.h>

// Common bspl includes
#include <mobius/bspl_excMaxDegreeViolation.h>

#define mobiusBSpl_NotUsed(x) x

#if defined _WIN32
  #if defined mobiusBSpl_EXPORTS
    #define mobiusBSpl_EXPORT __declspec(dllexport)
  #else
    #define mobiusBSpl_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusBSpl_EXPORT
#endif

//-----------------------------------------------------------------------------
// DOXY group definition
//-----------------------------------------------------------------------------
//! \defgroup MOBIUS_BSPL B-spline library
//!
//! Data structures and algorithms for operating with B-spline functions.
//-----------------------------------------------------------------------------

#define mobiusBSpl_MaxDegree 25

//-----------------------------------------------------------------------------

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Generic tools.
namespace bspl
{
  //! Returns last knot index (0-based). That's a handy shortcut for
  //! evaluating "m = n + p + 1".
  //!
  //! \param[in] n index of the last pole (0-based).
  //! \param[in] p degree of the B-spline basis function to use.
  //! \return last knot index.
  mobiusBSpl_EXPORT int
    M(const int n, const int p);

  //! Returns last pole index (0-based). That's a handy shortcut for
  //! evaluating "n = m - p - 1".
  //!
  //! \param[in] m index of the last knot (0-based).
  //! \param[in] p degree of the B-spline basis function to use.
  //! \return last pole index.
  mobiusBSpl_EXPORT int
    N(const int m, const int p);

  //! Returns number of knots for the given indices related to the
  //! B-spline degree and number of control points to be used for
  //! a derived B-object. This is nothing more than a convenience method which
  //! you could prefer to use! instead of explicit well-known "m = n + p + 1"
  //! formulation.
  //!
  //! \param[in] n index of the last pole (0-based).
  //! \param[in] p degree of the B-spline basis function to use.
  //! \return number of knots.
  mobiusBSpl_EXPORT int
    NumberOfKnots(const int n, const int p);

  //! Checks whether the given number of poles in enough for a B-object
  //! with the passed degree.
  //!
  //! \param[in] n index of the last pole (0-based).
  //! \param[in] p degree of the B-spline basis function to use.
  //! \return true/false.
  mobiusBSpl_EXPORT bool
    Check(const int n, const int p);

  //! Checks equation "m = n + p + 1".
  //!
  //! \param[in] n index of the last pole (0-based).
  //! \param[in] m index of the last knot (0-based).
  //! \param[in] p degree of the B-spline basis function to use.
  //! \return true/false.
  mobiusBSpl_EXPORT bool
    Check(const int n, const int m, const int p);

};

};

#endif
