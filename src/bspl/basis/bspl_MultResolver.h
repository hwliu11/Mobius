//-----------------------------------------------------------------------------
// Created on: 13 December 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

#ifndef bspl_MultResolver_HeaderFile
#define bspl_MultResolver_HeaderFile

// BSpl includes
#include <mobius/bspl_KnotMultiset.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Utility to count multiplicity of knots.
class bspl_MultResolver
{
// Members:
public:

  std::vector<bspl_KnotMultiset::elem> Knots; //!< Knots being processed.

public:

  //! Default ctor.
  mobiusBSpl_EXPORT
    bspl_MultResolver();

  //! Ctor accepting the knot vector to resolve. This ctor automatically
  //! calls Resolve() method.
  //! \param[in] U knot vector to resolve.
  mobiusBSpl_EXPORT
    bspl_MultResolver(const std::vector<double>& U);

public:

  //! Resolves multiplicities for the passed parameter.
  //! \param[in] u knot value to resolve.
  mobiusBSpl_EXPORT void
    Resolve(const double u);

  //! Resolves multiplicities for the passed knot vector.
  //! \param[in] U knot vector to resolve.
  mobiusBSpl_EXPORT void
    Resolve(const std::vector<double>& U);

public:

  //! \return first knot with its multiplicity.
  const bspl_KnotMultiset::elem& GetFirstKnot() const { return Knots[0]; }

  //! \return last knot with its multiplicity.
  const bspl_KnotMultiset::elem& GetLastKnot() const { return Knots[Knots.size() - 1]; }

};

}

#endif
