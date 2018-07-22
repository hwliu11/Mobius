//-----------------------------------------------------------------------------
// Created on: 11 June 2013
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

#ifndef bspl_FindSpan_HeaderFile
#define bspl_FindSpan_HeaderFile

// bspl includes
#include <mobius/bspl.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! \brief A2.1 from "The NURBS Book".
//!
//! Algorithm finding span index using binary search. This function works
//! properly for clamped knot vectors only. If the passed value falls outside
//! the min and max knot values, this function returns index of the closest
//! non-degenerated span.
//! 
//!
//! Inputs:
//!   u -- target knot;
//!   U -- reference to the knot vector;
//!   p -- degree.
//!
//! Outputs:
//!   index of the knot span.
//!
//! Notice that this tool accepts a reference to the knot vector in order to
//! avoid copying the knots.
class bspl_FindSpan
{
public:

  mobiusBSpl_EXPORT
    bspl_FindSpan(const std::vector<double>& U,
                  const int                  p);

public:

  mobiusBSpl_EXPORT int
    operator()(const double u) const;

  mobiusBSpl_EXPORT int
    operator()(const double u,
               int&         firstNonZeroIndex) const;

private:

  void operator=(const bspl_FindSpan&) = delete;

protected:

  const std::vector<double>& m_U;    //!< Reference to knot vector.
  int                        m_iDeg; //!< Degree.

};

};

#endif
