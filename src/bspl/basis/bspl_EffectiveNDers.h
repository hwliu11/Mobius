//-----------------------------------------------------------------------------
// Created on: 10 February 2014
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

#ifndef bspl_EffectiveNDers_HeaderFile
#define bspl_EffectiveNDers_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! \brief A2.3 from "The NURBS Book".
//!
//! Algorithm calculating those B-spline basis functions and their derivatives
//! which are non-vanishing in the given span. The target span is referenced by
//! its zero-based index. In order to use this functionality properly,
//! one should locate the target span by BasisFindSpan() and then ask this
//! routine to evaluate the basis functions (and derivatives) for the passed
//! parameter u.
//!
//! This algorithm is principally an extension of BasisEffectiveN(). Actually,
//! it does very similar job, however, it also allows calculation of (k)-th
//! order derivatives. The derivatives are calculated according to the
//! non-recurrent formula:
//!
//! <pre>
//! N^(k)_i,p = p! / (p - k)! Sum_{j=0,k} ( a_k,j N_{i+j},{p-k} )
//! </pre>
//!
//! This algorithm is actually the implementation of A2.3 from
//! "The NURBS Book".
class bspl_EffectiveNDers
{
public:

  bspl_EffectiveNDers(ptr<alloc2d> alloc    = NULL,
                      const int    memBlock = -1)
  {
    m_pAlloc    = alloc;
    m_iMemBlock = memBlock;
  }

public:

  mobiusBSpl_EXPORT void
    operator()(const adouble               u,
               const std::vector<adouble>& U,
               const int                  p,
               const int                  i,
               const int                  order,
               adouble**                   ders) const;

protected:

  ptr<alloc2d> m_pAlloc;
  int          m_iMemBlock;

};

};

#endif
