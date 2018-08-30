//-----------------------------------------------------------------------------
// Created on: 06 March 2015
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
#include <mobius/bspl_UnifyKnots.h>

// bspl includes
#include <mobius/bspl_KnotMultiset.h>

//! Unifies the passed knot vectors.
//! \param knot_vectors [in] knot vectors to unify.
//! \return addendums to unification.
std::vector< std::vector<adouble> >
  mobius::bspl_UnifyKnots::operator()(std::vector< std::vector<adouble> >& knot_vectors) const
{
  std::vector< std::vector<adouble> > X;

  for ( size_t i = 0; i < knot_vectors.size(); ++i )
  {
    bspl_KnotMultiset Ui = knot_vectors[i];
    bspl_KnotMultiset U;
    //
    for ( size_t j = 0; j < knot_vectors.size(); ++j )
    {
      if ( j == i )
        continue;

      bspl_KnotMultiset Uj = knot_vectors[j];
      U = U + Uj;
    }

    bspl_KnotMultiset Xi = U - Ui;
    X.push_back( Xi.Convert() );
  }

  return X; // All have to be the same after unification
}
