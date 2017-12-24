//-----------------------------------------------------------------------------
// Created on: 05 September 2014
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
#include <mobius/core_IsoTransformChain.h>

//! Constructs empty chain of isometric transformations.
mobius::core_IsoTransformChain::core_IsoTransformChain()
{}

//! Constructs a chain comprised of only one transformation.
//! \param transform [in] transformation to populate the chain with.
mobius::core_IsoTransformChain::core_IsoTransformChain(const core_IsoTransform& transform)
{
  this->operator<<(transform);
}

//! Enqueues the passed transformation to chain. The order is FILO, so
//! the very first transformation will be applied last.
//! \param transform [in] transformation to put into chain.
//! \return this chain for convenient streaming.
mobius::core_IsoTransformChain&
  mobius::core_IsoTransformChain::operator<<(const core_IsoTransform& transform)
{
  m_tr_list.push_back(transform);
  return *this;
}

//! Enqueues the passed transformation chain to this chain.
//! \param transformChain [in] transformation chain to put into this chain.
//! \return this chain for convenient streaming.
mobius::core_IsoTransformChain&
  mobius::core_IsoTransformChain::operator<<(const core_IsoTransformChain& transformChain)
{
  for ( size_t t = 0; t < transformChain.m_tr_list.size(); ++t )
    this->operator<<( transformChain.m_tr_list[t] );

  return *this;
}

//! Applies transformations to the given point one by one.
//! \param coord [in] point to transform.
//! \return transformed coordinates.
mobius::core_XYZ mobius::core_IsoTransformChain::Apply(const core_XYZ& coord) const
{
  core_XYZ result = coord;

  if ( m_tr_list.size() )
    for ( int t = (int) (m_tr_list.size() - 1); t >= 0; --t )
      result = m_tr_list[t].Apply(result);

  // Return result
  return result;
}
