//-----------------------------------------------------------------------------
// Created on: 05 September 2014
// Created by: Sergey SLYADNEV
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
