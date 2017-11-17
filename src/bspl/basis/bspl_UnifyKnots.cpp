//-----------------------------------------------------------------------------
// Created on: 06 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/bspl_UnifyKnots.h>

// bspl includes
#include <mobius/bspl_KnotMultiset.h>

//! Unifies the passed knot vectors.
//! \param knot_vectors [in] knot vectors to unify.
//! \return addendums to unification.
std::vector< std::vector<double> >
  mobius::bspl_UnifyKnots::operator()(std::vector< std::vector<double> >& knot_vectors) const
{
  std::vector< std::vector<double> > X;

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
