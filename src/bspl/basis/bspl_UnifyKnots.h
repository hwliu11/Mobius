//-----------------------------------------------------------------------------
// Created on: 06 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_UnifyKnots_HeaderFile
#define bspl_UnifyKnots_HeaderFile

// bspl includes
#include <mobius/bspl.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Passing a collection of knots calculates the addition knot vector required
//! for unification. Applying this addition you obtain compatible knot
//! vectors.
class bspl_UnifyKnots
{
public:

  mobiusBSpl_EXPORT std::vector< std::vector<double> >
    operator()(std::vector< std::vector<double> >& knot_vectors) const;

};

};

#endif
