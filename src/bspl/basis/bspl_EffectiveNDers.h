//-----------------------------------------------------------------------------
// Created on: 10 February 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_EffectiveNDers_HeaderFile
#define bspl_EffectiveNDers_HeaderFile

// bspl includes
#include <mobius/bspl.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
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

  mobiusBSpl_EXPORT void
    operator()(const double               u,
               const std::vector<double>& U,
               const int                  p,
               const int                  i,
               const int                  n,
               double**                   ders) const;

};

};

#endif
