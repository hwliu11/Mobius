//-----------------------------------------------------------------------------
// Created on: 05 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_InsKnot_HeaderFile
#define bspl_InsKnot_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Knot insertion implementation according to A5.1 from "The NURBS Book".
class bspl_InsKnot
{
public:

  mobiusBSpl_EXPORT bool
    operator()(const int                    np,
               const int                    p,
               const std::vector<double>&   UP,
               const std::vector<core_XYZ>& Pw,
               const double                 u,
               const int                    k,
               const int                    s,
               const int                    r,
               int&                         nq,
               std::vector<double>&         UQ,
               std::vector<core_XYZ>&       Qw) const;

};

};

#endif
