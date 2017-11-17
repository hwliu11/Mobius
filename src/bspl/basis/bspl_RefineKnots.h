//-----------------------------------------------------------------------------
// Created on: 06 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_RefineKnots_HeaderFile
#define bspl_RefineKnots_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Knot refinement implementation according to A5.4 from "The NURBS Book".
class bspl_RefineKnots
{
public:

  mobiusBSpl_EXPORT bool
    operator()(const int                  n,
               const int                  p,
               const std::vector<double>& U,
               const std::vector<xyz>&    Pw,
               const double*              X,
               const int                  r,
               std::vector<double>&       Ubar,
               std::vector<xyz>&          Qw) const;

};

};

#endif
