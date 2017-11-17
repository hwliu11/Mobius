//-----------------------------------------------------------------------------
// Created on: 02 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef core_SolveLinearSystem_HeaderFile
#define core_SolveLinearSystem_HeaderFile

// core includes
#include <mobius/core.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Solver for linear system of equations.
class core_SolveLinearSystem
{
// Construction & destruction:
public:

  mobiusCore_EXPORT core_SolveLinearSystem();
  mobiusCore_EXPORT virtual ~core_SolveLinearSystem();

public:

  mobiusCore_EXPORT void
    operator()(const double* A,
               const double* b,
               double*       x,
               const int     dim);

};

};

#endif
