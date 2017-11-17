//-----------------------------------------------------------------------------
// Created on: 17 August 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_ScalarLaw_HeaderFile
#define bspl_ScalarLaw_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_OBJECT.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! This class represents abstract 1-dimensional scalar law.
class bspl_ScalarLaw : public OBJECT
{
public:

  //! This function is intended to query the scalar law function with the
  //! given parameter.
  //! \param u [in] parameter value to query the scalar law function.
  //! \return resulting value.
  virtual double
    Eval(const double u) const = 0;

};

};

#endif
