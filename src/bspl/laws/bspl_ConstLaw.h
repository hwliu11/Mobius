//-----------------------------------------------------------------------------
// Created on: 20 June 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_ConstLaw_HeaderFile
#define bspl_ConstLaw_HeaderFile

// bspl includes
#include <mobius/bspl_ScalarLaw.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Constant law.
class bspl_ConstLaw : public bspl_ScalarLaw
{
public:

  mobiusBSpl_EXPORT
    bspl_ConstLaw(const double level);

public:

  mobiusBSpl_EXPORT virtual double
    Eval(const double u) const;

protected:

  double m_fLevel; //!< Function value.

};

};

#endif
