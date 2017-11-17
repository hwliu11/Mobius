//-----------------------------------------------------------------------------
// Created on: 20 June 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/bspl_ConstLaw.h>

//! Creates law for the given constant value.
//! \param level [in] value to use.
mobius::bspl_ConstLaw::bspl_ConstLaw(const double level)
: bspl_ScalarLaw(),
  m_fLevel(level)
{}

//! Evaluates law.
//! \param u [in] parameter to evaluate law for.
//! \return evaluated function.
double mobius::bspl_ConstLaw::Eval(const double u) const
{
  mobiusBSpl_NotUsed(u);

  return m_fLevel;
}
