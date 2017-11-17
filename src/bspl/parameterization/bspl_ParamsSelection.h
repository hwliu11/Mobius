//-----------------------------------------------------------------------------
// Created on: 26 October 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_ParamsSelection_HeaderFile
#define bspl_ParamsSelection_HeaderFile

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Defines a strategy for selection of interpolant parameters in curve/surface
//! reconstruction context.
enum bspl_ParamsSelection
{
  ParamsSelection_Undefined = 0, //!< Undefined.
  ParamsSelection_Uniform,       //!< Uniform distribution of parameters.
  ParamsSelection_ChordLength,   //!< Parameters are chosen respectively to chord length.
  ParamsSelection_Centripetal,   //!< Centripetal model described by Lee.
  ParamsSelection_Exponent       //!< Generalization of centripetal model.
};

};

#endif
