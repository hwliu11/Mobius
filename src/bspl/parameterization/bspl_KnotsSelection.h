//-----------------------------------------------------------------------------
// Created on: 26 October 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_KnotsSelection_HeaderFile
#define bspl_KnotsSelection_HeaderFile

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Defines a strategy for knot selection in curve/surface reconstruction
//! context.
enum bspl_KnotsSelection
{
  KnotsSelection_Undefined = 0, //!< Undefined.
  KnotsSelection_Uniform,       //!< Uniform distribution of knots on [0;1] interval.
  KnotsSelection_Average        //!< Averaging technique using p (degree) interpolant parameters.
};

};

#endif
