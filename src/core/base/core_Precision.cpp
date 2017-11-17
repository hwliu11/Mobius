//-----------------------------------------------------------------------------
// Created on: 17 January 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#include <mobius/core_Precision.h>

//! Precision in 3D space to cover rounding errors.
//! \return geometric precision to use in 3D.
double mobius::core_Precision::Resolution3D()
{
  return 1.0e-6;
}
