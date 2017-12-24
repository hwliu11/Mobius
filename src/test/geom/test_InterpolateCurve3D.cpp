//-----------------------------------------------------------------------------
// Created on: 04 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/test_InterpolateCurve3D.h>

// geom includes
#include <mobius/geom_InterpolateCurve.h>

//! Test scenario 001.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_InterpolateCurve3D::test1(const int test_NotUsed(funcID))
{
  /* ~~~~~~~~~~~~~~~~~~~~~~
   *  Prepare input points
   * ~~~~~~~~~~~~~~~~~~~~~~ */

  xyz Q[5] = { xyz( 0.0,  0.0, 0.0),
               xyz( 3.0,  4.0, 0.0),
               xyz(-1.0,  4.0, 0.0),
               xyz(-4.0,  0.0, 0.0),
               xyz(-4.0, -3.0, 0.0) };

  std::vector<xyz> Q_vec;
  for ( int k = 0; k < sizeof(Q)/sizeof(xyz); ++k )
    Q_vec.push_back(Q[k]);

  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   *  Run interpolation algorithm
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

  // Construct interpolation tool
  geom_InterpolateCurve Interp(Q_vec, 3, ParamsSelection_ChordLength, KnotsSelection_Average);

  // Run interpolation
  Interp.Perform();

  // Verify
  if ( Interp.ErrorCode() != geom_InterpolateCurve::ErrCode_NoError )
    return false;

  return true;
}
