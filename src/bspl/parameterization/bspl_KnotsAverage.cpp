//-----------------------------------------------------------------------------
// Created on: 02 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/bspl_KnotsAverage.h>

//! Recognizes the averaging context.
//! \param D1_start [in] indicates whether D1 derivative constraint is requested
//!                      at the starting point of the curve being reconstructed.
//! \param D1_end   [in] indicates whether D1 derivative constraint is requested
//!                      at the trailing point of the curve being reconstructed.
//! \param D2_start [in] indicates whether D2 derivative constraint is requested
//!                      at the starting point of the curve being reconstructed.
//! \param D2_end   [in] indicates whether D2 derivative constraint is requested
//!                      at the trailing point of the curve being reconstructed.
//! \return averaging context.
mobius::bspl_KnotsAverage::Case
  mobius::bspl_KnotsAverage::Recognize(const bool D1_start,
                                       const bool D1_end,
                                       const bool D2_start,
                                       const bool D2_end)
{
  // TODO: currently averaging technique can process only a limited
  //       number of cases. It is not an easy question how to pre-select
  //       knots when many constraints are specified. Selection of knots
  //       has a great impact on the resulting interpolant's geometry, so
  //       one cannot simply choose "just any" knot vector and expect good
  //       geometry at the end

  if ( !D1_start && !D1_end && !D2_start && !D2_end )
    return Case_PointsOnly;

  if ( D1_start && D1_end && !D2_start && !D2_end )
    return Case_Points_D1;

  if ( D1_start && D1_end && D2_start && D2_end )
    return Case_Points_D1_D2;

  return Case_Unhandled;
}
