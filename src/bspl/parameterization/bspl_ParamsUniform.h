//-----------------------------------------------------------------------------
// Created on: 16 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_ParamsUniform_HeaderFile
#define bspl_ParamsUniform_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Chooses target curve's parameters according to the simplest uniform
//! distribution rule:
//!
//! t_0 = 0;
//! ...
//! t_k = k/n;
//! ...
//! t_n = 1;
class bspl_ParamsUniform
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_CannotProceedWithSolePoint,
  };

public:

  //! Calculates parameter values by uniform distribution rule.
  //! \param n [in]  index of the last parameter in 0-based collection.
  //! \param t [out] calculated parameter values.
  //! \return error code.
  static ErrCode Calculate(const int n,
                           double*   t)
  {
    if ( n < 2 )
      return ErrCode_CannotProceedWithSolePoint; // Cannot proceed with a sole point

    // First parameter
    t[0] = 0.0;

    // Calculate next parameter
    for ( int k = 1; k < n; ++k )
    {
      t[k] = (double) k / n;
    }

    // Last parameter
    t[n] = 1.0;

    // Success
    return ErrCode_NoError;
  }

private:

  bspl_ParamsUniform() {}
  bspl_ParamsUniform(const bspl_ParamsUniform&) {}

};

};

#endif
