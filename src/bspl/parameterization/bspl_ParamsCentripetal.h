//-----------------------------------------------------------------------------
// Created on: 16 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_ParamsCentripetal_HeaderFile
#define bspl_ParamsCentripetal_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_HeapAlloc.h>
#include <mobius/core_XYZ.h>

// STL includes
#include <vector>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Chooses target curve's parameters according to centripetal rule:
//!
//! t_0 = 0;
//! ...
//! t_k = t_k-1 + Sqrt|Q_k - Q_k-1|/d;
//! ...
//! t_n = 1;
//!
//! Here {Q_k : k=0,n} are the data points; {t_k} are resulting
//! parameters; d = Sum(Sqrt|Q_k - Q_k-1|).
class bspl_ParamsCentripetal
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_CannotProceedWithSolePoint,
    ErrCode_InvalidGridDimensions
  };

public:

//---------------------------------------------------------------------------//

  //! Calculates parameter values by centripetal rule.
  //! \param Q [in]  input points.
  //! \param t [out] calculated parameter values.
  //! \return error code.
  static ErrCode Calculate(const std::vector<xyz>& Q,
                           double*                 t)
  {
    const int len = (int) Q.size();
    if ( len == 1 )
      return ErrCode_CannotProceedWithSolePoint; // Cannot proceed with a sole point

    // First parameter
    t[0] = 0.0;

    // Calculate d (by Lee's formula)
    double d = 0.0;
    for ( int idx = 1; idx < len; ++idx )
    {
      xyz QQ =  Q.at(idx) - Q.at(idx-1);
      d      += sqrt( QQ.Modulus() );
    }

    // Calculate next parameter
    for ( int idx = 1; idx < len-1; ++idx )
    {
      xyz QQ = Q.at(idx) - Q.at(idx-1);
      t[idx] = t[idx-1] + sqrt( QQ.Modulus() ) / d;
    }

    // Last parameter
    t[len-1] = 1.0;

    // Success
    return ErrCode_NoError;
  }

//---------------------------------------------------------------------------//

  //! Calculates parameter values by centripetal rule in 2-dimensional case.
  //! \param Q [in]  input points.
  //! \param u [out] calculated parameter values in U direction.
  //! \param v [out] calculated parameter values in V direction.
  //! \return error code.
  static ErrCode Calculate(const std::vector< std::vector<xyz> >& Q,
                           double*                                u,
                           double*                                v)
  {
    const int n = (int) Q.size() - 1;
    if ( n <= 0 )
      return ErrCode_InvalidGridDimensions;

    const int m = (int) Q.at(0).size() - 1;
    if ( m <= 0 )
      return ErrCode_InvalidGridDimensions;

    core_HeapAlloc<double> alloc;
    core_HeapAlloc2D<double> alloc2d;

    //-----------------------------------
    // Calculate lengths of each isoline
    //-----------------------------------

    // Arrays for polyline lengths
    double* d_isoU = alloc.Allocate(n + 1, false);
    double* d_isoV = alloc.Allocate(m + 1, false);

    // Loop over the V direction to calculate the sum of square roots
    // for each V-isoline
    for ( int l = 0; l <= m; ++l )
    {
      // Loop over the U direction
      for ( int k = 1; k <= n; ++k )
      {
        xyz    QQ =  Q[k][l] - Q[k-1][l];
        d_isoV[l] += sqrt( QQ.Modulus() );
      }
    }

    // Loop over the U direction to calculate the sum of square roots
    // for each U-isoline
    for ( int k = 0; k <= n; ++k )
    {
      // Loop over the V direction
      for ( int l = 1; l <= m; ++l )
      {
        xyz    QQ =  Q[k][l] - Q[k][l-1];
        d_isoU[k] += sqrt( QQ.Modulus() );
      }
    }

    //----------------------------------------------------
    // Calculate non-averaged parameters for each isoline
    //----------------------------------------------------

    // Parameters per single isolines
    double** param_isoU = alloc2d.Allocate(n + 1, m + 1, false);
    double** param_isoV = alloc2d.Allocate(n + 1, m + 1, false);

    // Calculate reper V parameters. Now loop over the V direction (columns)
    for ( int j = 0; j <= m; ++j )
    {
      // Bind first parameters to 0
      param_isoV[0][j] = 0.0;

      // Loop over the U direction calculating parameters
      for ( int i = 1; i < n; ++i )
      {
        xyz           QQ = Q[i][j] - Q[i-1][j];
        param_isoV[i][j] = param_isoV[i-1][j] + sqrt( QQ.Modulus() )/d_isoV[j];
      }

      // Bind last parameters to 1
      param_isoV[n][j] = 1.0;
    }

    // Calculate reper V parameters. Now loop over the U direction (rows)
    for ( int i = 0; i <= n; ++i )
    {
      // Bind first parameters to 0
      param_isoU[i][0] = 0.0;

      // Loop over the V direction calculating parameters
      for ( int j = 1; j < m; ++j )
      {
        xyz           QQ = Q[i][j] - Q[i][j-1];
        param_isoU[i][j] = param_isoU[i][j-1] + sqrt( QQ.Modulus() )/d_isoU[i];
      }

      // Bind last parameters to 1
      param_isoU[i][m] = 1.0;
    }

    //----------------------------------
    // Apply averaging for U parameters
    //----------------------------------

    // First parameter
    u[0] = 0.0;

    // Intermediate parameters
    for ( int i = 1; i < n; ++i )
    {
      double sum_overV = 0.0;
      for ( int j = 0; j <= m; ++j )
        sum_overV += param_isoV[i][j];

      u[i] = sum_overV / (m + 1);
    }

    // Last parameter
    u[n] = 1.0;

    //----------------------------------
    // Apply averaging for V parameters
    //----------------------------------

    // First parameter
    v[0] = 0.0;

    // Intermediate parameters
    for ( int j = 1; j < m; ++j )
    {
      double sum_overU = 0.0;
      for ( int i = 0; i <= n; ++i )
        sum_overU += param_isoU[i][j];

      v[j] = sum_overU / (n + 1);
    }

    // Last parameter
    v[m] = 1.0;

    // Success
    return ErrCode_NoError;
  }

//---------------------------------------------------------------------------//

  //! Calculates iso-U parameter values by chord length rule in 2-dimensional case.
  //! \param Q [in]  input points.
  //! \param v [out] calculated parameter values in V direction.
  //! \return error code.
  static ErrCode Calculate_V(const std::vector< std::vector<xyz> >& Q,
                             double*                                v)
  {
    const int n = (int) Q.size() - 1;
    if ( n <= 0 )
      return ErrCode_InvalidGridDimensions;

    const int m = (int) Q.at(0).size() - 1;
    if ( m <= 0 )
      return ErrCode_InvalidGridDimensions;

    core_HeapAlloc<double> alloc;
    core_HeapAlloc2D<double> alloc2d;

    //-----------------------------------
    // Calculate lengths of each isoline
    //-----------------------------------

    // Arrays for polyline lengths
    double* d_isoU = alloc.Allocate(n + 1, false);

    // Loop over the U direction to calculate the sum of chord lengths
    // for each U-isoline
    for ( int k = 0; k <= n; ++k )
    {
      // Loop over the V direction
      for ( int l = 1; l <= m; ++l )
      {
        xyz    QQ =  Q[k][l] - Q[k][l-1];
        d_isoU[k] += sqrt( QQ.Modulus() );
      }
    }

    //----------------------------------------------------
    // Calculate non-averaged parameters for each isoline
    //----------------------------------------------------

    // Parameters per single isolines
    double** param_isoU = alloc2d.Allocate(n + 1, m + 1, false);

    // Calculate reper V parameters. Now loop over the U direction (rows)
    for ( int i = 0; i <= n; ++i )
    {
      // Bind first parameters to 0
      param_isoU[i][0] = 0.0;

      // Loop over the V direction calculating parameters
      for ( int j = 1; j < m; ++j )
      {
        xyz           QQ = Q[i][j] - Q[i][j-1];
        param_isoU[i][j] = param_isoU[i][j-1] + sqrt( QQ.Modulus() )/d_isoU[i];
      }

      // Bind last parameters to 1
      param_isoU[i][m] = 1.0;
    }

    //----------------------------------
    // Apply averaging for V parameters
    //----------------------------------

    // First parameter
    v[0] = 0.0;

    // Intermediate parameters
    for ( int j = 1; j < m; ++j )
    {
      double sum_overU = 0.0;
      for ( int i = 0; i <= n; ++i )
        sum_overU += param_isoU[i][j];

      v[j] = sum_overU / (n + 1);
    }

    // Last parameter
    v[m] = 1.0;

    // Success
    return ErrCode_NoError;
  }

private:

  bspl_ParamsCentripetal() {}
  bspl_ParamsCentripetal(const bspl_ParamsCentripetal&) {}

};

};

#endif
