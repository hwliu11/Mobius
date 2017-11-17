//-----------------------------------------------------------------------------
// Created on: 02 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_ParamsChordLength_HeaderFile
#define bspl_ParamsChordLength_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_HeapAlloc.h>
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! This tool allows to choose interpolation parameters according to the
//! well-known chord length rule. It can handle both 1-dimensional and
//! 2-dimensional objects (keeping in mind that there is no problem to extend
//! it to higher dimensions, but we do not have a good reason to do that in
//! Mobius). In both cases the core idea is to choose parameters so that they
//! reflect the spatial distribution of the reper points:
//!
//! \verbatim
//!   t_0 = 0;
//!   ...
//!   t_k = t_k-1 + |Q_k - Q_k-1|/d;
//!   ...
//!   t_n = 1;
//! \endverbatim
//!
//! Here {Q_k : k=0,n} are points to interpolate; {t_k} are the resulting
//! parameters; d (= Sum(|Q_k - Q_k-1|)) is the total length of the polygon
//! built on the input points.
//!
//! For a 1-dimensional case the given formulation is enough, so we can end
//! up with this simply calculated values. However, in a 2-dimensional case
//! applying this rule will lead to different parameters calculated for
//! each row (column) in a reper grid. Generally we want to avoid such
//! issues as we want our reper points to lie on the isoparametric lines of
//! the resulting interpolant surface. Therefore, we need to apply some
//! additional unification processing, which is given in [Piegl, The NURBS Book]:
//! a simple averaging over the grid rows (columns):
//!
//! \verbatim
//!   u = ( Sum_{i=0}^{m}{u_0^i}, Sum_{i=0}^{m}{u_1^i}, ..., Sum_{i=0}^{m}{u_n^i} ) / m;
//!   v = ( Sum_{i=0}^{n}{v_0^i}, Sum_{i=0}^{n}{v_1^i}, ..., Sum_{i=0}^{n}{v_m^i} ) / n;
//! \endverbatim
//!
//! Notice that for each u-isoline under construction we average its reper
//! parameters over the v axis. The same is done for v-isolines.
class bspl_ParamsChordLength
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

  //! Calculates parameter values by chord length rule in 1-dimensional case.
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

    // Calculate d (total length)
    double d = 0.0;
    for ( int idx = 1; idx < len; ++idx )
    {
      xyz QQ =  Q.at(idx) - Q.at(idx-1);
      d      += QQ.Modulus();
    }

    // Calculate next parameter
    for ( int idx = 1; idx < len-1; ++idx )
    {
      xyz QQ = Q.at(idx) - Q.at(idx-1);
      t[idx] = t[idx-1] + QQ.Modulus() / d;
    }

    // Last parameter
    t[len-1] = 1.0;

    // Success
    return ErrCode_NoError;
  }

//---------------------------------------------------------------------------//

  //! Calculates parameter values by chord length rule in 2-dimensional case.
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
    //
    for ( int i = 0; i < n + 1; ++i ) d_isoU[i] = 0;
    for ( int i = 0; i < m + 1; ++i ) d_isoV[i] = 0;

    // Loop over the V direction to calculate the sum of chord lengths
    // for each V-isoline
    for ( int l = 0; l <= m; ++l )
    {
      // Loop over the U direction
      for ( int k = 1; k <= n; ++k )
      {
        xyz    QQ =  Q[k][l] - Q[k-1][l];
        d_isoV[l] += QQ.Modulus();
      }
    }

    // Loop over the U direction to calculate the sum of chord lengths
    // for each U-isoline
    for ( int k = 0; k <= n; ++k )
    {
      // Loop over the V direction
      for ( int l = 1; l <= m; ++l )
      {
        xyz    QQ =  Q[k][l] - Q[k][l-1];
        d_isoU[k] += QQ.Modulus();
      }
    }

    //----------------------------------------------------
    // Calculate non-averaged parameters for each isoline
    //----------------------------------------------------

    // Parameters per single isolines
    double** param_isoU = alloc2d.Allocate(n + 1, m + 1, false);
    double** param_isoV = alloc2d.Allocate(n + 1, m + 1, false);
    //
    for ( int i = 0; i < n + 1; ++i )
      for ( int j = 0; j < m + 1; ++j )
      {
        param_isoU[i][j] = 0;
        param_isoV[i][j] = 0;
      }

    // Calculate reper U parameters. Now loop over the V direction (columns)
    for ( int j = 0; j <= m; ++j )
    {
      // Bind first parameters to 0
      param_isoV[0][j] = 0.0;

      // Loop over the U direction calculating parameters
      for ( int i = 1; i < n; ++i )
      {
        xyz           QQ = Q[i][j] - Q[i-1][j];
        param_isoV[i][j] = param_isoV[i-1][j] + QQ.Modulus()/d_isoV[j];
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
        param_isoU[i][j] = param_isoU[i][j-1] + QQ.Modulus()/d_isoU[i];
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
    //
    for ( int i = 0; i < n + 1; ++i ) d_isoU[i] = 0;

    // Loop over the U direction to calculate the sum of chord lengths
    // for each U-isoline
    for ( int k = 0; k <= n; ++k )
    {
      // Loop over the V direction
      for ( int l = 1; l <= m; ++l )
      {
        xyz    QQ =  Q[k][l] - Q[k][l-1];
        d_isoU[k] += QQ.Modulus();
      }
    }

    //----------------------------------------------------
    // Calculate non-averaged parameters for each isoline
    //----------------------------------------------------

    // Parameters per single isolines
    double** param_isoU = alloc2d.Allocate(n + 1, m + 1, false);
    //
    for ( int i = 0; i < n + 1; ++i )
      for ( int j = 0; j < m + 1; ++j )
        param_isoU[i][j] = 0;

    // Calculate reper V parameters. Now loop over the U direction (rows)
    for ( int i = 0; i <= n; ++i )
    {
      // Bind first parameters to 0
      param_isoU[i][0] = 0.0;

      // Loop over the V direction calculating parameters
      for ( int j = 1; j < m; ++j )
      {
        xyz           QQ = Q[i][j] - Q[i][j-1];
        param_isoU[i][j] = param_isoU[i][j-1] + QQ.Modulus()/d_isoU[i];
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

  bspl_ParamsChordLength() {}
  bspl_ParamsChordLength(const bspl_ParamsChordLength&) {}

};

};

#endif
