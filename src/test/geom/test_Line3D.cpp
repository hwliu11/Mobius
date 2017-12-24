//-----------------------------------------------------------------------------
// Created on: 22 May 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/test_Line3D.h>

// geom includes
#include <mobius/geom_Line.h>

//! Test scenario 001.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_Line3D::test1(const int funcID)
{
  // Line properties
  core_XYZ O(1, 1, 1);
  core_XYZ Dir(1, 2, 3);

  // Set description variables
  SetVarDescr("Origin",    O,   ID(), funcID);
  SetVarDescr("Direction", Dir, ID(), funcID);

  // Construct line
  core_Ptr<geom_Line> L = new geom_Line(O, Dir);

  // Calculate bounding box
  double xMin, yMin, zMin, xMax, yMax, zMax;
  L->Bounds(xMin, xMax, yMin, yMax, zMin, zMax);

  // Evaluate line
  core_XYZ P[3];
  L->Eval( 0.0, P[0]);
  L->Eval( 1.0, P[1]);
  L->Eval(-1.0, P[2]);

  // Referential results
  core_XYZ P_ref[] =
  { core_XYZ(1, 1, 1),
    core_XYZ(1.2672612, 1.5345224, 1.8017837),
    core_XYZ(0.7327387, 0.4654775, 0.1982162) };

  /* ====================================
   *  Verify calculation of bounding box
   * ==================================== */

  const double prec = 1.0e-6;

  if ( xMin != -DBL_MAX )
    return false;
  if ( yMin != -DBL_MAX )
    return false;
  if ( zMin != -DBL_MAX )
    return false;
  if ( xMax != DBL_MAX )
    return false;
  if ( yMax != DBL_MAX )
    return false;
  if ( zMax != DBL_MAX )
    return false;

  /* ===========================
   *  Verify evaluation results
   * =========================== */

  size_t num_pts = sizeof(P)/sizeof(core_XYZ);
  for ( size_t pt_idx = 0; pt_idx < num_pts; ++pt_idx )
  {
    const double dist = (P[pt_idx] - P_ref[pt_idx]).Modulus();
    if ( dist > prec )
      return false;
  }

  return true;
}
