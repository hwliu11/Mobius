//-----------------------------------------------------------------------------
// Created on: 25 February 2015
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of Sergey Slyadnev nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

// Own include
#include <mobius/visu_InterpSurfCmd.h>

// bspl includes
#include <mobius/bspl_KnotsSelection.h>
#include <mobius/bspl_ParamsSelection.h>

// geom includes
#include <mobius/geom_VectorField.h>

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ActorPositionCloud.h>
#include <mobius/visu_ActorVectorField.h>

//! Executes command.
//! \return true in case of success, false -- otherwise.
bool mobius::visu_InterpSurfCmd::Execute()
{
  std::cout << this->Name() << std::endl;

  //-------------------------------------------------------------------------
  // Some preparations
  //-------------------------------------------------------------------------

  if ( this->Argc() < 5 )
  {
    std::cout << "Error: you have to respect the following signature:" << std::endl;
    std::cout << "name num_points_in_row num_points_in_col u_degree v_degree" << std::endl;
    return false;
  }

  std::string surfName    = this->Arg(0);
  const int   num_in_rows = this->Arg<int>(1, 2);
  const int   num_in_cols = this->Arg<int>(2, 2);
  const int   deg_U       = this->Arg<int>(3, 2);
  const int   deg_V       = this->Arg<int>(4, 2);
  const bool  is_D1_start = (this->Arg<int>(5, 0) > 0);
  const bool  is_D1_end   = (this->Arg<int>(6, 0) > 0);
  const bool  is_D2_start = (this->Arg<int>(7, 0) > 0);
  const bool  is_D2_end   = (this->Arg<int>(8, 0) > 0);

  if ( is_D1_start || is_D1_end )
    std::cout << "Experimental mode with D1 constraints is enabled" << std::endl;

  if ( is_D2_start || is_D2_end )
    std::cout << "Experimental mode with D2 constraints is enabled" << std::endl;

  // Vector field to store D1 constraints
  t_ptr<t_vector_field> D1_start = new t_vector_field(new t_pcloud);
  t_ptr<t_vector_field> D1_end = new t_vector_field(new t_pcloud);

  // Vector field to store D2 constraints
  t_ptr<t_vector_field> D2_start = new t_vector_field(new t_pcloud);
  t_ptr<t_vector_field> D2_end = new t_vector_field(new t_pcloud);

  //-------------------------------------------------------------------------
  // Extract highlighted points
  //-------------------------------------------------------------------------

  // Get all points into a single collection first
  std::vector<t_xyz> point_series;
  TActorVec actors = this->Scene()->Actors();
  for ( size_t actor_idx = 0; actor_idx < actors.size(); ++actor_idx )
  {
    const t_ptr<visu_Actor>& actor = actors[actor_idx];

    visu_ActorPositionCloud*
      pclActor = dynamic_cast<visu_ActorPositionCloud*>( actor.Access() );

    if ( pclActor )
    {
      t_ptr<visu_DataPositionCloud>
        pclData = t_ptr<visu_DataPositionCloud>::DownCast( pclActor->GetHiliData() );

      const t_ptr<t_pcloud>& pcl = pclData->GetCloud();
      for ( int p_idx = 0; p_idx < pcl->GetNumberOfPoints(); ++p_idx )
      {
        const t_xyz& P = pcl->GetPoint(p_idx);
        point_series.push_back(P);
      }
    }
  }

  //-------------------------------------------------------------------------
  // Distribute points by rows and columns
  //-------------------------------------------------------------------------

  std::cout << "Number of points in a row is " << num_in_rows << std::endl;
  std::cout << "Number of points in a column is " << num_in_cols << std::endl;

  int row = -1, col = 0;
  std::vector< std::vector<t_xyz> > points;
  for ( size_t p = 0; p < point_series.size(); ++p )
  {
    if ( !(col % num_in_cols) )
    {
      points.push_back( std::vector<t_xyz>() );
      col = 0;
      ++row;
    }

    t_xyz& P = point_series[p];
    points[row].push_back(P);
    ++col;
  }

  //-------------------------------------------------------------------------
  // Prepare tangency constraints
  //-------------------------------------------------------------------------

  if ( is_D1_start )
  {
    for ( size_t p = 0; p < points[0].size(); ++p )
    {
      D1_start->ChangeCloud()->AddPoint( points[0][p] );
      D1_start->AddVector(p, t_xyz(50, 0, 0) );
    }

    double xMin, xMax, yMin, yMax, zMin, zMax;
    D1_start->GetCloud()->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
    const double d_start = ( t_xyz(xMin, yMin, zMin) - t_xyz(xMax, yMax, zMax) ).Modulus();

    t_ptr<visu_ActorVectorField>
      derivs_start_actor = new visu_ActorVectorField(D1_start, QrRed, d_start / 10.0);
    this->Scene()->Add( derivs_start_actor.Access(), "interp_tangency_start" );
  }
  else
    this->Scene()->Clear("interp_tangency_start");

  if ( is_D1_end )
  {
    for ( size_t p = 0; p < points[points.size()-1].size(); ++p )
    {
      D1_end->ChangeCloud()->AddPoint( points[points.size()-1][p] );
      D1_end->AddVector(p, t_xyz(50, 0, 0) );
    }

    double xMin, xMax, yMin, yMax, zMin, zMax;
    D1_end->GetCloud()->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
    const double d_end = ( t_xyz(xMin, yMin, zMin) - t_xyz(xMax, yMax, zMax) ).Modulus();

    t_ptr<visu_ActorVectorField>
      derivs_end_actor = new visu_ActorVectorField(D1_end, QrRed, d_end / 10.0);
    this->Scene()->Add( derivs_end_actor.Access(), "interp_tangency_end" );
  }
  else
    this->Scene()->Clear("interp_tangency_end");

  if ( is_D2_start )
  {
    for ( size_t p = 0; p < points[0].size(); ++p )
    {
      D2_start->ChangeCloud()->AddPoint( points[0][p] );
      D2_start->AddVector(p, t_xyz(0, 50, 50) );
    }

    double xMin, xMax, yMin, yMax, zMin, zMax;
    D2_start->GetCloud()->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
    const double d_start = ( t_xyz(xMin, yMin, zMin) - t_xyz(xMax, yMax, zMax) ).Modulus();

    t_ptr<visu_ActorVectorField>
      derivs_start_actor = new visu_ActorVectorField(D2_start, QrBlue, d_start / 10.0);
    this->Scene()->Add( derivs_start_actor.Access(), "interp_normal_start" );
  }
  else
    this->Scene()->Clear("interp_normal_start");

  if ( is_D2_end )
  {
    for ( size_t p = 0; p < points[points.size()-1].size(); ++p )
    {
      D2_end->ChangeCloud()->AddPoint( points[points.size()-1][p] );
      D2_end->AddVector(p, t_xyz(0, 50, 50) );
    }

    double xMin, xMax, yMin, yMax, zMin, zMax;
    D2_end->GetCloud()->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
    const double d_end = ( t_xyz(xMin, yMin, zMin) - t_xyz(xMax, yMax, zMax) ).Modulus();

    t_ptr<visu_ActorVectorField>
      derivs_end_actor = new visu_ActorVectorField(D2_end, QrBlue, d_end / 10.0);
    this->Scene()->Add( derivs_end_actor.Access(), "interp_normal_end" );
  }
  else
    this->Scene()->Clear("interp_normal_end");

  //-------------------------------------------------------------------------
  // Perform interpolation
  //-------------------------------------------------------------------------

  bspl_ParamsSelection paramsType = ParamsSelection_Centripetal;
  bspl_KnotsSelection  knotsType  = KnotsSelection_Average;

  geom_InterpolateSurface Interp(points, deg_U, deg_V,
                                 D1_start, D1_end,
                                 D2_start, D2_end,
                                 paramsType, knotsType);

  try
  {
    Interp.Perform();
  }
  catch ( std::exception ex )
  {
    std::cout << "Exception occurred: " << ex.what() << std::endl;
    return false;
  }

  if ( Interp.GetErrorCode() != geom_InterpolateSurface::ErrCode_NoError )
  {
    std::cout << "Error: Qr interpolation failed" << std::endl;
    return false;
  }

  // Render intermediate results
  for ( size_t iso_V_c = 0; iso_V_c < Interp.IsoV_Curves.size(); ++iso_V_c )
  {
    t_ptr<t_bcurve> iso_V = Interp.IsoV_Curves[iso_V_c];
    t_ptr<visu_Actor> actorIsoV = new visu_ActorBSplCurve(iso_V, QrGreen, false, true);

    // Add isocurve to Scene
    //this->Scene()->Add( actorIsoV, std::string("iso_V_") + core::str::to_string<size_t>(iso_V_c) );
  }

  // Render second pass results
  for ( size_t iso_U_c = 0; iso_U_c < Interp.ReperU_Curves.size(); ++iso_U_c )
  {
    t_ptr<t_bcurve> iso_U = Interp.ReperU_Curves[iso_U_c];
    t_ptr<visu_Actor> actorIsoU = new visu_ActorBSplCurve(iso_U, QrYellow, false, true);

    // Add isocurve to Scene
    //this->Scene()->Add( actorIsoU, std::string("iso_U_") + core::str::to_string<size_t>(iso_U_c) );
  }

  // Access constructed B-spline surface
  const t_ptr<t_bsurf>& surf = Interp.GetResult();

  // Actor for interpolant
  t_ptr<visu_ActorBSplSurface>
    actorInterp = new visu_ActorBSplSurface(surf);
  //actorInterp->SetPoles(true);

  // Add interpolant to Scene
  this->Scene()->Add(actorInterp.Access(), surfName);

  //-------------------------------------------------------------------------
  // Set variable to global context
  //-------------------------------------------------------------------------

  std::cout << "Surface '" << surfName << "' has been created" << std::endl;

  // Set var
  this->CmdRepo()->Vars().RegisterVariable( surfName, surf.Access() );

  //-------------------------------------------------------------------------
  // Adjust Scene
  //-------------------------------------------------------------------------

  // Reset Picker to original state
  this->Picker()->SetMode(visu_Picker::Mode_Single);

  return true;
}
