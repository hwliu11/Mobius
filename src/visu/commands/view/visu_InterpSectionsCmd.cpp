//-----------------------------------------------------------------------------
// Created on: 03 March 2015
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
#include <mobius/visu_InterpSectionsCmd.h>

// geom includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_InterpolateCurve.h>
#include <mobius/geom_SectionCloud.h>

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ActorPositionCloud.h>
#include <mobius/visu_ActorSectionCloud.h>
#include <mobius/visu_ColorSelector.h>

//! Executes command.
//! \return true in case of success, false -- otherwise.
bool mobius::visu_InterpSectionsCmd::Execute()
{
  std::cout << this->Name().c_str() << std::endl;

  //-------------------------------------------------------------------------
  // Some preparations
  //-------------------------------------------------------------------------

  if ( this->Argc() != 1 )
  {
    std::cout << "Unexpected number of arguments" << std::endl;
    std::cout << "Example: interp_sections section_cloud_name wl_name n_pts" << std::endl;
    return false;
  }

  std::string scloud_name = this->Arg(0);

  // Access section cloud from variable
  t_ptr<core_OBJECT> scloud_obj = this->CmdRepo()->Vars().FindVariable(scloud_name);
  if ( scloud_obj.IsNull() )
  {
    std::cout << "No object with name " << scloud_name.c_str() << " exists" << std::endl;
    return false;
  }

  // Convert to section cloud
  t_ptr<t_scloud> sct_cloud = t_ptr<t_scloud>::DownCast(scloud_obj);
  if ( sct_cloud.IsNull() )
  {
    std::cout << "Object " << scloud_name.c_str() << " is not a section cloud" << std::endl;
    return false;
  }

  //----------------------------------------------------------------------------
  // Interpolate and disretize uniformly each section for searching smooth path
  //----------------------------------------------------------------------------

  const int num_pts = 40;

  for ( size_t s = 0; s < sct_cloud->GetNumberOfSections(); ++s )
  {
    const t_ptr<t_sline>& sct = sct_cloud->GetSectionByIndex(s);
    if ( sct->Slices.size() )
      continue;

    bool interp_done = false;
    sct->Interpolate(1, interp_done);
    if ( !interp_done )
    {
      std::cout << "Interpolation failed for section " << (s + 1) << std::endl;
      return false;
    }
    sct->Discrete(num_pts);
  }

  ////--------------------
  //// Create middle line
  ////--------------------

  //// TODO: all this code to geom_SectionCloud

  //// Loop over the available sections trying to find a position for "middle" line
  //t_xyz midline_start_pt;
  //int midline_start_sct_idx = -1, midline_end_sct_idx = -1;
  //for ( size_t s = 0; s < sct_cloud->NumberOfSections(); ++s )
  //{
  //  const t_ptr<sline>& sct = sct_cloud->SectionByIndex(s);
  //  if ( sct->Slices.size() != 2 )
  //    continue; // Skip non-sliced or too sliced sections

  //  // Joint point should be the same (detect if slices are sewn in a single node)
  //  const t_xyz& end_slice1   = sct->Slices[0]->Pts->GetPoint(sct->Slices[0]->Pts->GetNumberOfPoints() - 1);
  //  const t_xyz& start_slice2 = sct->Slices[1]->Pts->GetPoint(0);
  //  if ( !(end_slice1 - start_slice2).IsOrigin( core_Precision::Resolution3D()) )
  //    continue;

  //  // Access leading point of the mid-line being constructed
  //  const size_t n_pts_in_slice = sct->Slices[0]->Pts->GetNumberOfPoints();
  //  midline_start_pt            = sct->Slices[0]->Pts->GetPoint(n_pts_in_slice - 1);
  //  midline_start_sct_idx       = (int) s;
  //  break;
  //}

  //const int num_pts_per_patch = 10;
  //t_ptr<sline> midline = new sline(-1, new t_pcloud);
  //t_ptr<scloud> scloud_result = new scloud;

  //if ( midline_start_sct_idx != -1 )
  //{
  //  //-------------------------------------------------------------------------
  //  // Elaborate mid-line path between sliced sections
  //  //-------------------------------------------------------------------------

  //  // Section ID against break point
  //  std::map<int, size_t> break_pts_map;

  //  // Create mid-line and add leading point to it
  //  midline->Pts->AddPoint(midline_start_pt);

  //  // Working variables
  //  t_xyz midline_current_pt = midline_start_pt;

  //  // Find a chain of points closest to the current one in each section
  //  for ( size_t s = midline_start_sct_idx + 1; s < sct_cloud->NumberOfSections(); ++s )
  //  {
  //    midline_end_sct_idx = (int) s; // Rebind where we finish
  //    const t_ptr<sline>& next_sct = sct_cloud->SectionByIndex(s);
  //    if ( next_sct->Slices.size() )
  //      break; // Sliced line is a good condition to stop heuristics

  //    // Find closest point on the current section
  //    double min_dist   = DBL_MAX;
  //    t_xyz    pt_min     = midline_current_pt;
  //    size_t pt_min_idx = 0;
  //    for ( size_t ipt_next_sct = 0; ipt_next_sct < next_sct->Pts->GetNumberOfPoints(); ++ipt_next_sct )
  //    {
  //      const t_xyz& next_pt = next_sct->Pts->GetPoint(ipt_next_sct);
  //      const double dist = (midline_current_pt - next_pt).Modulus();

  //      if ( dist < min_dist )
  //      {
  //        min_dist   = dist;
  //        pt_min     = next_pt;
  //        pt_min_idx = ipt_next_sct;
  //      }
  //    }

  //    midline->Pts->AddPoint(pt_min);
  //    midline_current_pt = pt_min;
  //    break_pts_map.insert( std::pair<int, size_t>(next_sct->ID, pt_min_idx) );
  //  }

  //  t_ptr<visu_ActorPositionCloud>
  //    actor_midline_pts = new visu_ActorPositionCloud(midline->Pts, QrGreen);

  //  actor_midline_pts->SetPointSize(12);

  //  this->Scene()->Add(actor_midline_pts.Access(), "midline_pts");

  //  bool interp_done = false;
  //  midline->InterpolateAndDiscrete(num_pts, interp_done);
  //  if ( !interp_done )
  //  {
  //    std::cout << "Cannot interpolate midline" << std::endl;
  //    return false;
  //  }

  //  //-------------------------------------------------------------------------
  //  // Split the traversed sections by mid-line
  //  //-------------------------------------------------------------------------

  //  for ( std::map<int, size_t>::const_iterator it = break_pts_map.cbegin(); it != break_pts_map.cend(); it++ )
  //  {
  //    const t_ptr<sline>& sct = sct_cloud->SectionByID(it->first);
  //    bool splitting_done = false;
  //    sct->Split(it->second, splitting_done);
  //    if ( !splitting_done )
  //    {
  //      std::cout << "Cannot split section by mid-line" << std::endl;
  //      return false;
  //    }

  //    // Register slices
  //    for ( size_t ss = 0; ss < sct->Slices.size(); ++ss )
  //    {
  //      bool interp_done = false;
  //      sct->Slices[ss]->Interpolate(1, interp_done);

  //      if ( !interp_done )
  //      {
  //        std::cout << "Slice interpolation failed" << std::endl;
  //        return false;
  //      }

  //      // Prepare actor
  //      t_ptr<visu_ActorBSplCurve>
  //        ss_actor = new visu_ActorBSplCurve(sct->Slices[ss]->Curve, ss == 0 ? QrBlue : QrLightBlue, false, true, false);

  //      // Add actor to scene
  //      this->Scene()->Add( ss_actor.Access() );
  //    }
  //  }
  //}

  //---------------------------------------------------------------------
  // Classify remaining sections
  //---------------------------------------------------------------------

  // If there are still any sections which are not sliced, it normally
  // means that these sections lie completely below or above the mid-line.
  // In that case let us produce dummy slices (one contains the complete
  // section, while another contains nothing) in order to unify the
  // procedure

  //for ( size_t s = 0; s < sct_cloud->NumberOfSections(); ++s )
  //{
  //  const t_ptr<sline>& sct_line = sct_cloud->SectionByIndex(s);
  //  if ( sct_line->Slices.size() )
  //    continue;

  //  // Classify just by one point
  //  const t_xyz& pt = sct_line->Pts->GetPoint(0);
  //  if ( pt.Z() > midline_start_pt.Z() )
  //  {
  //    sct_line->Slices.push_back(NULL);
  //    sct_line->Slices.push_back(sct_line);
  //  }
  //  else
  //  {
  //    sct_line->Slices.push_back(sct_line);
  //    sct_line->Slices.push_back(NULL);
  //  }
  //}

  //---------------------------------------------------------------------
  // Produce intermediate waterlines
  //---------------------------------------------------------------------

  //// Loop over the curve to have uniform discretization
  //{
  //  const double uMin = 0.0;
  //  const double uMax = 1.0;
  //  const double step = (uMax - uMin) / (num_pts_per_patch - 1);
  //  for ( int pnt_idx = 0; pnt_idx < num_pts_per_patch - 1; ++pnt_idx )
  //  {
  //    double u = uMin + pnt_idx*step;

  //    if ( fabs(u - uMax) < 1.0e-10 )
  //    {
  //      //u = uMax; // Just to compensate round off errors
  //      break; // Substituted by mid-line
  //    }

  //    // We are going to produce discretization for waterline sections
  //    t_ptr<sline> wl_section_line = new sline;
  //    wl_section_line->Pts = new t_pcloud;

  //    for ( size_t s = /*midline_start_sct_idx*/0; s </*= midline_end_sct_idx*/sct_cloud->NumberOfSections(); ++s )
  //    {
  //      const t_ptr<sline>& sct_line = sct_cloud->SectionByIndex(s)->Slices[0];
  //      if ( sct_line.IsNull() )
  //        continue;

  //      const t_ptr<t_bcurve>& sct_curve = sct_line->Curve;
  //      if ( sct_curve.IsNull() )
  //      {
  //        std::cout << "Section curve is NULL" << std::endl;
  //        return false;
  //      }

  //      t_xyz C;
  //      sct_curve->Eval(u, C);
  //      wl_section_line->Pts->AddPoint(C);
  //    }

  //    // Register section in our working rectangular grid
  //    scloud_result->AddSection(wl_section_line);
  //  }
  //}

  //if ( !midline->Pts.IsNull() )
  //  scloud_result->AddSection(midline);

  //{
  //  // Loop over the curve to have uniform discretization
  //  const double uMin = 0.0;
  //  const double uMax = 1.0;
  //  const double step = (uMax - uMin) / (num_pts_per_patch - 1);
  //  for ( int pnt_idx = 1; pnt_idx < num_pts_per_patch; ++pnt_idx ) // Start from 1 as we want to have mid-line as a first section
  //  {
  //    double u = uMin + pnt_idx*step;

  //    if ( fabs(u - uMax) < 1.0e-10 )
  //    {
  //      u = uMax; // Just to compensate round off errors
  //      //break; // Substituted by mid-line
  //    }

  //    // We are going to produce discretization for waterline sections
  //    t_ptr<sline> wl_section_line = new sline;
  //    wl_section_line->Pts = new t_pcloud;

  //    for ( size_t s = /*midline_start_sct_idx*/0; s </*= midline_end_sct_idx*/sct_cloud->NumberOfSections(); ++s )
  //    {
  //      const t_ptr<sline>& sct_line = sct_cloud->SectionByIndex(s)->Slices[1];
  //      if ( sct_line.IsNull() )
  //        continue;

  //      const t_ptr<t_bcurve>& sct_curve = sct_line->Curve;
  //      if ( sct_curve.IsNull() )
  //      {
  //        std::cout << "Section curve is NULL" << std::endl;
  //        return false;
  //      }

  //      t_xyz C;
  //      sct_curve->Eval(u, C);
  //      wl_section_line->Pts->AddPoint(C);
  //    }

  //    // Register section in our working rectangular grid
  //    scloud_result->AddSection(wl_section_line);
  //  }
  //}

  /////////////////////////////////////////////////////////////////////////////

  // Transom patches to be added

  /////////////////////////////////////////////////////////////////////////////

  //for ( int s = 0; s <= midline_start_sct_idx/*sct_cloud->NumberOfSections()*/; ++s )
  //{
  //  const t_ptr<sline>& sct_line = sct_cloud->SectionByIndex(s)->Slices[0];
  //  if ( sct_line.IsNull() )
  //    continue;

  //  scloud_result->AddSection(sct_line);
  //}

  //for ( int s = 0; s <= midline_start_sct_idx/*sct_cloud->NumberOfSections()*/; ++s )
  //{
  //  const t_ptr<sline>& sct_line = sct_cloud->SectionByIndex(s)->Slices[1];
  //  if ( sct_line.IsNull() )
  //    continue;

  //  scloud_result->AddSection(sct_line);
  //}


  //std::vector< std::vector<t_xyz> > levels = scloud_result->Points();

  //// Render grid
  //t_ptr<visu_ActorPositionCloud>
  //  wl_grid_actor = new visu_ActorPositionCloud(scloud_result->ToPositionCloud(), QrBlue);
  //wl_grid_actor->SetPointSize(8);

  //// Add to scene
  //this->Scene()->Add(wl_grid_actor.Access(), "wl_grid");

  /////

  //for ( size_t l = 0; l < levels.size(); ++l )
  //{
  //  std::string wl_name = "wl_" + core::str::to_string(l + 1);
  //  geom_InterpolateCurve InterpCrv(levels[l], 1,
  //                                      ParamsSelection_ChordLength,
  //                                      KnotsSelection_Average);
  //  try
  //  {
  //    InterpCrv.Perform();
  //  }
  //  catch ( ... )
  //  {
  //    std::cout << "Waterline interpolation failed" << std::endl;
  //    return false;
  //  }

  //  // Register section in a variable
  //  t_ptr<sline> wl_section = new sline( (int) l, levels[l] );
  //  wl_section->Curve = InterpCrv.GetResult();
  //  this->CmdRepo()->Vars().RegisterVariable( wl_name, wl_section.Access() );

  //  // Prepare actor
  //  t_ptr<visu_ActorBSplCurve>
  //    bcurve_actor = new visu_ActorBSplCurve(InterpCrv.GetResult(), QrRed, false, true, false);

  //  // Populate scene
  //  this->Scene()->Add(bcurve_actor.Access(), wl_name);
  //}

  return true;
}
