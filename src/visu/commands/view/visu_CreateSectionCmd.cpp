//-----------------------------------------------------------------------------
// Created on: 06 March 2015
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
#include <mobius/visu_CreateSectionCmd.h>

// geom includes
#include <mobius/geom_InterpolateCurve.h>
#include <mobius/geom_SectionCloud.h>
#include <mobius/geom_SectionLine.h>

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ActorPositionCloud.h>

// standard includes
#include <algorithm>

//! Executes command.
//! \return true in case of success, false -- otherwise.
bool mobius::visu_CreateSectionCmd::Execute()
{
  std::cout << this->Name() << std::endl;

  //-------------------------------------------------------------------------
  // Some preparations
  //-------------------------------------------------------------------------

  if ( this->Argc() != 3 )
  {
    std::cout << "Unexpected number of arguments" << std::endl;
    std::cout << "Example: create_section scloud_name section_name degree" << std::endl;
    return false;
  }

  std::string scloud_name  = this->Arg(0);
  std::string section_name = this->Arg(1);
  const int   desired_deg  = this->Arg<int>(2, 3);

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

      if ( pclData.IsNull() )
        continue;

      const t_ptr<t_pcloud>& pcl = pclData->GetCloud();
      for ( int p_idx = 0; p_idx < pcl->GetNumberOfPoints(); ++p_idx )
      {
        const t_xyz& P = pcl->GetPoint(p_idx);
        point_series.push_back(P);
      }
    }
  }

  //-------------------------------------------------------------------------
  // Interpolate points creating a section
  //-------------------------------------------------------------------------

  const int deg = std::min( (int) (point_series.size() - 1), desired_deg );
  geom_InterpolateCurve Interp(point_series, deg, ParamsSelection_Centripetal, KnotsSelection_Average);
  Interp.Perform();
  t_ptr<t_bcurve> sct_crv = Interp.GetResult();

  // Create section wrapper
  t_ptr<t_sline> sct = new t_sline;
  sct->Curve = sct_crv;
  sct->Pts   = new t_pcloud(point_series);

  //-------------------------------------------------------------------------
  // Adjust Scene
  //-------------------------------------------------------------------------

  // Set variable
  this->CmdRepo()->Vars().RegisterVariable( section_name, sct.Access() );

  // Prepare actor
  t_ptr<visu_ActorBSplCurve>
    ss_actor = new visu_ActorBSplCurve(sct_crv, QrYellow, false, true, false);

  // Add actor to scene
  this->Scene()->Add( ss_actor.Access(), section_name );

  // Reset Picker to original state
  this->Picker()->SetMode(visu_Picker::Mode_Single);

  return true;
}
