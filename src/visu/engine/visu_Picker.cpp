//-----------------------------------------------------------------------------
// Created on: 21 May 2014
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
#include <mobius/visu_Picker.h>

// visu includes
#include <mobius/visu_ActorLine.h>

#define visuPicker_RayRange 10

//! Constructor.
//! \param Scene [in] Scene instance.
mobius::visu_Picker::visu_Picker(const t_ptr<visu_Scene>& Scene)
: core_OBJECT(),
  m_scene(Scene)
{
}

//! Picks the scene.
//! \param mouseX [in] value on OX axis passing from left to right.
//! \param mouseY [in] value on OY axis passing from top to bottom.
void mobius::visu_Picker::Pick(const int mouseX,
                               const int mouseY)
{
  //---------------------------------------------------------------------------
  // Get picked point in the world CS with applied Camera's
  // transformation
  //---------------------------------------------------------------------------

  GLdouble X, Y, Z;
  m_scene->Camera()->Push();
  visu_Utils::DisplayToWorld(mouseX, mouseY, X, Y, Z);
  m_scene->Camera()->Pop();

  //---------------------------------------------------------------------------
  // Get picked point reverting Camera's transformation so as to have this
  // point snapped to the frozen XOY plane. This point in a couple with
  // the picked one gives an intuitive ray which can be used for picking
  //---------------------------------------------------------------------------

  // Get world coordinates without Camera transformation
  GLdouble X_fixed, Y_fixed, Z_fixed;
  visu_Utils::DisplayToWorld(mouseX, mouseY, X_fixed, Y_fixed, Z_fixed);

  // Revert transformation of Camera (Q^-1 P Q) as we have to
  // compensate this transformation applied during the scene rendering
  core_Quaternion qn = m_scene->Camera()->Rotation();
  core_Quaternion XYZ_res_qn = qn.Inverted() * t_xyz(X_fixed, Y_fixed, 0.0) * qn;
  t_xyz XYZ_res( XYZ_res_qn.Qx(), XYZ_res_qn.Qy(), XYZ_res_qn.Qz() );

  //---------------------------------------------------------------------------
  // Actor used for highlighting
  //---------------------------------------------------------------------------

  // Prepare picking ray
  t_xyz P1(XYZ_res), P2(X, Y, Z);
  t_xyz Dir = P2 - P1;
  t_ptr<geom_Line> Ray = new geom_Line(P1, Dir);

  // Get size of the scene
  double scene_xMin, scene_xMax, scene_yMin, scene_yMax, scene_zMin, scene_zMax;
  m_scene->GetBounds(scene_xMin, scene_xMax, scene_yMin, scene_yMax, scene_zMin, scene_zMax);
  t_xyz scene_min(scene_xMin, scene_yMin, scene_zMin),
      scene_max(scene_xMax, scene_yMax, scene_zMax);
  const double diag = (scene_max - scene_min).Modulus();

  // Install Actor for picking ray
  t_ptr<visu_Actor> rayActor = new visu_ActorLine(Ray, -diag/2, diag/2, true);
  m_scene->SetPickingRay(rayActor);

  //---------------------------------------------------------------------------
  // Intersect Actors with ray
  //---------------------------------------------------------------------------

  const TActorVec& actors = m_scene->Actors();
  for ( size_t aidx = 0; aidx < actors.size(); ++aidx )
  {
    const t_ptr<visu_Actor>& actor = actors.at(aidx);

    // Find intersected data
    t_ptr<visu_DataSet> hili_data = actor->IntersectWithLine(Ray);
    if ( hili_data.IsNull() )
    {
      //actor->ClearHiliData(); // TODO: make this a regime
      continue;
    }

    // Join data sets if requested
    if ( m_mode == Mode_Join )
    {
      t_ptr<visu_DataSet> prev_hili_data = actor->GetHiliData();

      // Join in the order-preserving way: previous go first
      if ( !prev_hili_data.IsNull() )
      {
        prev_hili_data->Join(hili_data);
        hili_data = prev_hili_data;
      }
    }

    // Set data to highlight
    actor->SetHiliData(hili_data);
  }
}

//! Sets picking mode.
//! \param mode [in] mode to set.
void mobius::visu_Picker::SetMode(const Mode mode)
{
  m_mode = mode;
}

//! Returns picking mode.
//! \return picking mode.
mobius::visu_Picker::Mode mobius::visu_Picker::GetMode() const
{
  return m_mode;
}

//! Returns the working Scene.
//! \return Scene instance.
const mobius::t_ptr<mobius::visu_Scene>&
  mobius::visu_Picker::Scene() const
{
  return m_scene;
}
