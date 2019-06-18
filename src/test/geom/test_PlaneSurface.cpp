//-----------------------------------------------------------------------------
// Created on: 17 June 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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
#include <mobius/test_PlaneSurface.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Geom includes
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_PlaneSurface.h>

// Core includes
#include <mobius/core_Precision.h>

#undef FILE_DEBUG
#if defined FILE_DEBUG
  #pragma message("===== warning: FILE_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

//! Test scenario 001: convert plane to B-surface.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_PlaneSurface::toBSurf01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Construct plane.
  t_ptr<t_plane> plane = new t_plane( t_xyz::O(), t_xyz::OX(), t_xyz::OY() );
  //
  plane->SetLimits(-10., 10., -20., 20.);

  // Convert to B-surface.
  t_ptr<t_bsurf> bsurf = plane->ToBSurface(3, 3);
  //
  if ( bsurf.IsNull() )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002: inverts point to plane.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_PlaneSurface::invertPoint01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_xyz Q(1., 1., 1.);
  t_uv  uvRef(1., 1.);

  // Construct plane.
  t_ptr<t_plane> plane = new t_plane( t_xyz::O(), t_xyz::OX(), t_xyz::OY() );

  // Invert point.
  t_uv uv;
  plane->InvertPoint(Q, uv);

  // Verify.
  if ( (uv - uvRef).Modulus() > core_Precision::Resolution2D() )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 003: inverts point to plane.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_PlaneSurface::invertPoint02(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Points to invert.
  std::vector<t_xyz> Qs =
  {
    t_xyz(-1.4860999584197998, 3.7158999443054199, 1.5206999778747559),
    t_xyz(-0.60434997081756592, 3.3354101181030273, 1.9299999475479126),
    t_xyz(-2.3932499885559082, 2.8939800262451172, 1.497249960899353)
  };

  // Reference images.
  std::vector<t_uv> uvRefs =
  {
    t_uv(-0.143458014213316, -0.27858416319529877),
    t_uv(-0.15410975737311439, -1.1390696154435549),
    t_uv(-0.98716456624088567, 0.54156815221012056)
  };

  // Construct plane which is oriented in custom way.
  t_ptr<t_plane> plane = new t_plane( t_xyz(-1.5284546451612899, 3.5535670719602965, 1.1804265434491319),
                                      t_xyz(0.52146743448289168, 0.47125999594087664, -0.71132673997231122),
                                      t_xyz(-0.74285812414181951, -0.15940910178539608, -0.6501926988704736) );

  // Invert and verify.
  for ( size_t k = 0; k < Qs.size(); ++k )
  {
    // Invert point.
    t_uv uv;
    plane->InvertPoint(Qs[k], uv);

    if ( (uv - uvRefs[k]).Modulus() > core_Precision::Resolution2D() )
      return res.failure();
  }

  return res.success();
}
