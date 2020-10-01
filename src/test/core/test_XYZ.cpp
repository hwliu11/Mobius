//-----------------------------------------------------------------------------
// Created on: 01 October 2020
//-----------------------------------------------------------------------------
// Copyright (c) 2020-present, Sergey Slyadnev
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
#include <mobius/test_XYZ.h>

// core includes
#include <mobius/core_XYZ.h>

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_XYZ::are_same_plane(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  {
    std::vector<t_xyz> Vs = { t_xyz(1., 0., 0.),
                              t_xyz(0., 1., 0.),
                              t_xyz(1., 1., 0.) };

    if ( !t_xyz::AreSamePlane(Vs) )
      return res.failure();
  }

  {
    std::vector<t_xyz> Vs = { t_xyz(1., 0., 0.),
                              t_xyz(0., 1., 0.),
                              t_xyz(0., 0., 1.) };

    if ( t_xyz::AreSamePlane(Vs) )
      return res.failure();
  }

  {
    std::vector<t_xyz> Vs = { t_xyz(1., 0., 0.),
                              t_xyz(0., -1., 0.),
                              t_xyz(0., 0., 1.) };

    if ( t_xyz::AreSamePlane(Vs) )
      return res.failure();
  }

  {
    std::vector<t_xyz> Vs = { t_xyz(1.06058e-16, 1, 0),
                              t_xyz(-1.06058e-16, -1, 0),
                              t_xyz(0.5, -5.30288e-17, 0.866025),
                              t_xyz(-0.5, 5.30288e-17, -0.866025) };

    if ( !t_xyz::AreSamePlane(Vs) )
      return res.failure();
  }

  return res.success();
}
