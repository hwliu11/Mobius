//-----------------------------------------------------------------------------
// Created on: 27 January 2020
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

// Poly includes
#include <mobius/poly_GridTessellator.h>

//-----------------------------------------------------------------------------

mobius::poly_GridTessellator::poly_GridTessellator(const t_xyz&       Pmin,
                                                   const t_xyz&       Pmax,
                                                   const int          numSlices,
                                                   core_ProgressEntry progress,
                                                   core_PlotterEntry  plotter)
: poly_Tessellator (progress, plotter),
  m_Pmin           (Pmin),
  m_Pmax           (Pmax),
  m_iNumSlices     (numSlices)
{
  const t_xyz& P0P7 = m_Pmax - m_Pmin;
  
  // Scale the passed number of slices w.r.t. the voxel shape.
  m_iNumSlicesX = int( numSlices * P0P7.X() / P0P7.GetMaxComponent() );
  m_iNumSlicesY = int( numSlices * P0P7.Y() / P0P7.GetMaxComponent() );
  m_iNumSlicesZ = int( numSlices * P0P7.Z() / P0P7.GetMaxComponent() );

  // Compute the voxel sizes in all three directions.
  m_fGrainX = P0P7.X() / m_iNumSlicesX;
  m_fGrainY = P0P7.Y() / m_iNumSlicesY;
  m_fGrainZ = P0P7.Z() / m_iNumSlicesZ;
}

//-----------------------------------------------------------------------------

mobius::poly_GridTessellator::~poly_GridTessellator()
{}
