//-----------------------------------------------------------------------------
// Created on: 12 December 2019
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

// Poly includes
#include <mobius/poly_DistanceField.h>

//-----------------------------------------------------------------------------

mobius::poly_DistanceField::poly_DistanceField(core_ProgressEntry progress,
                                               core_PlotterEntry  plotter)
: core_OBJECT (),
  m_pRoot     (nullptr),
  m_progress  (progress),
  m_plotter   (plotter)
{}

//-----------------------------------------------------------------------------

mobius::poly_DistanceField::~poly_DistanceField()
{
  if ( m_pRoot )
    delete m_pRoot; // Call SVO dtor which releases the octree recursively.
}

//-----------------------------------------------------------------------------

bool mobius::poly_DistanceField::Build(const double                    resolution,
                                       const t_ptr<poly_DistanceFunc>& func)
{
  if ( func.IsNull() )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Null function.");
    return false;
  }

  const t_xyz& cornerMin = func->GetDomainMin();
  const t_xyz& cornerMax = func->GetDomainMax();

  /* ==================
   *  Create root node.
   * ================== */

  if ( m_pRoot )
    delete m_pRoot; // Call SVO dtor which releases the octree recursively.

  // Create root SVO node and initialize it with the distance scalars.
  m_pRoot = new poly_SVO(cornerMin, cornerMax);
  //
  for ( size_t nx = 0; nx < 1; ++nx )
  {
    const double x = ( (nx == 0) ? cornerMin.X() : cornerMax.X() );
    for ( size_t ny = 0; ny < 1; ++ny )
    {
      const double y = ( (ny == 0) ? cornerMin.Y() : cornerMax.Y() );
      for ( size_t nz = 0; nz < 1; ++nz )
      {
        const double z = ( (nz == 0) ? cornerMin.Z() : cornerMax.Z() );

        // Evaluate distance.
        const double f = func->Eval(x, y, z);

        // Set scalar.
        m_pRoot->SetScalar(poly_SVO::GetCornerID(nx, ny, nz), f);
      }
    }
  }

  /* ==================
   *  Create hierarchy.
   * ================== */

  // TODO: NYI

  return false;
}
