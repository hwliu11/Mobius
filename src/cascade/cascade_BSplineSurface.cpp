//-----------------------------------------------------------------------------
// Created on: 24 December 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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
#include <mobius/cascade_BSplineSurface.h>

// Cascade includes
#include <mobius/cascade_MultResolver.h>

// OCCT includes
#include <gp_Pnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_HArray1OfInteger.hxx>
#include <TColStd_HArray1OfReal.hxx>

//! Converts Mobius b-surface to OCCT b-surface directly (without any
//! re-approximation or any other nontrivial stuff).
//! \param source [in] Mobius surface to convert.
//! \return OCCT surface or NULL if conversion is impossible for some reason.
Handle(Geom_BSplineSurface)
  mobius::cascade_BSplineSurface::FromMobius(const Ptr<bsurf>& source)
{
  // Degrees in U and V directions
  const Standard_Integer uDeg = source->Degree_U();
  const Standard_Integer vDeg = source->Degree_V();

  // Mobius properties of b-surface
  const std::vector< std::vector<xyz> >& qr_Poles  = source->Poles();
  std::vector<Standard_Real>             qr_UKnots = source->Knots_U();
  std::vector<Standard_Real>             qr_VKnots = source->Knots_V();

  // Poles are transferred as-is
  TColgp_Array2OfPnt occt_Poles( 1, (Standard_Integer) qr_Poles.size(),
                                 1, (Standard_Integer) qr_Poles[0].size() );
  for ( Standard_Integer i = occt_Poles.LowerRow(); i <= occt_Poles.UpperRow(); ++i )
  {
    for ( Standard_Integer j = occt_Poles.LowerCol(); j <= occt_Poles.UpperCol(); ++j )
    {
      const xyz& P = qr_Poles[i-1][j-1];
      occt_Poles(i, j) = gp_Pnt( P.X(), P.Y(), P.Z() );
    }
  }

  // Resolve U knots
  cascade_MultResolver uResolver;
  for ( int i = 0; i < (int) qr_UKnots.size(); ++i )
    uResolver.Resolve(qr_UKnots[i]);

  // Resolve V knots
  cascade_MultResolver vResolver;
  for ( int i = 0; i < (int) qr_VKnots.size(); ++i )
    vResolver.Resolve(qr_VKnots[i]);

  // Access OCCT collections
  Handle(TColStd_HArray1OfReal)    occt_UKnots = uResolver.GetOpenCascadeKnots();
  Handle(TColStd_HArray1OfInteger) occt_UMults = uResolver.GetOpenCascadeMults();
  Handle(TColStd_HArray1OfReal)    occt_VKnots = vResolver.GetOpenCascadeKnots();
  Handle(TColStd_HArray1OfInteger) occt_VMults = vResolver.GetOpenCascadeMults();

  // Construct OCCT surface
  Handle(Geom_BSplineSurface)
    result = new Geom_BSplineSurface(occt_Poles,
                                     occt_UKnots->Array1(),
                                     occt_VKnots->Array1(),
                                     occt_UMults->Array1(),
                                     occt_VMults->Array1(),
                                     uDeg,
                                     vDeg,
                                     /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
                                     /* Periodic surfaces are not supported in Mobius currently (winter, 2014) */
                                     /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
                                     Standard_False, 
                                     Standard_False);

  return result;
}
