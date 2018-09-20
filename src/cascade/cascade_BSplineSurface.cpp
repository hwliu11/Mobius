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

//-----------------------------------------------------------------------------
// Mobius-OCCT connector
//-----------------------------------------------------------------------------

//! Constructor.
//! \param[in] mobiusSurface Mobius surface to convert.
mobius::cascade_BSplineSurface::cascade_BSplineSurface(const ptr<bsurf>& mobiusSurface)
{
  m_mobiusSurface = mobiusSurface;
  m_bIsDone       = false;
}

//-----------------------------------------------------------------------------

//! Constructor.
//! \param[in] occtSurface OCCT surface to convert.
mobius::cascade_BSplineSurface::cascade_BSplineSurface(const Handle(Geom_BSplineSurface)& occtSurface)
{
  m_occtSurface = occtSurface;
  m_bIsDone     = false;
}

//-----------------------------------------------------------------------------

//! Destructor.
mobius::cascade_BSplineSurface::~cascade_BSplineSurface()
{}

//-----------------------------------------------------------------------------

//! Converts Mobius B-spline surface to OCCT one or vice versa by direct
//! supplying of knots, multiplicities and poles.
void mobius::cascade_BSplineSurface::DirectConvert()
{
  if ( !m_mobiusSurface.IsNull() )
    this->convertToOpenCascade();
  else if ( !m_occtSurface.IsNull() )
    this->convertToMobius();
}

//-----------------------------------------------------------------------------

//! Accessor for the Mobius surface.
//! \return Mobius surface.
const mobius::ptr<mobius::bsurf>&
  mobius::cascade_BSplineSurface::GetMobiusSurface() const
{
  return m_mobiusSurface;
}

//-----------------------------------------------------------------------------

//! Accessor for the OpenCascade surface.
//! \return OpenCascade surface.
const Handle(Geom_BSplineSurface)&
  mobius::cascade_BSplineSurface::GetOpenCascadeSurface() const
{
  return m_occtSurface;
}

//-----------------------------------------------------------------------------

//! Returns true if the result is accessible, false -- otherwise.
//! \return true/false.
bool mobius::cascade_BSplineSurface::IsDone() const
{
  return m_bIsDone;
}

//-----------------------------------------------------------------------------

//! Converts Mobius B-surface to OCCT B-surface.
void mobius::cascade_BSplineSurface::convertToOpenCascade()
{
  // Degrees in U and V directions.
  const int uDeg = m_mobiusSurface->Degree_U();
  const int vDeg = m_mobiusSurface->Degree_V();

  // Mobius properties of B-surface.
  const std::vector< std::vector<xyz> >& mobius_Poles  = m_mobiusSurface->Poles();
  std::vector<double>                    mobius_UKnots = m_mobiusSurface->Knots_U();
  std::vector<double>                    mobius_VKnots = m_mobiusSurface->Knots_V();

  // Poles are transferred as-is.
  TColgp_Array2OfPnt occt_Poles( 1, (int) mobius_Poles.size(),
                                 1, (int) mobius_Poles[0].size() );
  //
  for ( int i = occt_Poles.LowerRow(); i <= occt_Poles.UpperRow(); ++i )
  {
    for ( int j = occt_Poles.LowerCol(); j <= occt_Poles.UpperCol(); ++j )
    {
      const xyz& P = mobius_Poles[i-1][j-1];
      occt_Poles(i, j) = gp_Pnt( P.X(), P.Y(), P.Z() );
    }
  }

  // Resolve U knots.
  cascade_MultResolver uResolver;
  for ( int i = 0; i < (int) mobius_UKnots.size(); ++i )
    uResolver.Resolve(mobius_UKnots[i]);

  // Resolve V knots.
  cascade_MultResolver vResolver;
  for ( int i = 0; i < (int) mobius_VKnots.size(); ++i )
    vResolver.Resolve(mobius_VKnots[i]);

  // Access OCCT collections.
  Handle(TColStd_HArray1OfReal)    occt_UKnots = uResolver.GetOpenCascadeKnots();
  Handle(TColStd_HArray1OfInteger) occt_UMults = uResolver.GetOpenCascadeMults();
  Handle(TColStd_HArray1OfReal)    occt_VKnots = vResolver.GetOpenCascadeKnots();
  Handle(TColStd_HArray1OfInteger) occt_VMults = vResolver.GetOpenCascadeMults();

  // Construct OCCT surface.
  m_occtSurface = new Geom_BSplineSurface(occt_Poles,
                                          occt_UKnots->Array1(),
                                          occt_VKnots->Array1(),
                                          occt_UMults->Array1(),
                                          occt_VMults->Array1(),
                                          uDeg,
                                          vDeg,
                                          /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
                                          /* Periodic surfaces are not supported in Mobius currently (winter, 2014) */
                                          /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
                                          false, 
                                          false);

  m_bIsDone = true;
}

//-----------------------------------------------------------------------------

//! Converts OCCT B-surface to Mobius B-surface.
void mobius::cascade_BSplineSurface::convertToMobius()
{
  // Degrees in U and V directions.
  const int uDeg = m_occtSurface->UDegree();
  const int vDeg = m_occtSurface->VDegree();

  // OCCT properties of B-surface.
  const TColgp_Array2OfPnt&      occt_Poles  = m_occtSurface->Poles();
  const TColStd_Array1OfReal&    occt_UKnots = m_occtSurface->UKnots();
  const TColStd_Array1OfReal&    occt_VKnots = m_occtSurface->VKnots();
  const TColStd_Array1OfInteger& occt_UMults = m_occtSurface->UMultiplicities();
  const TColStd_Array1OfInteger& occt_VMults = m_occtSurface->VMultiplicities();
  
  // Poles are transferred as-is.
  std::vector< std::vector<xyz> > mobius_Poles;
  //
  for ( int i = occt_Poles.LowerRow(); i <= occt_Poles.UpperRow(); ++i )
  {
    std::vector<xyz> row;

    for ( int j = occt_Poles.LowerCol(); j <= occt_Poles.UpperCol(); ++j )
    {
      const gp_Pnt& P = occt_Poles(i, j);
      row.push_back( xyz( P.X(), P.Y(), P.Z() ) );
    }

    mobius_Poles.push_back(row);
  }

  // Fill array of Mobius knots just repeating OCCT knots as many times
  // as multiplicity value dictates.
  std::vector<double> mobius_UKnots, mobius_VKnots;
  //
  for ( int k = occt_UKnots.Lower(); k <= occt_UKnots.Upper(); ++k )
  {
    // Get multiplicity.
    const int mult = occt_UMults(k);

    // Fill knots.
    for ( int m = 0; m < mult; ++m )
      mobius_UKnots.push_back( occt_UKnots(k) );
  }
  //
  for ( int k = occt_VKnots.Lower(); k <= occt_VKnots.Upper(); ++k )
  {
    // Get multiplicity.
    const int mult = occt_VMults(k);

    // Fill knots.
    for ( int m = 0; m < mult; ++m )
      mobius_VKnots.push_back( occt_VKnots(k) );
  }

  // Construct Mobius surface.
  m_mobiusSurface = new bsurf(mobius_Poles, mobius_UKnots, mobius_VKnots, uDeg, vDeg);

  m_bIsDone = true;
}
