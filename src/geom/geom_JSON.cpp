//-----------------------------------------------------------------------------
// Created on: 18 June 2018
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
#include <mobius/geom_JSON.h>

//-----------------------------------------------------------------------------

//! Default ctor.
mobius::geom_JSON::geom_JSON() : core_JSON()
{}

//-----------------------------------------------------------------------------

//! Constructor accepting JSON string to process.
//! \param[in] json string representing JSON to process.
mobius::geom_JSON::geom_JSON(const std::string& json)
: core_JSON(json)
{}

//-----------------------------------------------------------------------------

//! Destructor.
mobius::geom_JSON::~geom_JSON()
{}

//-----------------------------------------------------------------------------

void mobius::geom_JSON::DumpBCurve(const t_ptr<t_bcurve>& curve)
{
  std::stringstream out;

  // Get parametric bounds.
  const double uMin = curve->GetMinParameter();
  const double uMax = curve->GetMaxParameter();

  out << std::setprecision( std::numeric_limits<double>::max_digits10 );
  out << "{";
  out <<  "\n    entity: curve";
  out << ",\n    type: b-curve";
  out << ",\n    continuity: " << this->ContinuityToString( curve->GetContinuity() );

  out << ",\n    domain: {"; // Begin 'domain'.
  out <<  "\n        U_min: " << uMin;
  out << ",\n        U_max: " << uMax;
  out <<  "\n    }"; // End 'domain'.

  out << ",\n    flags: {"; // Begin 'flags'.
  out <<  "\n        is_rational: 0"; // TODO: rational curves are not supported.
  out << ",\n        is_periodic: 0"; // TODO: periodic curves are not supported.
  out << ",\n        is_closed: 0";   // TODO: closeness property is not available in Mobius.
  out <<  "\n    }"; // End 'flags'.

  out << ",\n    properties: {"; // Begin 'properties'.
  out <<  "\n        degree: " << curve->GetDegree();

  // Dump knots.
  const std::vector<double>& knots = curve->GetKnots();
  out << ",\n        knots: [";
  //
  for ( int k = 0; k < int( knots.size() ); ++k )
  {
    out << knots[k];

    if ( k < int( knots.size() ) - 1 )
      out << ", ";
  }
  out << "]";

  // Dump poles.
  const std::vector<t_xyz>& poles = curve->GetPoles();
  //
  out << ",\n        num_poles: " << curve->GetNumOfPoles();
  out << ",\n        poles: [";
  //
  for ( int uIdx = 0; uIdx < int( poles.size() ); ++uIdx )
  {
    const t_xyz& P = poles[uIdx];

    out << "[" << P.X() << ", " << P.Y() << ", " << P.Z() << "]";

    if ( uIdx < int( poles.size() ) - 1 )
      out << ", ";
  }
  out << "]";
  out << "\n    }"; // End 'properties'.

  out << "\n}";
 
  // Initialize json.
  m_json = out.str();
}

//-----------------------------------------------------------------------------

void mobius::geom_JSON::DumpBSurface(const t_ptr<t_bsurf>& surface)
{
  std::stringstream out;

  // Get parametric bounds.
  const double uMin = surface->GetMinParameter_U();
  const double uMax = surface->GetMaxParameter_U();
  const double vMin = surface->GetMinParameter_V();
  const double vMax = surface->GetMaxParameter_V();

  out << std::setprecision( std::numeric_limits<double>::max_digits10 );
  out << "{";
  out <<  "\n    entity: surface";
  out << ",\n    type: b-surface";
  out << ",\n    continuity: " << this->ContinuityToString( surface->GetContinuity() );

  out << ",\n    domain: {"; // Begin 'domain'.
  out <<  "\n        U_min: " << uMin;
  out << ",\n        U_max: " << uMax;
  out << ",\n        V_min: " << vMin;
  out << ",\n        V_max: " << vMax;
  out <<  "\n    }"; // End 'domain'.

  out << ",\n    flags: {"; // Begin 'flags'.
  out <<  "\n        is_U_rational: 0"; // TODO: not currently supported.
  out << ",\n        is_V_rational: 0"; // TODO: not currently supported.
  out << ",\n        is_U_periodic: 0"; // TODO: not currently supported.
  out << ",\n        is_V_periodic: 0"; // TODO: not currently supported.
  out << ",\n        is_U_closed: 0"; // TODO: not currently supported.
  out << ",\n        is_V_closed: 0"; // TODO: not currently supported.
  out <<  "\n    }"; // End 'flags'.

  out << ",\n    properties: {"; // Begin 'properties'.
  out <<  "\n        U_degree: " << surface->GetDegree_U();
  out << ",\n        V_degree: " << surface->GetDegree_V();

  // Dump U knots.
  const std::vector<double>& U_knots = surface->GetKnots_U();
  out << ",\n        U_knots: [";
  //
  for ( int k = 0; k < int( U_knots.size() ); ++k )
  {
    out << U_knots[k];

    if ( k < int( U_knots.size() ) - 1 )
      out << ", ";
  }
  out << "]";

  // Dump V knots.
  const std::vector<double>& V_knots = surface->GetKnots_V();
  out << ",\n        V_knots: [";
  //
  for ( int k = 0; k < int( V_knots.size() ); ++k )
  {
    out << V_knots[k];

    if ( k < int( V_knots.size() ) - 1 )
      out << ", ";
  }
  out << "]";

  // Dump poles.
  const std::vector< std::vector<t_xyz> >& poles = surface->GetPoles();
  //
  out << ",\n        num_poles_in_U_axis: " << surface->GetNumOfPoles_U();
  out << ",\n        num_poles_in_V_axis: " << surface->GetNumOfPoles_V();
  out << ",\n        poles: {"; // Begin 'poles'.
  //
  for ( int uIdx = 0; uIdx < int( poles.size() ); ++uIdx )
  {
    out << "\n            u" << uIdx << ": [";
    for ( int vIdx = 0; vIdx < int( poles[0].size() ); ++vIdx )
    {
      const t_xyz& P = poles[uIdx][vIdx];

      out << "[" << P.X() << ", " << P.Y() << ", " << P.Z() << "]";

      if ( vIdx < int( poles[0].size() ) - 1 )
        out << ", ";
    }
    out << "]";
    if ( uIdx < int( poles.size() ) - 1 )
      out << ",";
  }
  out << "\n        }"; // End 'poles'.
  out << "\n    }"; // End 'properties'.

  out << "\n}";

  // Initialize json.
  m_json = out.str();
}

//-----------------------------------------------------------------------------

bool mobius::geom_JSON::ExtractBCurve(t_ptr<t_bcurve>& curve) const
{
  // Extract degree.
  int p = 0;
  if ( !this->ExtractNumericBlockForKey<int>("degree", p) )
    return false;

  // Extract knot vector.
  std::vector<double> U;
  if ( !this->ExtractVector1d("knots", U) )
    return false;

  // Extract poles.
  std::vector<t_xyz> poles;
  if ( !this->ExtractVector3d("poles", poles) )
    return false;

  // Construct B-curve.
  curve = new t_bcurve(poles, U, p);
  return true;
}

//-----------------------------------------------------------------------------

bool mobius::geom_JSON::ExtractBSurface(t_ptr<t_bsurf>& surface) const
{
  // Extract degrees.
  int p = 0, q = 0;
  if ( !this->ExtractNumericBlockForKey<int>("U_degree", p) )
    return false;
  if ( !this->ExtractNumericBlockForKey<int>("V_degree", q) )
    return false;

  // Extract knot vectors.
  std::vector<double> U, V;
  if ( !this->ExtractVector1d("U_knots", U) )
    return false;
  if ( !this->ExtractVector1d("V_knots", V) )
    return false;

  // Extract poles.
  std::vector< std::vector<t_xyz> > poles;
  if ( !this->ExtractGrid3d("poles", poles) )
    return false;

  // Costruct B-surface.
  surface = new t_bsurf(poles, U, V, p, q);
  return true;
}
