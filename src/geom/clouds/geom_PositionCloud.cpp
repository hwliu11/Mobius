//-----------------------------------------------------------------------------
// Created on: 15 November 2013
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
#include <mobius/geom_PositionCloud.h>

// Standard includes
#include <fstream>

//-----------------------------------------------------------------------------

mobius::geom_PositionCloud::geom_PositionCloud() : geom_PointCloud()
{}

//-----------------------------------------------------------------------------

mobius::geom_PositionCloud::geom_PositionCloud(const std::vector<t_xyz>& pts) : geom_PointCloud()
{
  this->SetPoints(pts);
}

//-----------------------------------------------------------------------------

mobius::geom_PositionCloud::geom_PositionCloud(const std::vector<double>& coords)
{
  for ( size_t k = 0; k < coords.size() - 2; k += 3 )
    m_cloud.push_back( t_xyz(coords[k], coords[k+1], coords[k+2]) );
}

//-----------------------------------------------------------------------------

mobius::geom_PositionCloud::~geom_PositionCloud()
{}

//-----------------------------------------------------------------------------

void mobius::geom_PositionCloud::GetBounds(double& xMin, double& xMax,
                                           double& yMin, double& yMax,
                                           double& zMin, double& zMax) const
{
  double x_min = DBL_MAX, x_max = -DBL_MAX;
  double y_min = DBL_MAX, y_max = -DBL_MAX;
  double z_min = DBL_MAX, z_max = -DBL_MAX;

  // Iterate over the points to calculate bounds
  for ( int p = 0; p < (int) m_cloud.size(); ++p )
  {
    const t_xyz& P = m_cloud.at(p);
    const double x = P.X(),
                 y = P.Y(),
                 z = P.Z();

    if ( x > x_max )
      x_max = x;
    if ( x < x_min )
      x_min = x;
    if ( y > y_max )
      y_max = y;
    if ( y < y_min )
      y_min = y;
    if ( z > z_max )
      z_max = z;
    if ( z < z_min )
      z_min = z;
  }

  // Set results
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}

//-----------------------------------------------------------------------------

void mobius::geom_PositionCloud::AddPoint(const t_xyz& point)
{
  m_cloud.push_back(point);
}

//-----------------------------------------------------------------------------

void mobius::geom_PositionCloud::AddPoint(const double x,
                                          const double y,
                                          const double z)
{
  this->AddPoint( t_xyz(x, y, z) );
}

//-----------------------------------------------------------------------------

int mobius::geom_PositionCloud::GetNumberOfPoints() const
{
  return int( m_cloud.size() );
}

//-----------------------------------------------------------------------------

const mobius::t_xyz&
  mobius::geom_PositionCloud::GetPoint(const int idx) const
{
  return m_cloud.at(idx);
}

//-----------------------------------------------------------------------------

void mobius::geom_PositionCloud::GetPoint(const int idx,
                                          double&   x,
                                          double&   y,
                                          double&   z) const
{
  const t_xyz& pt = this->GetPoint(idx);
  //
  x = pt.X();
  y = pt.Y();
  z = pt.Z();
}

//-----------------------------------------------------------------------------

void mobius::geom_PositionCloud::SetPoints(const std::vector<t_xyz>& cloud)
{
  m_cloud = cloud;
}

//-----------------------------------------------------------------------------

const std::vector<mobius::t_xyz>&
  mobius::geom_PositionCloud::GetPoints() const
{
  return m_cloud;
}

//-----------------------------------------------------------------------------

void mobius::geom_PositionCloud::Clear()
{
  m_cloud.clear();
}

//-----------------------------------------------------------------------------

bool mobius::geom_PositionCloud::Load(const std::string& filename)
{
  std::ifstream FILE(filename);
  if ( !FILE.is_open() )
    return false;

  while ( !FILE.eof() )
  {
    char str[256];
    FILE.getline(str, 256);

    std::vector<std::string> tokens;
    std::istringstream iss(str);
    std::copy( std::istream_iterator<std::string>(iss),
               std::istream_iterator<std::string>(),
               std::back_inserter< std::vector<std::string> >(tokens) );

    if ( tokens.empty() || tokens.size() < 3 )
      continue;

    double x = atof( tokens[0].c_str() ),
           y = atof( tokens[1].c_str() ),
           z = atof( tokens[2].c_str() );

    this->AddPoint(x, y, z);
  }

  FILE.close();
  return true;
}

//-----------------------------------------------------------------------------

bool mobius::geom_PositionCloud::SaveAs(const std::string& filename) const
{
  std::ofstream FILE(filename);
  if ( !FILE.is_open() )
    return false;

  for ( int i = 0; i < this->GetNumberOfPoints(); ++i )
  {
    double x, y, z;
    this->GetPoint(i, x, y, z);

    FILE << x << " " << y << " " << z << "\n";
  }

  FILE.close();
  return true;
}
