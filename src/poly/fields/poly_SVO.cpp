//-----------------------------------------------------------------------------
// Created on: 10 December 2019
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
#include <mobius/poly_SVO.h>

// Standard includes
#include <algorithm>
#include <cfloat>

//-----------------------------------------------------------------------------

bool mobius::poly_SVO::IsValidCornerId(const size_t id)
{
  return id >= 0 && id <= 7;
}

//-----------------------------------------------------------------------------

size_t mobius::poly_SVO::GetCornerID(const size_t nx,
                                     const size_t ny,
                                     const size_t nz)
{
  return nx | (ny << 1) | (nz << 2);
}

//-----------------------------------------------------------------------------

void mobius::poly_SVO::GetCornerLocation(const size_t id,
                                         size_t&      nx,
                                         size_t&      ny,
                                         size_t&      nz)
{
  nx = (id >> 0) & 1;
  ny = (id >> 1) & 1;
  nz = (id >> 2) & 1;
}

//-----------------------------------------------------------------------------

mobius::poly_SVO::poly_SVO() : m_pChildren(nullptr)
{
  for ( size_t k = 0; k < 8; ++k )
    m_scalars[k] = DBL_MAX;
}

//-----------------------------------------------------------------------------

mobius::poly_SVO::poly_SVO(const t_xyz& cornerMin,
                           const t_xyz& cornerMax) : m_pChildren(nullptr),
                                                     m_cornerMin(cornerMin),
                                                     m_cornerMax(cornerMax)
{
  for ( size_t k = 0; k < 8; ++k )
    m_scalars[k] = DBL_MAX;
}

//-----------------------------------------------------------------------------

mobius::poly_SVO::~poly_SVO()
{
  this->Release();
}

//-----------------------------------------------------------------------------

void mobius::poly_SVO::Release()
{
  if ( m_pChildren != nullptr )
  {
    // First destruct the children.
    for ( size_t k = 0; k < 8; ++k )
      delete m_pChildren[k];

    delete[] m_pChildren;
  }

  m_pChildren = nullptr;
}

//-----------------------------------------------------------------------------

bool mobius::poly_SVO::HasScalars()
{
  bool allInf = true;
  for ( size_t k = 0; k < 8; ++k )
  {
    if ( m_scalars[k] != DBL_MAX )
    {
      allInf = false;
      break;
    }
  }

  return allInf;
}

//-----------------------------------------------------------------------------

void mobius::poly_SVO::SetScalar(const size_t id,
                                 const double s)
{
  if ( !IsValidCornerId(id) )
    return;

  m_scalars[id] = s;
}

//-----------------------------------------------------------------------------

double mobius::poly_SVO::GetScalar(const size_t id) const
{
  if ( !IsValidCornerId(id) )
    return DBL_MAX;

  return m_scalars[id];
}

//-----------------------------------------------------------------------------

bool mobius::poly_SVO::IsLeaf() const
{
  return (m_pChildren == nullptr);
}

//-----------------------------------------------------------------------------

bool mobius::poly_SVO::Split()
{
  if ( m_pChildren )
    return false; // Cannot split a node which has children.

  m_pChildren = new poly_SVO*[8];
  return true;
}

//-----------------------------------------------------------------------------

mobius::poly_SVO* mobius::poly_SVO::GetChild(const size_t id) const
{
  if ( !m_pChildren || !IsValidCornerId(id) )
    return nullptr;

  return m_pChildren[id];
}

//-----------------------------------------------------------------------------

mobius::poly_SVO*
  mobius::poly_SVO::FindChild(const std::vector<size_t>& path) const
{
  poly_SVO* current = const_cast<poly_SVO*>(this);

  for ( size_t k = 0; k < path.size(); ++k )
  {
    poly_SVO* next = current->GetChild(path[k]);
    //
    if ( !next )
      return nullptr;

    current = next;
  }

  return current;
}

//-----------------------------------------------------------------------------

double mobius::poly_SVO::Eval(const t_xyz& P) const
{
  // Find the nearest point on the box by coordinates snapping.
  const double PP[3] = { std::min( std::max( P.X(), m_cornerMin.X() ), m_cornerMax.X() ),
                         std::min( std::max( P.Y(), m_cornerMin.Y() ), m_cornerMax.Y() ),
                         std::min( std::max( P.Z(), m_cornerMin.Z() ), m_cornerMax.Z() ) };

  const poly_SVO* pNode = this;

  const double x = PP[0];
  const double y = PP[1];
  const double z = PP[2];

  for ( ;; )
  {
    if ( this->IsLeaf() )
    {
      // Get coordinates of the box corners.
      const double x0 = pNode->m_cornerMin.X();
      const double y0 = pNode->m_cornerMin.Y();
      const double z0 = pNode->m_cornerMin.Z();
      const double x1 = pNode->m_cornerMax.X();
      const double y1 = pNode->m_cornerMax.Y();
      const double z1 = pNode->m_cornerMax.Z();

      // Normalize for interpolation on unit length.
      const double xd = (x - x0)/(x1 - x0);
      const double yd = (y - y0)/(y1 - y0);
      const double zd = (z - z0)/(z1 - z0);

      // Get scalar values from corners.
      const double c000 = pNode->m_scalars[ GetCornerID(0, 0, 0) ];
      const double c001 = pNode->m_scalars[ GetCornerID(0, 0, 1) ];
      const double c010 = pNode->m_scalars[ GetCornerID(0, 1, 0) ];
      const double c011 = pNode->m_scalars[ GetCornerID(0, 1, 1) ];
      const double c100 = pNode->m_scalars[ GetCornerID(1, 0, 0) ];
      const double c101 = pNode->m_scalars[ GetCornerID(1, 0, 1) ];
      const double c110 = pNode->m_scalars[ GetCornerID(1, 1, 0) ];
      const double c111 = pNode->m_scalars[ GetCornerID(1, 1, 1) ];

      // Interpolate along OX.
      const double c00 = c000*(1 - xd) + c100*xd;
      const double c01 = c001*(1 - xd) + c101*xd;
      const double c10 = c010*(1 - xd) + c110*xd;
      const double c11 = c011*(1 - xd) + c111*xd;

      // Interpolate along OY.
      const double c0 = c00*(1 - yd) + c10*yd;
      const double c1 = c01*(1 - yd) + c11*yd;

      // Interpolate along OZ.
      const double c = c0*(1 - zd) + c1*zd;

      // Now we have the interpolated value to return.
      return c;
    }
    else
    {
      const double center[3] = { 0.5*( pNode->m_cornerMin.X() + pNode->m_cornerMax.X() ),
                                 0.5*( pNode->m_cornerMin.Y() + pNode->m_cornerMax.Y() ),
                                 0.5*( pNode->m_cornerMin.Z() + pNode->m_cornerMax.Z() ) };

      // Find ID of the octant containing the point of interest.
      for ( size_t dim = 0; dim < 3; ++dim )
      {
        size_t subID = 0;
        //
        if ( PP[dim] > center[dim] )
        {
          subID |= 1i64 << dim;
        }

        pNode = pNode->GetChild(subID);
      }
    }
  }
}

//-----------------------------------------------------------------------------

unsigned long long
  mobius::poly_SVO::GetMemoryInBytes(int& numNodes) const
{
  unsigned long long bytes = 0;
  if ( this->IsLeaf() )
  {
    bytes += sizeof(poly_SVO);
  }
  else
  {
    for ( size_t subID = 0; subID < 8; ++subID )
    {
      bytes += m_pChildren[subID]->GetMemoryInBytes(numNodes);
    }

    numNodes += 8;
  }

  return bytes;
}
