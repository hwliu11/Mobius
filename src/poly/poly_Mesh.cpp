//-----------------------------------------------------------------------------
// Created on: 17 September 2018
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
#include <mobius/poly_Mesh.h>

// Embedded OpenCascade includes
#include <mobius/Intf_InterferencePolygon2d.hxx>
#include <mobius/Intf_Polygon2d.hxx>
#include <mobius/ElSLib.hxx>
#include <mobius/gp_Ax3.hxx>

using namespace mobius;

// Instantiate for allowed traits.
#ifdef _WIN32
template class poly_Mesh<>;
#endif

namespace
{

//! The derived polygon class.
class SimplePolygon : public occ::Intf_Polygon2d
{
public:

  //! Ctor.
  SimplePolygon(const t_uv& pole0,
                const t_uv& pole1)
  {
    m_pole0 = occ::gp_Pnt2d( pole0.U(), pole0.V() );
    m_pole1 = occ::gp_Pnt2d( pole1.U(), pole1.V() );

    // One thing which is pretty inconvenient is the necessity to
    // update the AABB of a polygon manually. If you forget doing that,
    // the intersection check will return nothing.
    myBox.Add(m_pole0);
    myBox.Add(m_pole1);
  }

public:

  //! Returns the tolerance of the polygon.
  virtual double DeflectionOverEstimation() const
  {
    return occ::Precision::Confusion();
  }

  //! Returns the number of segments in the polyline.
  virtual int NbSegments() const
  {
    return 1;
  }

  //! Returns the points of the segment <index> in the Polygon.
  virtual void Segment(const int, occ::gp_Pnt2d& beg, occ::gp_Pnt2d& end) const
  {
    beg = m_pole0;
    end = m_pole1;
  }

protected:
 
  occ::gp_Pnt2d m_pole0, m_pole1;

};

} // Anonymous namespace

//-----------------------------------------------------------------------------

void poly_MeshUtils::ComputeCenter(const t_xyz& v0,
                                   const t_xyz& v1,
                                   const t_xyz& v2,
                                   t_xyz&       center)
{
  center = (v0 + v1 + v2) / 3;
}

//-----------------------------------------------------------------------------

bool poly_MeshUtils::ComputeNormal(const t_xyz& v0,
                                   const t_xyz& v1,
                                   const t_xyz& v2,
                                   t_xyz&       norm)
{
  // Compute norm.
  norm = (v1 - v0)^(v2 - v0);
  //
  if ( norm.Modulus() > core_Precision::Resolution3D() )
    norm.Normalize();

  return true;
}

//-----------------------------------------------------------------------------

double poly_MeshUtils::ComputeArea(const t_xyz& v0,
                                   const t_xyz& v1,
                                   const t_xyz& v2)
{
  // Compute area.
  const double area = 0.5 * ((v1 - v0) ^ (v2 - v0)).Modulus();
  return area;
}

//-----------------------------------------------------------------------------

bool poly_MeshUtils::ComputePyramidProps(const t_xyz&    v0,
                                         const t_xyz&    v1,
                                         const t_xyz&    v2,
                                         const core_XYZ& apex,
                                         double          gProps[10],
                                         const int       nbPnts,
                                         const double*   pnts)
{
  // Compute triangle area
  double area = ComputeArea(v0, v1, v2);
  //
  if ( area <= core_Precision::Resolution3D() )
    return false;

  // Define plane and coordinates of triangle nodes on plane
  t_xyz triNorm;
  ComputeNormal(v0, v1, v2, triNorm);

  t_xyz triCenter;
  ComputeCenter(v0, v1, v2, triCenter);

  occ::gp_Dir triNormDir(triNorm.X(), triNorm.Y(), triNorm.Z());
  occ::gp_Ax3 posPln(occ::gp_Pnt(triCenter.X(), triCenter.Y(), triCenter.Z()), triNormDir);

  // Coordinates of nodes on plane
  double x1, y1, x2, y2, x3, y3;
  //
  occ::ElSLib::PlaneParameters(posPln, occ::gp_Pnt( v0.X(), v0.Y(), v0.Z() ), x1, y1);
  occ::ElSLib::PlaneParameters(posPln, occ::gp_Pnt( v1.X(), v1.Y(), v1.Z() ), x2, y2);
  occ::ElSLib::PlaneParameters(posPln, occ::gp_Pnt( v2.X(), v2.Y(), v2.Z() ), x3, y3);
  //
  const double det = 2. * area;
  //
  double l1, l2; //barycentriche coordinates
  double x, y, z;
  double w; //weigh
  //
  for ( int i = 0; i < nbPnts; ++i )
  {
    int index = 3 * i;
    l1 = pnts[index];
    l2 = pnts[index + 1];
    w  = pnts[index + 2];
    w  *= det;
    x  = l1 * (x1 - x3) + l2 * (x2 - x3) + x3;
    y  = l1 * (y1 - y3) + l2 * (y2 - y3) + y3;
    occ::gp_Pnt aP = occ::ElSLib::PlaneValue(x, y, posPln);

    x = aP.X() - apex.X();
    y = aP.Y() - apex.Y();
    z = aP.Z() - apex.Z();
    //
    double xn = triNormDir.X() * w;
    double yn = triNormDir.Y() * w;
    double zn = triNormDir.Z() * w;
    double dv = x * xn + y * yn + z * zn;
    //
    gProps[0] += dv / 3.0;       // Volume
    //    
    gProps[1] += 0.25 * x * dv;  // Ix
    gProps[2] += 0.25 * y * dv;  // Iy
    gProps[3] += 0.25 * z * dv;  // Iz
    dv *= 0.2;
    gProps[7] -= x * y * dv;     // Ixy
    gProps[8] -= x * z * dv;     // Ixz
    gProps[9] -= y * z * dv;     // Iyz
    x *= x;
    y *= y;
    z *= z;
    gProps[4] += (y + z) * dv;   // Ixx
    gProps[5] += (x + z) * dv;   // Iyy
    gProps[6] += (x + y) * dv;   // Izz
  }

  return true;
}

//-----------------------------------------------------------------------------

template <typename ElemTraits>
bool poly_Mesh<ElemTraits>::AreSelfIntersecting(const int             tag,
                                                const poly_EdgeHandle eh0,
                                                const poly_EdgeHandle eh1) const
{
  if ( __surfAdt.IsNull() )
    return false; // The check is only possible with a CAD link.

  poly_Edge edges[2];
  if ( !this->GetEdge(eh0, edges[0]) ) return false;
  if ( !this->GetEdge(eh1, edges[1]) ) return false;

  t_xyz edge0Vertices[2], edge1Vertices[2];
  if ( !this->GetVertex(edges[0].hVertices[0], edge0Vertices[0]) ) return false;
  if ( !this->GetVertex(edges[0].hVertices[1], edge0Vertices[1]) ) return false;
  if ( !this->GetVertex(edges[1].hVertices[0], edge1Vertices[0]) ) return false;
  if ( !this->GetVertex(edges[1].hVertices[1], edge1Vertices[1]) ) return false;

  t_uv edge0UVs[2], edge1UVs[2];
  __surfAdt->InvertPoint(tag, edge0Vertices[0], edge0UVs[0]);
  __surfAdt->InvertPoint(tag, edge0Vertices[1], edge0UVs[1]);
  __surfAdt->InvertPoint(tag, edge1Vertices[0], edge1UVs[0]);
  __surfAdt->InvertPoint(tag, edge1Vertices[1], edge1UVs[1]);

  SimplePolygon poly0(edge0UVs[0], edge0UVs[1]);
  SimplePolygon poly1(edge1UVs[0], edge1UVs[1]);

  occ::Intf_InterferencePolygon2d algo(poly0, poly1);
  const int numPts = algo.NbSectionPoints();

  int numInters = 0;
  for ( int isol = 1; isol <= numPts; ++isol )
  {
    const double p[2] = { algo.PntValue(isol).ParamOnFirst(),
                          algo.PntValue(isol).ParamOnSecond() };

    if ( ( (p[0] > 0) && (p[0] < 1) ) || ( (p[1] > 0) && (p[1] < 1) ) )
      numInters++;
  }

  return numInters > 0;
}

//-----------------------------------------------------------------------------

template <typename ElemTraits>
bool poly_Mesh<ElemTraits>::AreSelfIntersecting(const int                           tag,
                                                const std::vector<poly_EdgeHandle>& ehs0,
                                                const std::vector<poly_EdgeHandle>& ehs1) const
{
  bool hasInters = false;
  for ( const auto he0 : ehs0 )
  {
    for ( const auto he1 : ehs1 )
    {
      if ( this->AreSelfIntersecting(tag, he0, he1) )
      {
        hasInters = true;
        break;
      }
    }

    if ( hasInters )
      break;
  }

  return hasInters;
}

//-----------------------------------------------------------------------------

template <typename ElemTraits>
bool poly_Mesh<ElemTraits>::AreSelfIntersecting(const std::unordered_set<int>& domain) const
{
  for ( auto tag : domain )
  {
    std::vector<poly_EdgeHandle> innerEhs, bndEhs;
    this->FindDomainEdges(tag, innerEhs, bndEhs);

    if ( this->AreSelfIntersecting(tag, innerEhs, bndEhs) )
      return true;
  }

  return false;
}
