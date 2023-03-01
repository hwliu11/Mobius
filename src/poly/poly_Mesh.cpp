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
template class poly_Mesh<>;

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
