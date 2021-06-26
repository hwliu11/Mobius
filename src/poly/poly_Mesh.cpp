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

// Core includes
#include <mobius/core_Precision.h>

//-----------------------------------------------------------------------------

mobius::poly_Mesh::poly_Mesh() : core_OBJECT()
{}

//-----------------------------------------------------------------------------

void mobius::poly_Mesh::GetBounds(double& xMin, double& xMax,
                                  double& yMin, double& yMax,
                                  double& zMin, double& zMax) const
{
  double x_min = DBL_MAX, x_max = -DBL_MAX;
  double y_min = DBL_MAX, y_max = -DBL_MAX;
  double z_min = DBL_MAX, z_max = -DBL_MAX;

  for ( auto vit = m_vertices.cbegin(); vit != m_vertices.cend(); ++vit )
  {
    const poly_Vertex& V = *vit;
    const double x = V.X(),
                 y = V.Y(),
                 z = V.Z();

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

  // Set results.
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::RefineByMidpoint(const poly_TriangleHandle ht,
                                         poly_TriangleHandle&      t0,
                                         poly_TriangleHandle&      t1,
                                         poly_TriangleHandle&      t2)
{
  // Get the triangle to split.
  poly_Triangle t;
  if ( !this->GetTriangle(ht, t) )
    return false;

  // Get vertices on the triangle to split.
  poly_VertexHandle htv[3];
  t.GetVertices(htv[0], htv[1], htv[2]);
  //
  t_xyz midPt;
  poly_Vertex tv[3];
  for ( size_t k = 0; k < 3; ++k )
  {
    this->GetVertex(htv[k], tv[k]);

    midPt += tv[k];
  }
  //
  midPt /= 3.0;

  // Add midpoint vertex.
  poly_VertexHandle hmv = this->AddVertex(midPt);

  // Add new triangles.
  t0 = this->AddTriangle( htv[0], htv[1], hmv,    t.GetFaceRef() );
  t1 = this->AddTriangle( hmv,    htv[1], htv[2], t.GetFaceRef() );
  t2 = this->AddTriangle( htv[0], hmv,    htv[2], t.GetFaceRef() );

  // Remove the refined triangle.
  this->RemoveTriangle(ht);

  return true;
}
//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::RefineByMidpoint(const poly_TriangleHandle ht)
{
  poly_TriangleHandle hrt[3];
  return this->RefineByMidpoint(ht, hrt[0], hrt[1], hrt[2]);
}

//-----------------------------------------------------------------------------

bool
  mobius::poly_Mesh::ComputeNormal(const poly_TriangleHandle ht,
                                   t_xyz&                    norm) const
{
  // Get triangle by its handle.
  poly_Triangle t;
  if ( !this->GetTriangle(ht, t) )
    return false;

  // Get vertices on the triangle.
  poly_VertexHandle htv[3];
  t_xyz             tv[3];
  //
  t.GetVertices(htv[0], htv[1], htv[2]);
  //
  for ( size_t k = 0; k < 3; ++k )
    this->GetVertex(htv[k], tv[k]);

  // Compute norm.
  norm = (tv[1] - tv[0])^(tv[2] - tv[0]);
  //
  if ( norm.Modulus() > core_Precision::Resolution3D() )
    norm.Normalize();

  return true;
}

//-----------------------------------------------------------------------------

double
  mobius::poly_Mesh::ComputeArea(const poly_TriangleHandle ht) const
{
  // Get triangle by its handle.
  poly_Triangle t;
  if ( !this->GetTriangle(ht, t) )
    return false;

  // Get vertices on the triangle.
  poly_VertexHandle htv[3];
  t_xyz             tv[3];
  //
  t.GetVertices(htv[0], htv[1], htv[2]);
  //
  for ( size_t k = 0; k < 3; ++k )
    this->GetVertex(htv[k], tv[k]);

  // Compute area.
  const double area = 0.5*( (tv[1] - tv[0])^(tv[2] - tv[0]) ).Modulus();
  return area;
}

//-----------------------------------------------------------------------------

double
  mobius::poly_Mesh::ComputeMaxLen(const poly_TriangleHandle ht) const
{
  // Get triangle by its handle.
  poly_Triangle t;
  if ( !this->GetTriangle(ht, t) )
    return false;

  // Get vertices on the triangle.
  poly_VertexHandle htv[3];
  t_xyz             tv[3];
  //
  t.GetVertices(htv[0], htv[1], htv[2]);
  //
  for ( size_t k = 0; k < 3; ++k )
    this->GetVertex(htv[k], tv[k]);

  // Compute max length.
  const double maxLen = std::max( (tv[1] - tv[0]).Modulus(),
                                   std::max( (tv[2] - tv[1]).Modulus(),
                                             (tv[2] - tv[0]).Modulus() ) );
  return maxLen;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::Subdivide(const poly_TriangleHandle ht)
{
  // Get triangle by its handle.
  poly_Triangle t;
  if ( !this->GetTriangle(ht, t) )
    return false;

  // Get vertices on the triangle.
  poly_VertexHandle htv[3];
  t_xyz             tv[3];
  //
  t.GetVertices(htv[0], htv[1], htv[2]);
  //
  for ( size_t k = 0; k < 3; ++k )
    this->GetVertex(htv[k], tv[k]);

  // Prepare subdivision points.
  t_xyz mv[3] = { (tv[0] + tv[1])*0.5,
                  (tv[1] + tv[2])*0.5,
                  (tv[2] + tv[0])*0.5 };
  //
  poly_VertexHandle hmv[3] = { this->AddVertex(mv[0]),
                               this->AddVertex(mv[1]),
                               this->AddVertex(mv[2]) };

  // Add new triangles.
  poly_TriangleHandle t0 = this->AddTriangle( htv[0], hmv[0], hmv[2], t.GetFaceRef() );
  poly_TriangleHandle t1 = this->AddTriangle( hmv[0], htv[1], hmv[1], t.GetFaceRef() );
  poly_TriangleHandle t2 = this->AddTriangle( hmv[1], htv[2], hmv[2], t.GetFaceRef() );
  poly_TriangleHandle t3 = this->AddTriangle( hmv[0], hmv[1], hmv[2], t.GetFaceRef() );

  // Remove the subdivided triangle.
  this->RemoveTriangle(ht);

  return true;
}

//-----------------------------------------------------------------------------

void mobius::poly_Mesh::ComputeEdges()
{
  for ( TriangleIterator tit(this); tit.More(); tit.Next() )
  {
    poly_TriangleHandle th = tit.Current();
    poly_Triangle       t;

    this->GetTriangle(th, t);

    poly_VertexHandle vh[3];
    t.GetVertices(vh[0], vh[1], vh[2]);

    // Compose the edges to check for.
    poly_Edge edges[3] = { poly_Edge(vh[0], vh[1]),
                           poly_Edge(vh[1], vh[2]),
                           poly_Edge(vh[2], vh[0]) };

    // Populate the map of links.
    for ( int eidx = 0; eidx < 3; ++eidx )
    {
      auto linkIt = m_links.find(edges[eidx]);
      //
      if ( linkIt == m_links.end() )
      {
        m_links.insert( {edges[eidx], {th}} );
        m_edges.push_back(edges[eidx]);
      }
      else
      {
        linkIt->second.push_back(th);
      }
    }
  }
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::GetTriangles(const poly_EdgeHandle             he,
                                     std::vector<poly_TriangleHandle>& hts) const
{
  poly_Edge e;
  if ( !this->GetEdge(he, e) )
    return false;

  auto linkIt = m_links.find(e);
  if ( linkIt == m_links.end() )
    return false;

  hts = linkIt->second;
  return true;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::CanFlip(const poly_EdgeHandle he,
                                const double          normDevRad,
                                poly_TriangleHandle&  ht0,
                                poly_TriangleHandle&  ht1) const
{
  std::vector<poly_TriangleHandle> hts;
  if ( !this->GetTriangles(he, hts) )
    return false;

  if ( hts.size() != 2 )
    return false;

  ht0 = hts[0];
  ht1 = hts[1];

  poly_Triangle t[2];
  this->GetTriangle(ht0, t[0]);
  this->GetTriangle(ht1, t[1]);
  //
  if ( t[0].IsDeleted() || t[1].IsDeleted() )
    return false;

  /* Check normal criterion. */

  t_xyz norm[2];
  this->ComputeNormal(ht0, norm[0]);
  this->ComputeNormal(ht1, norm[1]);
  //
  if ( norm[0].Angle(norm[1]) > normDevRad )
    return false;

  /* Check angle criterion. */

  // Get vertices of the edge.
  poly_Edge e;
  this->GetEdge(he, e);
  //
  poly_VertexHandle x, y;
  e.GetVertices(x, y);

  poly_VertexHandle t0_v[3], t1_v[3];
  poly_VertexHandle a(Mobius_InvalidHandleIndex);
  poly_VertexHandle b(Mobius_InvalidHandleIndex);
  t[0].GetVertices(t0_v[0], t0_v[1], t0_v[2]);
  t[1].GetVertices(t1_v[0], t1_v[1], t1_v[2]);

  // Get opposite vertices `a` and `b`.
  for ( int j = 0; j < 3; ++j )
  {
    if ( t0_v[j] == x || t0_v[j] == y )
      continue;

    a = t0_v[j];
    break;
  }
  //
  for ( int j = 0; j < 3; ++j )
  {
    if ( t1_v[j] == x || t1_v[j] == y )
      continue;

    b = t1_v[j];
    break;
  }
  //
  if ( a.GetIdx() == Mobius_InvalidHandleIndex )
    return false;
  //
  if ( b.GetIdx() == Mobius_InvalidHandleIndex )
    return false;

  t_xyz a_coords, b_coords, x_coords, y_coords;
  this->GetVertex(a, a_coords);
  this->GetVertex(b, b_coords);
  this->GetVertex(x, x_coords);
  this->GetVertex(y, y_coords);

  t_xyz bx = b_coords - x_coords;
  t_xyz ax = a_coords - x_coords;
  t_xyz xy = y_coords - x_coords;

  if ( ax.Dot(xy) < 0 || bx.Dot(xy) < 0 )
    return false;

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::CanFlip(const poly_EdgeHandle he,
                                const double          normDevRad) const
{
  poly_TriangleHandle hts[2];
  return this->CanFlip(he, normDevRad, hts[0], hts[1]);
}

//-----------------------------------------------------------------------------

int mobius::poly_Mesh::FlipEdges(const double normDevRad)
{
  int nbFlips = 0;

  for ( poly_Mesh::EdgeIterator eit(this); eit.More(); eit.Next() )
  {
    const poly_EdgeHandle eh = eit.Current();

    poly_TriangleHandle hts[2];
    if ( !this->CanFlip(eh, normDevRad, hts[0], hts[1]) )
      continue;

    // Get edge to flip.
    poly_Edge e;
    this->GetEdge(eh, e);

    // Get vertices of the edge.
    poly_VertexHandle x, y;
    e.GetVertices(x, y);

    // Get triangle to rotate.
    poly_Triangle ts[2];
    this->GetTriangle(hts[0], ts[0]);
    this->GetTriangle(hts[1], ts[1]);

    // Get vertices of the triangles.
    poly_VertexHandle ts0_v[3], ts1_v[3];
    poly_VertexHandle a(Mobius_InvalidHandleIndex);
    poly_VertexHandle b(Mobius_InvalidHandleIndex);
    ts[0].GetVertices(ts0_v[0], ts0_v[1], ts0_v[2]);
    ts[1].GetVertices(ts1_v[0], ts1_v[1], ts1_v[2]);

    // Get opposite vertices `a` and `b`.
    for ( int j = 0; j < 3; ++j )
    {
      if ( ts0_v[j] == x || ts0_v[j] == y )
        continue;

      a = ts0_v[j];
      break;
    }
    //
    for ( int j = 0; j < 3; ++j )
    {
      if ( ts1_v[j] == x || ts1_v[j] == y )
        continue;

      b = ts1_v[j];
      break;
    }
    //
    if ( a.GetIdx() == Mobius_InvalidHandleIndex )
      continue;
    //
    if ( b.GetIdx() == Mobius_InvalidHandleIndex )
      continue;

    this->AddTriangle( a, x, b, ts[0].GetFaceRef() );
    this->AddTriangle( b, y, a, ts[1].GetFaceRef() );

    // Remove triangles.
    this->RemoveTriangle(hts[0]);
    this->RemoveTriangle(hts[1]);
    nbFlips++;
  }

  return nbFlips;
}

//-----------------------------------------------------------------------------

mobius::poly_EdgeHandle mobius::poly_Mesh::FindEdge(const poly_Edge& e) const
{
  for ( size_t eidx = 0; eidx < m_edges.size(); ++eidx )
  {
    if ( m_edges[eidx] == e )
      return poly_EdgeHandle( int(eidx) );
  }

  return poly_EdgeHandle(Mobius_InvalidHandleIndex);
}
