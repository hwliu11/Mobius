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

bool mobius::poly_Mesh::ComputeNormal(const poly_VertexHandle hv0,
                                      const poly_VertexHandle hv1,
                                      const poly_VertexHandle hv2,
                                      t_xyz&                  norm) const
{
  t_xyz tv[3];
  //
  this->GetVertex(hv0, tv[0]);
  this->GetVertex(hv1, tv[1]);
  this->GetVertex(hv2, tv[2]);

  // Compute norm.
  norm = (tv[1] - tv[0])^(tv[2] - tv[0]);
  //
  if ( norm.Modulus() > core_Precision::Resolution3D() )
    norm.Normalize();

  return true;
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

  return this->ComputeNormal(htv[0], htv[1], htv[2], norm);
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

  // Get triangle links.
  /*poly_Edge e[3] = { poly_Edge(htv[0], htv[1]),
                     poly_Edge(htv[1], htv[2]),
                     poly_Edge(htv[2], htv[0]) };*/

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
  // Clean up any existing links.
  this->ClearEdges();

  // We keep the edge vertices sorted by their indices, so that we can
  // benefit from fast hashing. The idea is to hash twice using the
  // nested maps like
  //
  // <vertexHandle_0, <vertexHandle_1, edgeHandle_i>>
  // <vertexHandle_0, <vertexHandle_2, edgeHandle_j>>
  // ...
  typedef std::unordered_map<poly_VertexHandle, poly_EdgeHandle> t_vheh;
  std::unordered_map<poly_VertexHandle, t_vheh> visitedEdges;

  // Cache new links.
  for ( TriangleIterator tit(this); tit.More(); tit.Next() )
  {
    poly_TriangleHandle th = tit.Current();
    poly_Triangle       t;

    this->GetTriangle(th, t);
    //
    if ( t.IsDeleted() )
      continue;

    poly_VertexHandle vh[3];
    t.GetVertices(vh[0], vh[1], vh[2]);

    // Compose the edges to check for.
    poly_Edge edges[3];

    // Keep vertices sorted by index.
    edges[0] = (vh[0].iIdx < vh[1].iIdx ? poly_Edge(vh[0], vh[1]) : poly_Edge(vh[1], vh[0]));
    edges[1] = (vh[1].iIdx < vh[2].iIdx ? poly_Edge(vh[1], vh[2]) : poly_Edge(vh[2], vh[1]));
    edges[2] = (vh[2].iIdx < vh[0].iIdx ? poly_Edge(vh[2], vh[0]) : poly_Edge(vh[0], vh[2]));

    // Populate the map of links.
    for ( int eidx = 0; eidx < 3; ++eidx )
    {
      auto linkIt1 = visitedEdges.find(edges[eidx].hVertices[0]);
      //
      if ( linkIt1 == visitedEdges.end() )
      {
        poly_EdgeHandle eh( m_links.size() );

        t_vheh rec; rec.insert({edges[eidx].hVertices[1], eh});

        visitedEdges.insert( {edges[eidx].hVertices[0], rec});
        m_links     .insert    ( {eh, {th}} );
        m_edges     .push_back (edges[eidx]);
      }
      else
      {
        auto linkIt2 = linkIt1->second.find(edges[eidx].hVertices[1]);
        //
        if ( linkIt2 == linkIt1->second.end() )
        {
          poly_EdgeHandle eh( m_links.size() );

          linkIt1->second.insert({edges[eidx].hVertices[1], eh});

          m_links      .insert    ( {eh, {th}} );
          m_edges      .push_back (edges[eidx]);
        }
        else
        {
          m_links.find(linkIt2->second)->second.push_back(th);
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------

void mobius::poly_Mesh::ClearEdges()
{
  m_links.clear();
  m_edges.clear();
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::GetTriangles(const poly_EdgeHandle             he,
                                     std::vector<poly_TriangleHandle>& hts) const
{
  auto linkIt = m_links.find(he);
  if ( linkIt == m_links.end() )
    return false;

  hts = linkIt->second;
  return true;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::CanFlip(const poly_EdgeHandle he,
                                const double          normDevRad,
                                const double          planeDevRad,
                                poly_TriangleHandle&  ht0,
                                poly_TriangleHandle&  ht1,
                                poly_VertexHandle&    a,
                                poly_VertexHandle&    b,
                                poly_VertexHandle&    x,
                                poly_VertexHandle&    y,
                                t_xyz&                norm0,
                                t_xyz&                norm1) const
{
  a = b = x = y = poly_VertexHandle(Mobius_InvalidHandleIndex);

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

  this->ComputeNormal(ht0, norm0);
  this->ComputeNormal(ht1, norm1);
  //
  if ( norm0.Angle(norm1) > normDevRad )
    return false;

  /* Check angle criterion. */

  // Get vertices of the edge.
  poly_Edge e;
  this->GetEdge(he, e);
  //
  x = e.hVertices[0];
  y = e.hVertices[1];

  poly_VertexHandle t0_v[3], t1_v[3];
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

  // There's ambiguity how `x` and `y` are defined, so we
  // do all possible tests here.
  t_xyz bx = (b_coords - x_coords).Normalized();
  t_xyz ax = (a_coords - x_coords).Normalized();
  t_xyz xy = (y_coords - x_coords).Normalized();
  t_xyz by = (b_coords - y_coords).Normalized();
  t_xyz ay = (a_coords - y_coords).Normalized();
  t_xyz yx = (x_coords - y_coords).Normalized();

  const double adot1 = ax.Dot(xy);
  const double bdot1 = bx.Dot(xy);
  const double adot2 = ay.Dot(yx);
  const double bdot2 = by.Dot(yx);

  if ( adot1 < 0 || bdot1 < 0 || adot2 < 0 || bdot2 < 0 )
    return false;

  const double aang1 = std::acos(adot1);
  const double bang1 = std::acos(bdot1);
  const double aang2 = std::acos(adot2);
  const double bang2 = std::acos(bdot2);
  //
  if ( std::abs(aang1 - M_PI/2) < planeDevRad )
    return false;
  //
  if ( std::abs(bang1 - M_PI/2) < planeDevRad )
    return false;
  //
  if ( std::abs(aang2 - M_PI/2) < planeDevRad )
    return false;
  //
  if ( std::abs(bang2 - M_PI/2) < planeDevRad )
    return false;

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::CanFlip(const poly_EdgeHandle he,
                                const double          normDevRad,
                                const double          planeDevRad) const
{
  poly_TriangleHandle hts[2];
  poly_VertexHandle   a, b, x, y;
  t_xyz               norm0, norm1;
  //
  return this->CanFlip(he, normDevRad, planeDevRad,
                       hts[0], hts[1], a, b, x, y, norm0, norm1);
}

//-----------------------------------------------------------------------------

int mobius::poly_Mesh::FlipEdges(const double normDevRad,
                                 const double planeDevRad)
{
  int nbFlips = 0;

  for ( poly_Mesh::EdgeIterator eit(this); eit.More(); eit.Next() )
  {
    const poly_EdgeHandle eh = eit.Current();

    poly_VertexHandle   a, b, x, y;
    poly_TriangleHandle hts[2];
    t_xyz               norm0, norm1;
    //
    if ( !this->CanFlip(eh, normDevRad, planeDevRad,
                        hts[0], hts[1], a, b, x, y, norm0, norm1) )
      continue;

    // Get triangles to rotate.
    poly_Triangle ts[2];
    this->GetTriangle(hts[0], ts[0]);
    this->GetTriangle(hts[1], ts[1]);

    // Compute norms to preserve orientations.
    t_xyz testN[2];
    this->ComputeNormal(a, x, b, testN[0]);
    this->ComputeNormal(b, y, a, testN[1]);

    // Add new (rotated) triangles.
    if ( testN[0].Dot(norm0) > 0 )
    {
      this->AddTriangle( a, x, b, ts[0].GetFaceRef() );
    }
    else
    {
      this->AddTriangle( b, x, a, ts[0].GetFaceRef() );
    }
    //
    if ( testN[1].Dot(norm1) > 0 )
    {
      this->AddTriangle( b, y, a, ts[1].GetFaceRef() );
    }
    else
    {
      this->AddTriangle( a, y, b, ts[1].GetFaceRef() );
    }

    // Remove original triangles.
    this->RemoveTriangle(hts[0]);
    this->RemoveTriangle(hts[1]);
    nbFlips++;
  }

  // Invalidate all existing links.
  this->ClearEdges();

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
