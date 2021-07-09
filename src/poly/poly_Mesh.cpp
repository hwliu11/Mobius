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

// Poly includes
#include <mobius/poly_Jacobian.h>

// Core includes
#include <mobius/core_Precision.h>

//-----------------------------------------------------------------------------

mobius::poly_Mesh::poly_Mesh() : core_OBJECT()
{}

//-----------------------------------------------------------------------------

mobius::t_ptr<mobius::poly_Mesh> mobius::poly_Mesh::DeepCopy() const
{
  t_ptr<poly_Mesh> copy = new poly_Mesh;
  //
  copy->m_vertices  = this->m_vertices;
  copy->m_edges     = this->m_edges;
  copy->m_triangles = this->m_triangles;
  copy->m_quads     = this->m_quads;
  copy->m_links     = this->m_links;

  return copy;
}

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
  if ( !this->GetTriangle(ht, t) || t.IsDeleted() )
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

bool mobius::poly_Mesh::RefineByMidedges(const poly_TriangleHandle         ht,
                                         std::vector<poly_TriangleHandle>& hts)
{
  // Get the triangle to refine.
  poly_Triangle t;
  if ( !this->GetTriangle(ht, t) )
    return false;
  //
  if ( t.IsDeleted() )
    return false;

  // Get vertices on the triangle.
  poly_VertexHandle hv[3];
  t.GetVertices(hv[0], hv[1], hv[2]);

  // Get the edges.
  poly_Edge edges[3] = { poly_Edge(hv[0], hv[1]),
                         poly_Edge(hv[1], hv[2]),
                         poly_Edge(hv[2], hv[0]) };
  //
  poly_EdgeHandle hes[3] = { this->FindEdge(edges[0]),
                             this->FindEdge(edges[1]),
                             this->FindEdge(edges[2]) };
  //
  for ( int j = 0; j < 3; ++j )
  {
    if ( hes[j].iIdx == Mobius_InvalidHandleIndex )
      return false;
  }

  // Get corners.
  t_xyz v[3];
  this->GetVertex(hv[0], v[0]);
  this->GetVertex(hv[1], v[1]);
  this->GetVertex(hv[2], v[2]);

  // Build new vertices.
  poly_VertexHandle hmv[3];
  t_xyz             mv[3];
  //
  hmv[0] = this->AddVertex( 0.5*(v[0] + v[1]) );
  hmv[1] = this->AddVertex( 0.5*(v[1] + v[2]) );
  hmv[2] = this->AddVertex( 0.5*(v[2] + v[0]) );

  std::unordered_map<poly_EdgeHandle, poly_VertexHandle> eSplits;
  eSplits.insert({hes[0], hmv[0]});
  eSplits.insert({hes[1], hmv[1]});
  eSplits.insert({hes[2], hmv[2]});

  // Build new triangles.
  hts.push_back( this->AddTriangle( hv [0], hmv[0], hmv[2], t.GetFaceRef() ) );
  hts.push_back( this->AddTriangle( hmv[0], hv [1], hmv[1], t.GetFaceRef() ) );
  hts.push_back( this->AddTriangle( hmv[1], hv [2], hmv[2], t.GetFaceRef() ) );
  hts.push_back( this->AddTriangle( hmv[0], hmv[1], hmv[2], t.GetFaceRef() ) );

  // Add 9 newly created edges.
  std::unordered_map<poly_Edge, std::pair<poly_TriangleHandle, poly_TriangleHandle>> newEdges;
  //
  newEdges.insert( { poly_Edge(hv [0], hmv[0]), { hts[0], poly_TriangleHandle() } } );
  newEdges.insert( { poly_Edge(hmv[0], hmv[2]), { hts[0], hts[3]                } } );
  newEdges.insert( { poly_Edge(hmv[2], hv [0]), { hts[0], poly_TriangleHandle() } } );
  //
  newEdges.insert( { poly_Edge(hmv[0], hv [1]), { hts[1], poly_TriangleHandle() } } );
  newEdges.insert( { poly_Edge(hv [1], hmv[1]), { hts[1], poly_TriangleHandle() } } );
  newEdges.insert( { poly_Edge(hmv[1], hmv[0]), { hts[1], hts[3]                } } );
  //
  newEdges.insert( { poly_Edge(hmv[1], hv [2]), { hts[2], poly_TriangleHandle() } } );
  newEdges.insert( { poly_Edge(hv [2], hmv[2]), { hts[2], poly_TriangleHandle() } } );
  newEdges.insert( { poly_Edge(hmv[2], hmv[1]), { hts[2], hts[3]                } } );

  std::unordered_map<poly_TriangleHandle, poly_EdgeHandle> tris2Split;
  for ( int j = 0; j < 3; ++j )
  {
    std::vector<poly_TriangleHandle> edgeTris;

    if ( !this->GetTriangles(hes[j], edgeTris) )
      return false;

    for ( const auto& eth : edgeTris )
      if ( eth != ht )
        tris2Split.insert({eth, hes[j]});
  }

  // Split neighbor triangles.
  for ( const auto& toSplit : tris2Split )
  {
    // Get common edge.
    poly_Edge cmnEdge;
    if ( !this->GetEdge(toSplit.second, cmnEdge) )
      continue;

    if ( cmnEdge.IsDeleted() )
      continue;

    // Get triangle to split.
    poly_Triangle nextTri;
    if ( !this->GetTriangle(toSplit.first, nextTri) )
      continue;

    // Compute the reference normal to control the splitting validity.
    t_xyz refNorm;
    this->ComputeNormal(toSplit.first, refNorm);

    /* Find the opposite node. */

    poly_VertexHandle
      oppVh = this->GetOppositeVertex(toSplit.first, toSplit.second);
    //
    if ( !oppVh.IsValid() )
      continue; // Skip triangle if we are not able to find its opposite node.

    /* Split triangle */

    poly_VertexHandle a = oppVh;
    poly_VertexHandle b = cmnEdge.hVertices[0];
    poly_VertexHandle c = eSplits[toSplit.second];
    poly_VertexHandle d = cmnEdge.hVertices[1];

    // Compute norms to preserve orientations.
    t_xyz testN[2];
    this->ComputeNormal(a, b, c, testN[0]);
    this->ComputeNormal(a, c, d, testN[1]);

    // Add new triangles.
    poly_TriangleHandle thLeft, thRight;
    //
    if ( testN[0].Dot(refNorm) > 0 )
    {
      thLeft = this->AddTriangle( a, b, c, nextTri.GetFaceRef() );
    }
    else
    {
      thLeft = this->AddTriangle( a, c, b, nextTri.GetFaceRef() );
    }
    //
    if ( testN[1].Dot(refNorm) > 0 )
    {
      thRight = this->AddTriangle( a, c, d, nextTri.GetFaceRef() );
    }
    else
    {
      thRight = this->AddTriangle( a, d, c, nextTri.GetFaceRef() );
    }

    // Update edges coming out of split.
    newEdges[poly_Edge(b, c)].second = thLeft;
    newEdges[poly_Edge(c, d)].second = thRight;
    //
    newEdges.insert( { poly_Edge(a, c), { thLeft, thRight } } );

    // Update edges sharing the opposite vertex.
    this->updateLink( this->FindEdge( poly_Edge(a, b) ), toSplit.first, thLeft);
    this->updateLink( this->FindEdge( poly_Edge(a, d) ), toSplit.first, thRight);

    /* Remove old triangle. */
    this->RemoveTriangle(toSplit.first);
  }

  // Remove old triangle.
  this->RemoveTriangle(ht);

  // Erase old edges.
  m_links.erase(hes[0]);
  m_links.erase(hes[1]);
  m_links.erase(hes[2]);
  //
  this->RemoveEdge(hes[0]);
  this->RemoveEdge(hes[1]);
  this->RemoveEdge(hes[2]);
  //
  for ( const auto& edge2Insert : newEdges )
  {
    const poly_EdgeHandle newEh = this->AddEdge(edge2Insert.first);

    std::vector<poly_TriangleHandle> newTris;
    //
    if ( edge2Insert.second.first.IsValid() )
      newTris.push_back(edge2Insert.second.first);
    //
    if ( edge2Insert.second.second.IsValid() )
      newTris.push_back(edge2Insert.second.second);

    m_links.insert({newEh, newTris});
  }

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::RefineByMidedges(const poly_TriangleHandle ht)
{
  std::vector<poly_TriangleHandle> hts;
  return this->RefineByMidedges(ht, hts);
}

//-----------------------------------------------------------------------------

mobius::poly_VertexHandle
  mobius::poly_Mesh::GetOppositeVertex(const poly_TriangleHandle ht,
                                       const poly_EdgeHandle     he) const
{
  poly_Edge edge;
  if ( !this->GetEdge(he, edge) )
    return poly_VertexHandle();

  poly_Triangle nbrTri;
  if ( !this->GetTriangle(ht, nbrTri) )
    return poly_VertexHandle();

  poly_VertexHandle vhs[3];
  nbrTri.GetVertices(vhs[0], vhs[1], vhs[2]);

  poly_VertexHandle oppVh;
  for ( int vidx = 0; vidx < 3; ++vidx )
    if ( (vhs[vidx] != edge.hVertices[0]) && (vhs[vidx] != edge.hVertices[1]) )
      oppVh = vhs[vidx];

  if ( !oppVh.IsValid() )
    return poly_VertexHandle();

  return oppVh;
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

double mobius::poly_Mesh::ComputeScaledJacobian(const poly_TriangleHandle ht) const
{
  poly_Jacobian calc(this);

  double res = DBL_MAX;
  for ( int k = 0; k < 3; ++k )
  {
    t_uv   uv[3];
    double J[2][2] = { {0, 0}, {0, 0} };
    double J_det   = 0.;
    double J_det_n = 0.;
    //
    calc.Compute(ht, k, uv[0], uv[1], uv[2], J, J_det, J_det_n);

    res = std::min(res, J_det_n);
  }

  return res;
}

//-----------------------------------------------------------------------------

double
  mobius::poly_Mesh::ComputeScaledJacobian(const t_xyz& v0,
                                           const t_xyz& v1,
                                           const t_xyz& v2) const
{
  double res = DBL_MAX;
  for ( int k = 0; k < 3; ++k )
  {
    t_uv   uv[3];
    double J[2][2] = { {0, 0}, {0, 0} };
    double J_det   = 0.;
    double J_det_n = 0.;
    //
    poly_Jacobian::Compute(v0, v1, v2, k, uv[0], uv[1], uv[2], J, J_det, J_det_n);

    res = std::min(res, J_det_n);
  }

  return res;
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

bool mobius::poly_Mesh::IsDegenerated(const t_xyz& v0,
                                      const t_xyz& v1,
                                      const t_xyz& v2,
                                      const double prec) const
{
  t_xyz tv[3] = {v0, v1, v2};

  // Degenerated side.
  t_xyz vec01 = tv[1] - tv[0];
  const double sqNorm01 = vec01.SquaredModulus();
  if ( sqNorm01 <= prec )
  {
    return true;
  }

  // Degenerated side.
  t_xyz vec02 = tv[2] - tv[0];
  const double sqNorm02 = vec02.SquaredModulus();
  if ( sqNorm02 <= prec )
  {
    return true;
  }

  // Degenerated side.
  t_xyz vec12 = tv[2] - tv[1];
  const double sqNorm12 = vec12.SquaredModulus();
  if ( sqNorm12 <= prec )
  {
    return true;
  }

  /*const double a1 = vec01.Angle(vec02);
  const double a2 = vec12.Angle(vec01);
  const double a3 = vec02.Angle(vec12);*/

  const double SP0102 = vec01 * vec02;
  t_xyz vec = vec02 - (SP0102 / sqNorm01) * vec01;
  if ( vec.SquaredModulus() <= prec )
  {
    return true;
  }

  vec = vec01 - (SP0102 / sqNorm02) * vec02;
  if ( vec.SquaredModulus() <= prec )
  {
    return true;
  }

  vec = ((vec01 * vec12) / sqNorm12) * vec12 - vec01;

  return (vec.SquaredModulus() <= prec);
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::IsDegenerated(const poly_TriangleHandle ht,
                                      const double              prec) const
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

  return this->IsDegenerated(tv[0], tv[1], tv[2], prec);
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

        // Add edge and keep a link in a triangle.
        m_triangles[th.iIdx].hEdges[eidx] = this->AddEdge(edges[eidx]);
      }
      else
      {
        auto linkIt2 = linkIt1->second.find(edges[eidx].hVertices[1]);
        //
        if ( linkIt2 == linkIt1->second.end() )
        {
          poly_EdgeHandle eh( m_links.size() );

          linkIt1->second.insert({edges[eidx].hVertices[1], eh});

          m_links.insert( {eh, {th}} );

          // Add edge and keep a link in a triangle.
          m_triangles[th.iIdx].hEdges[eidx] = this->AddEdge(edges[eidx]);
        }
        else
        {
          m_links.find(linkIt2->second)->second.push_back(th);

          m_triangles[th.iIdx].hEdges[eidx] = linkIt2->second;
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

int mobius::poly_Mesh::CountTriangles(const poly_EdgeHandle he) const
{
  auto linkIt = m_links.find(he);
  if ( linkIt == m_links.end() )
    return 0;

  return int( linkIt->second.size() );
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

bool mobius::poly_Mesh::GetTriangles(const poly_EdgeHandle                    he,
                                     std::unordered_set<poly_TriangleHandle>& hts) const
{
  auto linkIt = m_links.find(he);
  if ( linkIt == m_links.end() )
    return false;

  for ( const auto& ht : linkIt->second )
    hts.insert(ht);

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::FindAdjacentByEdges(const poly_TriangleHandle         ht,
                                            std::vector<poly_TriangleHandle>& hts) const
{
  poly_Triangle t;
  this->GetTriangle(ht, t);

  poly_VertexHandle hv[3];
  t.GetVertices(hv[0], hv[1], hv[2]);

  poly_Edge edges[3] = { poly_Edge(hv[0], hv[1]),
                         poly_Edge(hv[1], hv[2]),
                         poly_Edge(hv[2], hv[0]) };

  poly_EdgeHandle hes[3] = { this->FindEdge(edges[0]),
                             this->FindEdge(edges[1]),
                             this->FindEdge(edges[2]) };
  //
  for ( int j = 0; j < 3; ++j )
  {
    if ( hes[j].iIdx == Mobius_InvalidHandleIndex )
      return false;
  }

  // Find triangles by edges.
  for ( int j = 0; j < 3; ++j )
  {
    std::vector<poly_TriangleHandle> edgeTris;

    if ( !this->GetTriangles(hes[j], edgeTris) )
      return false;

    for ( const auto& eth : edgeTris )
      if ( eth != ht )
        hts.push_back(eth);
  }

  return true;
}

//-----------------------------------------------------------------------------

void mobius::poly_Mesh::FindAdjacent(const poly_VertexHandle                  hv,
                                     std::unordered_set<poly_TriangleHandle>& hts,
                                     const std::unordered_set<int>&           domain) const
{
  const std::unordered_set<poly_TriangleHandle>&
    vertexTris = m_vertices[hv.iIdx].GetTriangleRefs();

  for ( const auto& vth : vertexTris )
  {
    if ( !domain.empty() && ( domain.find( m_triangles[vth.iIdx].GetFaceRef() ) == domain.end() ) )
      continue; // Skip faces that are out of interest.

    hts.insert(vth);
  }
}

//-----------------------------------------------------------------------------

void mobius::poly_Mesh::FindAdjacent(const poly_VertexHandle                hv,
                                     std::unordered_set<poly_VertexHandle>& hvs,
                                     bool&                                  isBoundary,
                                     const std::unordered_set<int>&         domain) const
{
  isBoundary = false;

  // Take all triangles containing this vertex. Do not pass the domain here as we want
  // to take all triangles, including out-of-domain ones and then reason about the
  // feature boundaries (if we filter out the out-of-domain triangles here, we won't
  // be able to detect the boundary).
  std::unordered_set<poly_TriangleHandle> ths;
  this->FindAdjacent(hv, ths);

  // Adjacent face IDs.
  std::unordered_set<int> faceIDs;

  // Vertices and their domains.
  std::unordered_map<poly_VertexHandle, int> vDomains;

  // Add the neighbor triangles' vertices to the result.
  for ( const auto& th : ths )
  {
    poly_Triangle t;
    this->GetTriangle(th, t);

    if ( t.IsDeleted() )
      continue;

    faceIDs.insert( t.GetFaceRef() );

    for ( int k = 0; k < 3; ++k )
    {
      if ( t.hVertices[k] != hv )
      {
        vDomains.insert({t.hVertices[k], t.GetFaceRef()});

        // Check if that's a boundary vertex.
        if ( !isBoundary )
        {
          // Compose a link.
          poly_EdgeHandle eh = this->FindEdge( poly_Edge(hv, t.hVertices[k]) );

          // Check if that's a boundary link.
          auto linkIt = m_links.find(eh);
          //
          if ( ( linkIt != m_links.end() ) && (linkIt->second.size() < 2) )
            isBoundary = true;
        }
      }
    }
  }

  if ( !isBoundary && (faceIDs.size() > 1) )
    isBoundary = true;

  // Compose the result.
  for ( const auto& tuple : vDomains )
  {
    if ( domain.empty() || domain.count(tuple.second) )
    {
      hvs.insert(tuple.first);
    }
  }
}

//-----------------------------------------------------------------------------

void mobius::poly_Mesh::FindAdjacentByVertices(const poly_TriangleHandle                ht,
                                               std::unordered_set<poly_TriangleHandle>& hts) const
{
  poly_Triangle t;
  this->GetTriangle(ht, t);

  this->FindAdjacent(t.hVertices[0], hts);
  this->FindAdjacent(t.hVertices[1], hts);
  this->FindAdjacent(t.hVertices[2], hts);
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::CanFlip(const poly_EdgeHandle he,
                                const double          normDevRad,
                                const double          planeDevRad,
                                const bool            checkJacobian,
                                const bool            checkWing,
                                poly_TriangleHandle&  ht0,
                                poly_TriangleHandle&  ht1,
                                poly_VertexHandle&    a,
                                poly_VertexHandle&    b,
                                poly_VertexHandle&    x,
                                poly_VertexHandle&    y,
                                t_xyz&                norm0,
                                t_xyz&                norm1) const
{
  /*          a
               o
               | \
               |  \
               |   \
             x o----o y
                \   |
                 \  |
                  \ |
                    o
                     b
  */
  a = b = x = y = poly_VertexHandle(Mobius_InvalidHandleIndex);

  std::vector<poly_TriangleHandle> hts;
  if ( !this->GetTriangles(he, hts) )
    return false;

  if ( hts.size() != 2 )
    return false;

  // Get scaled Jacobians to control the quality of the triangles
  // on edge flip (we do not want to make it worse).
  double J_before[2] = {0., 0.};
  //
  if ( checkJacobian )
  {
    J_before[0] = this->ComputeScaledJacobian(hts[0]);
    J_before[1] = this->ComputeScaledJacobian(hts[1]);
  }

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

  // Check scaled Jacobians after edge flip.
  if ( checkJacobian )
  {
    const double J_after[2] = { this->ComputeScaledJacobian(a_coords, x_coords, b_coords),
                                this->ComputeScaledJacobian(a_coords, y_coords, b_coords) };
    //
    if ( std::min(J_after[0], J_after[1]) < std::min(J_before[0], J_before[1]) )
      return false;
  }

  if ( checkWing )
  {
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
  }

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::CanFlip(const poly_EdgeHandle he,
                                const double          normDevRad,
                                const double          planeDevRad,
                                const bool            checkJacobian,
                                const bool            checkWing) const
{
  poly_TriangleHandle hts[2];
  poly_VertexHandle   a, b, x, y;
  t_xyz               norm0, norm1;
  //
  return this->CanFlip(he, normDevRad, planeDevRad, checkJacobian, checkWing,
                       hts[0], hts[1], a, b, x, y, norm0, norm1);
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::FlipEdge(const poly_EdgeHandle he,
                                 const double          normDevRad,
                                 const double          planeDevRad,
                                 const bool            checkJacobian,
                                 const bool            checkWing)
{
  poly_VertexHandle   a, b, x, y;
  poly_TriangleHandle hts[2];
  t_xyz               norm0, norm1;
  //
  if ( !this->CanFlip(he, normDevRad, planeDevRad, checkJacobian, checkWing,
                      hts[0], hts[1], a, b, x, y, norm0, norm1) )
    return false;

  // Get triangles to rotate.
  poly_Triangle ts[2];
  this->GetTriangle(hts[0], ts[0]);
  this->GetTriangle(hts[1], ts[1]);

  // Compute norms to preserve orientations.
  t_xyz testN[2];
  this->ComputeNormal(a, x, b, testN[0]);
  this->ComputeNormal(b, y, a, testN[1]);

  // Add new (rotated) triangles.
  poly_TriangleHandle newHts[2];
  //
  if ( testN[0].Dot(norm0) > 0 )
  {
    newHts[0] = this->AddTriangle( a, x, b, ts[0].GetFaceRef() );
  }
  else
  {
    newHts[0] = this->AddTriangle( b, x, a, ts[0].GetFaceRef() );
  }
  //
  if ( testN[1].Dot(norm1) > 0 )
  {
    newHts[1] = this->AddTriangle( b, y, a, ts[1].GetFaceRef() );
  }
  else
  {
    newHts[1] = this->AddTriangle( a, y, b, ts[1].GetFaceRef() );
  }

  // Remove original triangles.
  this->RemoveTriangle(hts[0]);
  this->RemoveTriangle(hts[1]);

  // Update links.
  this->updateLink(he, hts[0], newHts[0]);
  this->updateLink(he, hts[1], newHts[1]);

  return true;
}

//-----------------------------------------------------------------------------

int mobius::poly_Mesh::FlipEdges(const double normDevRad,
                                 const double planeDevRad)
{
  int nbFlips = 0;

  for ( poly_Mesh::EdgeIterator eit(this); eit.More(); eit.Next() )
  {
    const poly_EdgeHandle eh = eit.Current();

    if ( this->FlipEdge(eh, normDevRad, planeDevRad) )
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

//-----------------------------------------------------------------------------

mobius::poly_EdgeHandle
  mobius::poly_Mesh::FindEdge(const poly_VertexHandle& hv0,
                              const poly_VertexHandle& hv1) const
{
  return this->FindEdge( poly_Edge(hv0, hv1) );
}

//-----------------------------------------------------------------------------

mobius::poly_EdgeHandle
  mobius::poly_Mesh::FindEdge(const poly_TriangleHandle ht0,
                              const poly_TriangleHandle ht1) const
{
  poly_Triangle t0, t1;
  //
  if ( !this->GetTriangle(ht0, t0) )
    return poly_EdgeHandle();
  //
  if ( !this->GetTriangle(ht1, t1) )
    return poly_EdgeHandle();

  poly_VertexHandle a, b, c, d, e, f;
  t0.GetVertices(a, b, c);
  t1.GetVertices(d, e, f);

  // Get all edges.
  poly_Edge t0_edges[3] = { poly_Edge(a, b), poly_Edge(b, c), poly_Edge(c, a) };
  poly_Edge t1_edges[3] = { poly_Edge(d, e), poly_Edge(e, f), poly_Edge(f, d) };

  for ( int i = 0; i < 3; ++i )
    for ( int j = 0; j < 3; ++j )
      if ( t0_edges[i] == t1_edges[j] )
        return this->FindEdge(t0_edges[i]);

  return poly_EdgeHandle();
}

//-----------------------------------------------------------------------------

mobius::poly_VertexHandle
  mobius::poly_Mesh::FindVertex(const poly_TriangleHandle ht,
                                const poly_EdgeHandle     he,
                                int&                      vidx) const
{
  poly_Triangle t;
  //
  if ( !this->GetTriangle(ht, t) )
    return poly_VertexHandle();

  poly_Edge e;
  //
  if ( !this->GetEdge(he, e) )
    return poly_VertexHandle();

  for ( int i = 0; i < 3; ++i )
    for ( int j = 0; j < 2; ++j )
      if ( t.hVertices[i] == e.hVertices[j] )
      {
        vidx = i;
        return t.hVertices[i];
      }

  return poly_VertexHandle();
}

//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::CollapseEdge(const poly_EdgeHandle&         he,
                                     const bool                     checkBorderOn,
                                     const bool                     checkDegenOn,
                                     const double                   prec,
                                     const std::unordered_set<int>& domain)
{
  // Get triangles to remove.
  std::unordered_set<poly_TriangleHandle> hts2Remove;
  if ( !this->GetTriangles(he, hts2Remove) )
    return false;

  if ( checkBorderOn && (hts2Remove.size() < 2) )
    return false; // Collapsing a border edge distorts the mesh badly.

  // Get edge to collapse.
  poly_Edge e;
  if ( !this->GetEdge(he, e) )
    return false;

  // Get new vertex position.
  t_xyz V[2];
  this->GetVertex(e.hVertices[0], V[0]);
  this->GetVertex(e.hVertices[1], V[1]);
  //
  t_xyz Vm = (V[0] + V[1])*0.5;

  /* Check if edge collapse is not going to produce any degenerated triangles */
  if ( checkDegenOn )
  {
    for ( const auto& ht2Remove : hts2Remove )
    {
      std::unordered_set<poly_TriangleHandle> ths2Edit;
      this->FindAdjacentByVertices(ht2Remove, ths2Edit);

      // Get the vertex to survive (the one opposite to the collapsed edge).
      const poly_VertexHandle a = this->GetOppositeVertex(ht2Remove, he);

      t_xyz Va;
      this->GetVertex(a, Va);

      // Check neighbor triangles.
      for ( const auto& th2Check : ths2Edit )
      {
        // Skip the triangles that are supposed to be removed.
        if ( hts2Remove.find(th2Check) != hts2Remove.end() )
          continue;

        poly_Triangle t2Check;
        this->GetTriangle(th2Check, t2Check);

        if ( t2Check.IsDeleted() )
          continue;

        std::vector<t_xyz> t2CheckVerts;
        int ci = -1;
        const poly_VertexHandle c = this->FindVertex(th2Check, he, ci);
        //
        for ( int j = 0; j < 3; ++j )
        {
          if ( t2Check.hVertices[j] != c )
          {
            t_xyz Vj;
            this->GetVertex(t2Check.hVertices[j], Vj);

            t2CheckVerts.push_back(Vj);
          }
        }

        // Virtually move `c` to `Vm` for testing.
        t2CheckVerts.push_back(Vm);

        if ( t2CheckVerts.size() != 3 )
          continue;

        if ( this->IsDegenerated(t2CheckVerts[0], t2CheckVerts[1], t2CheckVerts[2], prec) )
        {
          return false;
        }
      }
    }
  }

  // If the border is restricted, we first check that edge collapse
  // is not going to affect any border triangles.
  //
  // TODO: we can cache the adjacent triangles to now find them twice
  //       as we have exactly the same iteration below.
  if ( checkBorderOn )
  {
    // Adjacent face IDs.
    std::unordered_set<int> faceIDs;

    for ( const auto& ht2Remove : hts2Remove )
    {
      std::unordered_set<poly_TriangleHandle> ths2Check;
      this->FindAdjacentByVertices(ht2Remove, ths2Check);

      // Check neighbor triangles.
      for ( const auto& th2Check : ths2Check )
      {
        poly_Triangle t2Check;
        this->GetTriangle(th2Check, t2Check);

        if ( t2Check.IsDeleted() )
          continue;

        // Check domain.
        if ( !domain.empty() && ( domain.find( t2Check.GetFaceRef() ) == domain.end() ) )
          return false; // Do not allow collapsing an edge that would
                        // affect out-of-domain elements.

        // Remember face ID to check that we're not going to affect multiple faces.
        faceIDs.insert( t2Check.GetFaceRef() );

        poly_EdgeHandle
          eh2Check[3] = { t2Check.hEdges[0], t2Check.hEdges[1], t2Check.hEdges[2] };

        for ( int i = 0; i < 3; ++i )
        {
          const int numEdgeTris = this->CountTriangles(eh2Check[i]);
          if ( numEdgeTris != 2 )
            return false;
        }
      }
    }

    if ( faceIDs.size() > 1 )
      return false; // Edge collapse is not a cross-patch operation.
  }

  /* When here, we're sure we can modify the mesh, so let's insert vertices,
     edit and remove triangles, etc.
   */

  // Add the new vertex.
  const poly_VertexHandle hVm = this->AddVertex(Vm);

  // Remove and modify triangles.
  for ( const auto& ht2Remove : hts2Remove )
  {
    std::unordered_set<poly_TriangleHandle> ths2Edit;
    this->FindAdjacentByVertices(ht2Remove, ths2Edit);

    // Current triangle to remove.
    poly_Triangle t2Remove;
    this->GetTriangle(ht2Remove, t2Remove);

    // Get the vertex to survive (the one opposite to the collapsed edge).
    const poly_VertexHandle a = this->GetOppositeVertex(ht2Remove, he);

    // Edit neighbor triangles.
    for ( const auto& th2Edit : ths2Edit )
    {
      // Skip the triangles that are supposed to be removed.
      if ( hts2Remove.find(th2Edit) != hts2Remove.end() )
        continue;

      poly_Triangle& t2Edit = this->ChangeTriangle(th2Edit);

      int ci = -1;
      const poly_VertexHandle c = this->FindVertex(th2Edit, he, ci);
      //
      if ( !c.IsValid() )
        continue;

      // Move vertex using the non-const reference to the triangle.
      t2Edit.hVertices[ci] = hVm;

      // Edit the edges.
      for ( int ei = 0; ei < 3; ++ei )
      {
        poly_Edge& e2Edit = m_edges[t2Edit.hEdges[ei].iIdx];

        // Check if this edge is going to move.
        int  vi              = -1;
        int  numVertAffected = 0;
        //
        for ( int k = 0; k < 3; ++k )
        {
          if ( e2Edit.hVertices[0] == t2Remove.hVertices[k] )
          {
            numVertAffected++;
            vi = (e2Edit.hVertices[0] == t2Remove.hVertices[k]) ? 0 : 1;
          }

          if ( e2Edit.hVertices[1] == t2Remove.hVertices[k] )
          {
            numVertAffected++;
            vi = (e2Edit.hVertices[0] == t2Remove.hVertices[k]) ? 0 : 1;
          }
        }

        if ( numVertAffected == 2 )
        {
          // Remove edge.
          this->RemoveEdge(he);
        }

        if ( numVertAffected == 1 )
        {
          e2Edit.hVertices[vi] = hVm;
        }
      }

      // Add back reference to keep consistent adjacency.
      this->ChangeVertex(hVm)           .AddTriangleRef(th2Edit);
      this->ChangeVertex(e.hVertices[0]).SetDeleted();
      this->ChangeVertex(e.hVertices[1]).SetDeleted();
    }

    this->RemoveTriangle(ht2Remove);
  }

  // Remove edge.
  this->RemoveEdge(he);

  return true;
}

//-----------------------------------------------------------------------------

void mobius::poly_Mesh::Smooth(const int                      iter,
                               const std::unordered_set<int>& domain)
{
  // Find adjacent vertices for each vertex.
  std::unordered_map< poly_VertexHandle, std::unordered_set<poly_VertexHandle> > adj;
  //
  for ( VertexIterator vit(this); vit.More(); vit.Next() )
  {
    const poly_VertexHandle vh = vit.Current();

    // Find neighbors.
    bool isBoundary = false;
    std::unordered_set<poly_VertexHandle> adjVerts;
    this->FindAdjacent(vh, adjVerts, isBoundary, domain);

    if ( !isBoundary )
      adj.insert({vh, adjVerts});
  }

  int it = 0;

  // Move each vertex.
  while ( it++ < iter )
  {
    for ( const auto& vTuple : adj )
    {
      t_xyz avrg;
      int   num = 0;

      // Get the average position.
      for ( const auto& nvs : vTuple.second )
      {
        t_xyz coord;
        this->GetVertex(nvs, coord);

        avrg += coord;
        ++num;
      }

      if ( !num )
        continue;

      avrg /= num;
      this->ChangeVertex(vTuple.first).ChangeCoords() = avrg;
    }
  }
}

//-----------------------------------------------------------------------------

void mobius::poly_Mesh::updateLink(const poly_EdgeHandle     he,
                                   const poly_TriangleHandle htOld,
                                   const poly_TriangleHandle htNew)
{
  auto link = m_links.find(he);
  //
  if ( link == m_links.end() )
    return; // No such link.

  std::vector<poly_TriangleHandle> newTuple;
  //
  for ( const auto& ht : link->second )
  {
    if ( ht == htOld )
      newTuple.push_back(htNew);
    else
      newTuple.push_back(ht);
  }
  //
  m_links[he] = newTuple;
}
