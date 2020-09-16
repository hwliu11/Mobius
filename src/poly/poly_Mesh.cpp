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
  t0 = this->AddTriangle(htv[0], htv[1], hmv);
  t1 = this->AddTriangle(hmv, htv[1], htv[2]);
  t2 = this->AddTriangle(htv[0], hmv, htv[2]);

  // Remove the refined triangle.
  if ( remove )
    this->RemoveTriangle(ht);

  return true;
}
//-----------------------------------------------------------------------------

bool mobius::poly_Mesh::RefineByMidpoint(const poly_TriangleHandle ht)
{
  poly_TriangleHandle hrt[3];
  return this->RefineByMidpoint(ht, hrt[0], hrt[1], hrt[2]);
}
