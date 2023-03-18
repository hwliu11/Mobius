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

#ifndef poly_Mesh_HeaderFile
#define poly_Mesh_HeaderFile

// Poly includes
#include <mobius/poly_Edge.h>
#include <mobius/poly_Jacobian.h>
#include <mobius/poly_Quad.h>
#include <mobius/poly_SurfAdapter.h>
#include <mobius/poly_Triangle.h>
#include <mobius/poly_Vertex.h>

// Core includes
#include <mobius/core_IAlgorithm.h>
#include <mobius/core_Precision.h>

// Geom includes
#include <mobius/geom_PlaneSurface.h>

// Standard includes
#include <math.h>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <map>

namespace mobius {

class geom_PlaneSurface;

//! \ingroup MOBIUS_POLY
//!
//! Utilities.
class poly_MeshUtils
{
public:

  //! Computes center for the passed triple of vertices.
  //! \param[in]  v0     the first vertex.
  //! \param[in]  v1     the second vertex.
  //! \param[in]  v2     the third vertex.
  //! \param[out] center the computed center point.
  mobiusPoly_EXPORT static void
    ComputeCenter(const t_xyz& v0,
                  const t_xyz& v1,
                  const t_xyz& v2,
                  t_xyz&       center);

  //! Computes normal vector for the passed triple of vertices.
  //! \param[in]  v0  the first vertex.
  //! \param[in]  v1  the second vertex.
  //! \param[in]  v2  the third vertex.
  //! \param[out] norm the computed normal vector.
  //! \return true if the normal vector was computed successfully,
  //!         false -- otherwise.
  mobiusPoly_EXPORT static bool
    ComputeNormal(const t_xyz& v0,
                  const t_xyz& v1,
                  const t_xyz& v2,
                  t_xyz&       norm);

  //! Computes area for the passed triangle nodes.
  //! \param[in] hv0 the first vertex.
  //! \param[in] hv1 the second vertex.
  //! \param[in] hv2 the third vertex.
  //! \return the computed area.
  mobiusPoly_EXPORT static double
    ComputeArea(const t_xyz& v0,
                const t_xyz& v1,
                const t_xyz& v2);

  //! Auxiliary method to compute intertia props from a volumetric element
  //! enclosed by a surface mesh.
  mobiusPoly_EXPORT static bool
    ComputePyramidProps(const t_xyz&    v0,
                        const t_xyz&    v1,
                        const t_xyz&    v2,
                        const core_XYZ& apex,
                        double          gProps[10],
                        const int       nbPnts,
                        const double*   pnts);

private:

  poly_MeshUtils() = delete;
  poly_MeshUtils(const poly_MeshUtils&) = delete;
  void operator=(const poly_MeshUtils&) = delete;

};

//! \ingroup MOBIUS_POLY
//!
//! Data structure representing surface triangulation.
//!
//! \sa mobius::poly_ReadSTL
template <typename ElemTraits = poly_Traits>
class poly_Mesh : public core_IAlgorithm
{
public:

  //! Precision of Gauss cubature formulas.
  enum PropsComputationDensity {
    //! Values of volume and center of mass are exact for mesh,
    //! while moments of inertia might have errors, depending of element size.
    OnePoint,
    //! Values of volume and center of mass are exact for mesh,
    //! expressions for moments are polynomials of second order.
    ThreePoints
  };

// Construction & destruction:
public:

  //! Default ctor with optional diagnostic tools.
  poly_Mesh(core_ProgressEntry progress = nullptr,
            core_PlotterEntry  plotter  = nullptr)
  //
  : core_IAlgorithm(progress, plotter)
  {}

public:

  //! Creates a deep copy of this mesh without traits.
  //! \return the mesh copy.
  t_ptr<poly_Mesh<>> DeepCopyWithoutTraits()
  {
    t_ptr<poly_Mesh<>> copy = new poly_Mesh<>;
    //
    copy->__vertices  = this->__vertices;
    copy->__edges     = this->__edges;
    copy->__quads     = this->__quads;
    copy->__links     = this->__links;
    copy->__surfAdt   = this->__surfAdt;

    for ( const auto& tt : this->__triangles )
    {
      poly_Triangle<> t = tt.CopyWithoutTraits();

      copy->__triangles.push_back(t);
    }

    return copy;
  }

  //! Creates a deep copy of this mesh.
  //! \return the mesh copy.
  t_ptr<poly_Mesh> DeepCopy() const
  {
    t_ptr<poly_Mesh> copy = new poly_Mesh;
    //
    copy->__vertices  = this->__vertices;
    copy->__edges     = this->__edges;
    copy->__triangles = this->__triangles;
    copy->__quads     = this->__quads;
    copy->__links     = this->__links;
    copy->__surfAdt   = this->__surfAdt;

    return copy;
  }

  //! Extracts a mesh region for the given triangle IDs `tids`.
  //! Use this method to get a submesh of the existing mesh. The
  //! constructed submesh is a deep copy.
  //! \param[in] tids the IDs of the triangles to copy into the region
  //!                 mesh being constructed.
  //! \return the extracted region.
  t_ptr< poly_Mesh<ElemTraits> >
    ExtractRegion(const std::unordered_set<int>& tids) const
  {
    t_ptr< poly_Mesh<ElemTraits> > region = new poly_Mesh<ElemTraits>;

    // keep matching of original vertex handles and their copies 
    // to avoid vertex handles duplication in the region
    std::unordered_map<poly_VertexHandle, poly_VertexHandle> ovh2rvh;

    for ( TriangleIterator tit(this); tit.More(); tit.Next() )
    {
      if ( tids.find(tit.Current().iIdx) == tids.end() )
        continue;

      poly_Triangle<ElemTraits> t;
      this->GetTriangle(tit.Current(), t);

      poly_VertexHandle hv0;
      poly_VertexHandle hv1;
      poly_VertexHandle hv2;
      t.GetVertices(hv0, hv1, hv2);

      // Use the exisiting copy of the original vertex handle instead of coping it once again.
      //
      // First vertex handle
      auto vhpair = ovh2rvh.find(hv0);
      if ( vhpair != ovh2rvh.cend() )
      {
        // reuse the existing copy of the original vertex handle
        hv0 = (*vhpair).second;
      }
      else
      {
        // add a new vertex as matching map does not contain it.
        poly_Vertex v;
        this->GetVertex(hv0, v);
        poly_VertexHandle _hv = region->AddVertex(v);

        ovh2rvh[hv0] = _hv;
        hv0 = _hv;
      }

      // Second vertex handle
      vhpair = ovh2rvh.find(hv1);
      if ( vhpair != ovh2rvh.cend() )
      {
        // reuse the existing copy of the original vertex handle
        hv1 = (*vhpair).second;
      }
      else
      {
        // add a new vertex as matching map does not contain it.
        poly_Vertex v;
        this->GetVertex(hv1, v);
        poly_VertexHandle _hv = region->AddVertex(v);

        ovh2rvh[hv1] = _hv;
        hv1 = _hv;
      }

      // Third vertex handle
      vhpair = ovh2rvh.find(hv2);
      if ( vhpair != ovh2rvh.cend() )
      {
        // reuse the existing copy of the original vertex handle
        hv2 = (*vhpair).second;
      }
      else
      {
        // add a new vertex if matching map does not contain it.
        poly_Vertex v;
        this->GetVertex(hv2, v);
        poly_VertexHandle _hv = region->AddVertex(v);

        ovh2rvh[hv2] = _hv;
        hv2 = _hv;
      }

      poly_TriangleHandle newTh = region->AddTriangle( hv0, hv1, hv2, t.GetFaceRef() );

      // Copy traits.
      region->ChangeTriangle(newTh).traits = t.traits;
    }

    return region;
  }

  //! Copies other mesh into this.
  //! \param[in] other the mesh to copy into this mesh
  void Merge(const t_ptr< poly_Mesh<ElemTraits> >& other)
  {
    for ( poly_Mesh<ElemTraits>::TriangleIterator tit(other); tit.More(); tit.Next() )
    {
      poly_Triangle<ElemTraits> t;
      other->GetTriangle(tit.Current(), t);

      poly_VertexHandle hv0;
      poly_VertexHandle hv1;
      poly_VertexHandle hv2;
      t.GetVertices(hv0, hv1, hv2);

      poly_Vertex v;
      other->GetVertex(hv0, v);
      hv0 = this->AddVertex(v);

      other->GetVertex(hv1, v);
      hv1 = this->AddVertex(v);

      other->GetVertex(hv2, v);
      hv2 = this->AddVertex(v);

      poly_TriangleHandle newTh = this->AddTriangle(hv0, hv1, hv2);

      // Copy traits.
      this->ChangeTriangle(newTh).traits = t.traits;
    }
  }

  //! Grows a mesh region by connectivity starting from the passed
  //! triangle handle `th`.
  //! \param[in]  th     the seed triangle handle.
  //! \param[out] region the collected region.
  void GrowRegion(const poly_TriangleHandle                th,
                  std::unordered_set<poly_TriangleHandle>& region)
  {
    // Fill adjacency matrix.
    std::map< poly_TriangleHandle, std::unordered_set<poly_TriangleHandle> > adj;
    //
    for ( TriangleIterator tit(this); tit.More(); tit.Next() )
    {
      std::unordered_set<poly_TriangleHandle> adjacent;
      this->FindAdjacentByVertices(tit.Current(), adjacent);

      adj.insert({tit.Current(), adjacent});
    }

    // Iterate over the adjacency matrix starting from the passed
    // triangle `th` and add all visited triangle indices to the region.
    // We use a pre-computed adjacency matrix as it is a little bit faster
    // than querying triangle adjacency information as long as we go.
    std::unordered_set<poly_TriangleHandle> processedRows;
    std::stack<poly_TriangleHandle> stack;
    stack.push(th);
    //
    while ( !stack.empty() )
    {
      const auto& next = stack.top();
      stack.pop();

      processedRows.insert(next);

      const auto& row = adj.find(next);

      for ( const auto& curr : row->second )
      {
        region.insert(curr);

        if ( processedRows.find(curr) == processedRows.end() )
          stack.push(curr);
      }
    }
  }

/* CAD link */
public:

  //! Sets CAD surface adapter to establish a reference from the
  //! discrete surface to the smooth analytical surface (if any).
  //! \param[in] adt the adapter to set.
  void
    SetSurfAdapter(const t_ptr<poly_SurfAdapter>& adt)
  {
    __surfAdt = adt;
  }

public:

  //! Checks if the passed edges intersect in the parametric domain of
  //! the corresponding surface.
  //! \param[in] tag the domain ID.
  //! \param[in] eh0 the first edge to check.
  //! \param[in] eh1 the second edge to check.
  //! \return true if the edges intersect, false -- otherwise.
  mobiusPoly_EXPORT bool
    AreSelfIntersecting(const int             tag,
                        const poly_EdgeHandle eh0,
                        const poly_EdgeHandle eh1) const;

  //! Checks if the passed edges intersect in the parametric domain of
  //! the corresponding surface.
  //! \param[in] tag  the domain ID.
  //! \param[in] ehs0 the first group of edges to check.
  //! \param[in] ehs1 the second group of edges to check.
  //! \return true if the edges intersect, false -- otherwise.
  mobiusPoly_EXPORT bool
    AreSelfIntersecting(const int                           tag,
                        const std::vector<poly_EdgeHandle>& ehs0,
                        const std::vector<poly_EdgeHandle>& ehs1) const;

  //! Checks if the boundary and inner edges intersect in the passed domain.
  //! \param[in] domain the domain to check.
  //! \return true if the edges intersect, false -- otherwise.
  mobiusPoly_EXPORT bool
    AreSelfIntersecting(const std::unordered_set<int>& domain) const;

public:

  //! Calculates axis-aligned boundary box for the mesh.
  //! \param[out] xMin min X.
  //! \param[out] xMax max X.
  //! \param[out] yMin min Y.
  //! \param[out] yMax max Y.
  //! \param[out] zMin min Z.
  //! \param[out] zMax max Z.
  void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const
  {
    double x_min = DBL_MAX, x_max = -DBL_MAX;
    double y_min = DBL_MAX, y_max = -DBL_MAX;
    double z_min = DBL_MAX, z_max = -DBL_MAX;

    for ( auto vit = __vertices.cbegin(); vit != __vertices.cend(); ++vit )
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

  //! Calculates barycenter.
  //! \return the computed point.
  core_XYZ ComputeCenter() const
  {
    core_XYZ center;
    t_xyz    vertex;
    int      vNum = 0;
    //
    for ( VertexIterator vIt(this); vIt.More(); vIt.Next() )
    {
      this->GetVertex(vIt.Current(), vertex);
      center += vertex;
      ++vNum;
    }

    if ( vNum > 1 )
    {
      center /= vNum;
    }

    return center;
  }

  //! Computes area for the passed triangle.
  //! \param[in] hv0 the first vertex handle.
  //! \param[in] hv1 the second vertex handle.
  //! \param[in] hv2 the third vertex handle.
  //! \return the computed area.
  double
    ComputeArea(const poly_VertexHandle hv0,
                const poly_VertexHandle hv1,
                const poly_VertexHandle hv2) const
  {
    t_xyz tv[3];
    this->GetVertex(hv0, tv[0]);
    this->GetVertex(hv1, tv[1]);
    this->GetVertex(hv2, tv[2]);

    // Compute area.
    return poly_MeshUtils::ComputeArea(tv[0], tv[1], tv[2]);
  }

  //! Computes area for the passed triangle.
  //! \param[in] ht handle of the triangle to compute the area for.
  //! \return the computed area.
  double ComputeArea(const poly_TriangleHandle ht) const
  {
    // Get triangle by its handle.
    poly_Triangle<ElemTraits> t;
    if ( !this->GetTriangle(ht, t) )
      return false;

    // Get vertices on the triangle.
    poly_VertexHandle htv[3];
    //
    t.GetVertices(htv[0], htv[1], htv[2]);
    //
    return ComputeArea(htv[0], htv[1], htv[2]);
  }

  //! Calculates area.
  //! \return the computed area.
  double ComputeArea()
  {
    double area = 0.;

    for ( TriangleIterator tit(this); tit.More(); tit.Next() )
    {
      area += ComputeArea(tit.Current());
    }

    return area;
  }

  //! Computes general properties (volume and axes of inertia) for the mesh.
  //! Calculation of volume properties is performed by numerical integration
  //! over triangle surfaces using Gauss cubature formulas.
  //! \param[in]  density             the precision of calculations.
  //! \param[out] volume              the computed volume.
  //! \param[out] firstAxisOfInertia  the first axis of inertia.
  //! \param[out] secondAxisOfInertia the second axis of inertia.
  //! \param[out] thirdAxisOfInertia  the third axis of inertia.
  void
    ComputeProps(const PropsComputationDensity density,
                 double&                       volume,
                 core_XYZ&                     firstAxisOfInertia,
                 core_XYZ&                     secondAxisOfInertia,
                 core_XYZ&                     thirdAxisOfInertia) const
  {
    // Gauss points for barycentric coordinates
    static double pntWg[] =
    {
      1. / 3., 1. / 3., 1. / 2., /* 1-point-based  */
      1. / 6., 1. / 6., 1. / 6., /* 3-points-based */
      2. / 3., 1. / 6., 1. / 6.,
      1. / 6., 2. / 3., 1. / 6. 
    };
    //
    int nbPoints = 1;
    double* gaussPnts = &pntWg[0];
    if ( density == PropsComputationDensity::ThreePoints )
    {
      nbPoints = 3;
      gaussPnts = &pntWg[3];
    }

    // Array to store global properties
    double gProps[10] = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0 };

    core_XYZ polyCenter = this->ComputeCenter();

    for ( TriangleIterator trIt(this); trIt.More(); trIt.Next() )
    {
      poly_Triangle<ElemTraits> triangle;
      this->GetTriangle(trIt.Current(), triangle);

      poly_VertexHandle hv0, hv1, hv2;
      triangle.GetVertices(hv0, hv1, hv2);

      t_xyz tv[3];
      this->GetVertex(hv0, tv[0]);
      this->GetVertex(hv1, tv[1]);
      this->GetVertex(hv2, tv[2]);

      poly_MeshUtils::ComputePyramidProps(tv[0], tv[1], tv[2], polyCenter, gProps, nbPoints, gaussPnts);
    }

    // initialize output parameters
    //
    volume = gProps[0];
    //
    firstAxisOfInertia  = core_XYZ(gProps[1], gProps[2], gProps[3]);
    secondAxisOfInertia = core_XYZ(gProps[4], gProps[5], gProps[6]);
    thirdAxisOfInertia  = core_XYZ(gProps[7], gProps[8], gProps[9]);
  }

  //! Refines the triangle of interest by its midpoint.
  //! \param[in]  ht  handle of the triangle to refine.
  //! \param[out] ht0 handle of the first created triangle.
  //! \param[out] ht1 handle of the second created triangle.
  //! \param[out] ht2 handle of the third created triangle.
  //! \return true in case of success, false -- otherwise.
  bool
    RefineByMidpoint(const poly_TriangleHandle ht,
                     poly_TriangleHandle&      ht0,
                     poly_TriangleHandle&      ht1,
                     poly_TriangleHandle&      ht2)
  {
    // Get the triangle to split.
    poly_Triangle<ElemTraits> t;
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
    ht0 = this->AddTriangle( htv[0], htv[1], hmv,    t.GetFaceRef() );
    ht1 = this->AddTriangle( hmv,    htv[1], htv[2], t.GetFaceRef() );
    ht2 = this->AddTriangle( htv[0], hmv,    htv[2], t.GetFaceRef() );

    // Remove the refined triangle.
    this->RemoveTriangle(ht);

    return true;
  }

  //! Refines the triangle of interest by its midpoint.
  //! \param[in] ht handle of the triangle to refine.
  //! \return true in case of success, false -- otherwise.
  bool RefineByMidpoint(const poly_TriangleHandle ht)
  {
    poly_TriangleHandle hrt[3];
    return this->RefineByMidpoint(ht, hrt[0], hrt[1], hrt[2]);
  }

  //! Refines the passed triangle by midedge subdivision.
  //! \param[in]  ht  handle of the triangle to refine.
  //! \param[out] hts newly constructed triangles.
  //! \return true in case of success, false -- otherwise.
  bool
    RefineByMidedges(const poly_TriangleHandle         ht,
                     std::vector<poly_TriangleHandle>& hts)
  {
    // Get the triangle to refine.
    poly_Triangle<ElemTraits> t;
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
      poly_Triangle<ElemTraits> nextTri;
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
    __links.erase(hes[0]);
    __links.erase(hes[1]);
    __links.erase(hes[2]);
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

      __links.insert({newEh, newTris});
    }

    return true;
  }

  //! Refines the passed triangle by midedge subdivision.
  //! \param[in] ht handle of the triangle to refine.
  //! \return true in case of success, false -- otherwise.
  bool RefineByMidedges(const poly_TriangleHandle ht)
  {
    std::vector<poly_TriangleHandle> hts;
    return this->RefineByMidedges(ht, hts);
  }

  //! Returns the opposite vertex for the given edge on the
  //! passed triangle.
  //! \param[in] ht triangle handle.
  //! \param[in] he edge handle.
  //! \return the opposite vertex.
  poly_VertexHandle
    GetOppositeVertex(const poly_TriangleHandle ht,
                      const poly_EdgeHandle     he) const
  {
    poly_Edge edge;
    if ( !this->GetEdge(he, edge) )
      return poly_VertexHandle();

    poly_Triangle<ElemTraits> nbrTri;
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

  //! Computes normal vector for the passed triple of vertices.
  //! \param[in]  hv0  the first vertex handle.
  //! \param[in]  hv1  the second vertex handle.
  //! \param[in]  hv2  the third vertex handle.
  //! \param[out] norm the computed normal vector.
  //! \return true if the normal vector was computed successfully,
  //!         false -- otherwise.
  bool
    ComputeNormal(const poly_VertexHandle hv0,
                  const poly_VertexHandle hv1,
                  const poly_VertexHandle hv2,
                  t_xyz&                  norm) const
  {
    t_xyz tv[3];
    //
    this->GetVertex(hv0, tv[0]);
    this->GetVertex(hv1, tv[1]);
    this->GetVertex(hv2, tv[2]);

    return poly_MeshUtils::ComputeNormal(tv[0], tv[1], tv[2], norm);
  }

  //! Computes center for the passed triple of vertices.
  //! \param[in]  hv0    the first vertex handle.
  //! \param[in]  hv1    the second vertex handle.
  //! \param[in]  hv2    the third vertex handle.
  //! \param[out] center the computed center point.
  void
    ComputeCenter(const poly_VertexHandle hv0,
                  const poly_VertexHandle hv1,
                  const poly_VertexHandle hv2,
                  t_xyz&                  center) const
  {
    t_xyz tv[3];
    //
    this->GetVertex(hv0, tv[0]);
    this->GetVertex(hv1, tv[1]);
    this->GetVertex(hv2, tv[2]);

    poly_MeshUtils::ComputeCenter(tv[0], tv[1], tv[2], center);
  }

  //! Computes normal vector for the triangle in question.
  //! \param[in]  ht   handle of the triangle in question.
  //! \param[out] norm computed normal vector.
  //! \return true if the normal vector was computed successfully,
  //!         false -- otherwise.
  bool
    ComputeNormal(const poly_TriangleHandle ht,
                  t_xyz&                    norm) const
  {
    // Get triangle by its handle.
    poly_Triangle<ElemTraits> t;
    if ( !this->GetTriangle(ht, t) )
      return false;

    // Get vertices on the triangle.
    poly_VertexHandle htv[3];
    t_xyz             tv[3];
    //
    t.GetVertices(htv[0], htv[1], htv[2]);

    return this->ComputeNormal(htv[0], htv[1], htv[2], norm);
  }

  //! Computes the min scaled Jacobian for the passed triangle.
  //! \param[in] ht handle of the triangle to compute the metric for.
  //! \return the computed scaled Jacobian.
  double
    ComputeScaledJacobian(const poly_TriangleHandle ht) const
  {
    // Get element.
    poly_Triangle<ElemTraits> elem;
    this->GetTriangle(ht, elem);

    // Get nodes.
    poly_VertexHandle n0, n1, n2;
    elem.GetVertices(n0, n1, n2);
    //
    t_xyz v0, v1, v2;
    //
    this->GetVertex(n0, v0);
    this->GetVertex(n1, v1);
    this->GetVertex(n2, v2);

    double res = DBL_MAX;
    for ( int k = 0; k < 3; ++k )
    {
      t_uv   uv[3];
      double J[2][2] = { {0, 0}, {0, 0} };
      double J_det   = 0.;
      double J_det_n = 0.;

      // Compute for element.
      poly_Jacobian::Compute(v0, v1, v2, k, uv[0], uv[1], uv[2], J, J_det, J_det_n);

      res = std::min(res, J_det_n);
    }

    return res;
  }

  //! Computes the min scaled Jacobian for the passed triangle nodes.
  //! \param[in] v0 the 0-th vertex coordinates.
  //! \param[in] v1 the 1-st vertex coordinates.
  //! \param[in] v2 the 2-nd vertex coordinates.
  //! \return the computed scaled Jacobian.
  double
    ComputeScaledJacobian(const t_xyz& v0,
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

  //! Computes the max length for the edges of the passed triangle.
  //! \param[in] ht handle of the triangle to compute the max length for.
  //! \return the computed max length.
  double
    ComputeMaxLen(const poly_TriangleHandle ht) const
  {
    // Get triangle by its handle.
    poly_Triangle<ElemTraits> t;
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

  //! Checks if the passed triangle is degenerated w.r.t. the given
  //! precision value.
  //! \param[in] v0   the 0-th vertex of the triangle to check.
  //! \param[in] v1   the 1-st vertex of the triangle to check.
  //! \param[in] v2   the 2-nd vertex of the triangle to check.
  //! \param[in] prec the precision to use.
  //! \return true/false.
  bool
    IsDegenerated(const t_xyz& v0,
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

  //! Checks if the passed triangle is degenerated w.r.t. the given
  //! precision value.
  //! \param[in] ht   the triangle to check.
  //! \param[in] prec the precision to use.
  //! \return true/false.
  bool
    IsDegenerated(const poly_TriangleHandle ht,
                  const double              prec) const
  {
    // Get triangle by its handle.
    poly_Triangle<ElemTraits> t;
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

  //! Creates a submesh for the given triangle by subdividing it
  //! at the center points.
  //! WARNING: the resulting mesh is not conformal!
  //! \param[in] ht the triangle to subdivide.
  //! \return true in case of success, false -- otherwise.
  bool
    Subdivide(const poly_TriangleHandle ht)
  {
    // Get triangle by its handle.
    poly_Triangle<ElemTraits> t;
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
    this->AddTriangle( htv[0], hmv[0], hmv[2], t.GetFaceRef() );
    this->AddTriangle( hmv[0], htv[1], hmv[1], t.GetFaceRef() );
    this->AddTriangle( hmv[1], htv[2], hmv[2], t.GetFaceRef() );
    this->AddTriangle( hmv[0], hmv[1], hmv[2], t.GetFaceRef() );

    // Remove the subdivided triangle.
    this->RemoveTriangle(ht);

    return true;
  }

  //! Computes connectivity information as a set of mesh links
  //! married to the elements they share.
  void ComputeEdges()
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
      poly_TriangleHandle       th = tit.Current();
      poly_Triangle<ElemTraits> t;

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
          poly_EdgeHandle eh( int( __links.size() ) );

          t_vheh rec; rec.insert({edges[eidx].hVertices[1], eh});

          visitedEdges.insert( {edges[eidx].hVertices[0], rec});
          __links     .insert    ( {eh, {th}} );

          // Add edge and keep a link in a triangle.
          __triangles[th.iIdx].hEdges[eidx] = this->AddEdge(edges[eidx]);
        }
        else
        {
          auto linkIt2 = linkIt1->second.find(edges[eidx].hVertices[1]);
          //
          if ( linkIt2 == linkIt1->second.end() )
          {
            poly_EdgeHandle eh( int( __links.size() ) );

            linkIt1->second.insert({edges[eidx].hVertices[1], eh});

            __links.insert( {eh, {th}} );

            // Add edge and keep a link in a triangle.
            __triangles[th.iIdx].hEdges[eidx] = this->AddEdge(edges[eidx]);
          }
          else
          {
            __links.find(linkIt2->second)->second.push_back(th);

            __triangles[th.iIdx].hEdges[eidx] = linkIt2->second;
          }
        }
      }
    }
  }

  //! Cleans up the available edge info.
  void ClearEdges()
  {
    __links.clear();
    __edges.clear();
  }

  //! Returns the number of triangles sharing the passed edge.
  //! \param[in] he the edge handle to check.
  //! \return the number of triangles.
  int CountTriangles(const poly_EdgeHandle he) const
  {
    auto linkIt = __links.find(he);
    if ( linkIt == __links.end() )
      return 0;

    return int( linkIt->second.size() );
  }

  //! Returns handles of the triangles sharing the passed edge.
  //! \param[in]  he  the edge handle to check.
  //! \param[out] hts the output triangles.
  //! \return false if the links were not computed or there is no
  //!         edge with such a handle.
  bool
    GetTriangles(const poly_EdgeHandle             he,
                 std::vector<poly_TriangleHandle>& hts) const
  {
    auto linkIt = __links.find(he);
    if ( linkIt == __links.end() )
      return false;

    hts = linkIt->second;
    return true;
  }

  //! Returns handles of the triangles sharing the passed edge.
  //! \param[in]  he  the edge handle to check.
  //! \param[out] hts the output triangles.
  //! \return false if the links were not computed or there is no
  //!         edge with such a handle.
  bool
    GetTriangles(const poly_EdgeHandle                    he,
                 std::unordered_set<poly_TriangleHandle>& hts) const
  {
    auto linkIt = __links.find(he);
    if ( linkIt == __links.end() )
      return false;

    for ( const auto& ht : linkIt->second )
      hts.insert(ht);

    return true;
  }

  //! Finds adjacent triangles for the given one.
  //! \param[in]  ht the triangle in question.
  //! \param[out] hts the output triangles.
  //! \return false if the links were not computed or there is no
  //!         triangle with such a handle.
  bool
    FindAdjacentByEdges(const poly_TriangleHandle         ht,
                        std::vector<poly_TriangleHandle>& hts) const
  {
    std::unordered_set<poly_TriangleHandle> tset;
    bool res = FindAdjacentByEdges(ht, tset);
    for (const auto& eth : tset)
    {
      hts.push_back(eth);
    }

    return res;
  }

  //! Finds adjacent triangles for the given one.
  //! \param[in]  ht the triangle in question.
  //! \param[out] hts the output triangles.
  //! \return false if the links were not computed or there is no
  //!         triangle with such a handle.
  bool
    FindAdjacentByEdges(const poly_TriangleHandle                ht,
                        std::unordered_set<poly_TriangleHandle>& hts) const
  {
    // TODO: this method uses linear search for edges.
    // DO NOT USE THIS METHOD!!!

    poly_Triangle<ElemTraits> t;
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
      std::unordered_set<poly_TriangleHandle> edgeTris;

      if ( !this->GetTriangles(hes[j], edgeTris) )
        return false;

      for ( const auto& eth : edgeTris )
        if ( eth != ht )
          hts.insert(eth);
    }

    return true;
  }

  //! Finds all triangles sharing the given vertex.
  //! \param[in]  hv     the vertex in question.
  //! \param[out] hts    the found triangles.
  //! \param[in]  domain the optional domain to keep only the
  //!                    faces of interest.
  void
    FindAdjacent(const poly_VertexHandle                  hv,
                 std::unordered_set<poly_TriangleHandle>& hts,
                 const std::unordered_set<int>&           domain = std::unordered_set<int>()) const
  {
    const std::unordered_set<poly_TriangleHandle>&
      vertexTris = __vertices[hv.iIdx].GetTriangleRefs();

    for ( const auto& vth : vertexTris )
    {
      if ( !domain.empty() && ( domain.find( __triangles[vth.iIdx].GetFaceRef() ) == domain.end() ) )
        continue; // Skip faces that are out of interest.

      hts.insert(vth);
    }
  }

  //! Finds all vertices adjacent to the given vertex.
  //! \param[in]  hv         the vertex in question.
  //! \param[out] hvs        the found neighbor vertices.
  //! \param[out] isBoundary the Boolean flag indicating whether
  //!                        the passed `hv` vertex is found to be
  //!                        the boundary one.
  //! \param[out] faceRefs   the surrounding face refs.
  //! \param[in]  domain     the optional domain to keep only the
  //!                        faces of interest.
  void
    FindAdjacent(const poly_VertexHandle                hv,
                 std::unordered_set<poly_VertexHandle>& hvs,
                 bool&                                  isBoundary,
                 std::unordered_set<int>&               faceRefs,
                 const std::unordered_set<int>&         domain) const
  {
    isBoundary = false;

    // Take all triangles containing this vertex. Do not pass the domain here as we want
    // to take all triangles, including out-of-domain ones and then reason about the
    // feature boundaries (if we filter out the out-of-domain triangles here, we won't
    // be able to detect the boundary).
    std::unordered_set<poly_TriangleHandle> ths;
    this->FindAdjacent(hv, ths);

    // Vertices and their domains.
    std::unordered_map<poly_VertexHandle, int> vDomains;

    // Add the neighbor triangles' vertices to the result.
    for ( const auto& th : ths )
    {
      poly_Triangle<ElemTraits> t;
      this->GetTriangle(th, t);

      if ( t.IsDeleted() )
        continue;

      faceRefs.insert( t.GetFaceRef() );

      for ( int k = 0; k < 3; ++k )
      {
        if ( t.hVertices[k] != hv )
        {
          vDomains.insert({t.hVertices[k], t.GetFaceRef()});

          // Check if that's a boundary vertex.
          if ( !isBoundary )
          {
            poly_EdgeHandle eh;
            for ( int j = 0; j < 3; ++j )
            {
              const poly_Edge& eCandidate = __edges[t.hEdges[j].iIdx];

              if ( ((eCandidate.hVertices[0] == hv) && (eCandidate.hVertices[1] == t.hVertices[k])) ||
                   ((eCandidate.hVertices[1] == hv) && (eCandidate.hVertices[0] == t.hVertices[k])) )
                eh = t.hEdges[j];
            }

            if ( eh.IsValid() )
            {
              // Check if that's a boundary link.
              auto linkIt = __links.find(eh);
              //
              if ( ( linkIt != __links.end() ) && (linkIt->second.size() < 2) )
                isBoundary = true;
            }
          }
        }
      }
    }

    if ( !isBoundary && (faceRefs.size() > 1) )
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

  //! Finds adjacent triangles for the given edge.
  //! \param[in]  he  the edge in question.
  //! \param[out] hts the output triangles.
  //! \return false if adjacency information is not available or if the
  //!         passed edge handle is invalid.
  bool
    FindAdjacent(const poly_EdgeHandle             he,
                 std::vector<poly_TriangleHandle>& hts) const
  {
    if ( !he.IsValid() )
      return false;

    auto linkIt = __links.find(he);
    //
    if ( linkIt == __links.end() )
      return false;

    hts = linkIt->second;
    return true;
  }

  //! Finds adjacent triangles for the given one.
  //! \param[in]  ht the triangle in question.
  //! \param[out] hts the output triangles.
  void
    FindAdjacentByVertices(const poly_TriangleHandle                ht,
                           std::unordered_set<poly_TriangleHandle>& hts) const
  {
    poly_Triangle<ElemTraits> t;
    this->GetTriangle(ht, t);

    this->FindAdjacent(t.hVertices[0], hts);
    this->FindAdjacent(t.hVertices[1], hts);
    this->FindAdjacent(t.hVertices[2], hts);
  }

  //! Checks if the passed edge can be flipped and returns the pair of
  //! triangles to flip. The links should have been computed before you
  //! call this method.
  //! \param[in]  he            the edge to check.
  //! \param[in]  normDevRad    the allowed absolute normal deviation (radians).
  //! \param[in]  planeDevRad   the allowed in-plane deviation (radians).
  //! \param[in]  checkJacobian the Boolean flag determining whether to check mesh
  //!                           quality metric (scaled Jacobian) on edge flip to
  //!                           ensure that flipping does not make the mesh worse.
  //! \param[in]  checkWing     the Boolean flag indicating whether to check the projection
  //!                           of the neighbor links to the edge supposed for flipping.
  //!                           If the dot product is negative, that's a "wing-like" shape
  //!                           that should not undergo edge flipping to avoid overlappings.
  //! \param[out] ht0           the first triangle.
  //! \param[out] ht1           the second triangle.
  //! \param[out] a             the opposite vertex on the first triangle.
  //! \param[out] b             the opposite vertex on the second triangle.
  //! \param[out] x             the first vertex on the edge to flip.
  //! \param[out] y             the second vertex on the edge to flip.
  //! \param[out] norm0         the normal vector for the first triangle.
  //! \param[out] norm1         the normal vector for the second triangle.
  //! \return true/false.
  bool
    CanFlip(const poly_EdgeHandle he,
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
    (void) planeDevRad;

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

    poly_Triangle<ElemTraits> t[2];
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

  //! Checks if the passed edge can be flipped. The links should have
  //! been computed before you call this method.
  //! \param[in] he            the edge to check.
  //! \param[in] normDevRad    the allowed absolute normal deviation (radians).
  //! \param[in] planeDevRad   the allowed in-plane deviation (radians).
  //! \param[in] checkJacobian the Boolean flag determining whether to check mesh
  //!                          quality metric (scaled Jacobian) on edge flip to
  //!                          ensure that flipping does not make the mesh worse.
  //! \param[in] checkWing     the Boolean flag indicating whether to check the projection
  //!                          of the neighbor links to the edge supposed for flipping.
  //!                          If the dot product is negative, that's a "wing-like" shape
  //!                          that should not undergo edge flipping to avoid overlappings.
  //! \return true/false.
  bool
    CanFlip(const poly_EdgeHandle he,
            const double          normDevRad    = 1./180.*M_PI,
            const double          planeDevRad   = 15./180.*M_PI,
            const bool            checkJacobian = true,
            const bool            checkWing     = true) const
  {
    poly_TriangleHandle hts[2];
    poly_VertexHandle   a, b, x, y;
    t_xyz               norm0, norm1;
    //
    return this->CanFlip(he, normDevRad, planeDevRad, checkJacobian, checkWing,
                         hts[0], hts[1], a, b, x, y, norm0, norm1);
  }

  //! Flips the passed edge if it allows flipping.
  //! \param[in] he            the edge to flip.
  //! \param[in] normDevRad    the allowed absolute normal deviation (radians).
  //! \param[in] planeDevRad   the allowed in-plane deviation (radians).
  //! \param[in] checkJacobian the Boolean flag determining whether to check mesh
  //!                          quality metric (scaled Jacobian) on edge flip to
  //!                          ensure that flipping does not make the mesh worse.
  //! \param[in] checkWing     the Boolean flag indicating whether to check the projection
  //!                          of the neighbor links to the edge supposed for flipping.
  //!                          If the dot product is negative, that's a "wing-like" shape
  //!                          that should not undergo edge flipping to avoid overlappings.
  //! \return true if the edge was flipped, false -- otherwise.
  bool
    FlipEdge(const poly_EdgeHandle he,
             const double          normDevRad    = 1./180.*M_PI,
             const double          planeDevRad   = 15./180.*M_PI,
             const bool            checkJacobian = true,
             const bool            checkWing     = true)
  {
    poly_VertexHandle   a, b, x, y;
    poly_TriangleHandle hts[2];
    t_xyz               norm0, norm1;
    //
    if ( !this->CanFlip(he, normDevRad, planeDevRad, checkJacobian, checkWing,
                        hts[0], hts[1], a, b, x, y, norm0, norm1) )
      return false;

    // Get triangles to rotate.
    poly_Triangle<ElemTraits> ts[2];
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

  //! Flips all edges that allow flipping.
  //! \param[in] normDevRad  the allowed absolute normal deviation (radians).
  //! \param[in] planeDevRad the allowed in-plane deviation (radians).
  //! \return the number of flips done.
  int
    FlipEdges(const double normDevRad  = 1./180.*M_PI,
              const double planeDevRad = 15./180.*M_PI)
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

  //! Finds the given edge in the collection of precomputed links.
  //! \param[in] e the edge to find.
  //! \return the edge handle.
  poly_EdgeHandle
    FindEdge(const poly_Edge& e) const
  {
    for ( size_t eidx = 0; eidx < __edges.size(); ++eidx )
    {
      if ( __edges[eidx] == e )
        return poly_EdgeHandle( int(eidx) );
    }

    return poly_EdgeHandle(Mobius_InvalidHandleIndex);
  }

  //! Finds an edge defined by the passed vertex handles (order does not matter)
  //! in the collection of precomputed links.
  //! \param[in] hv0 the first vertex of the edge to find.
  //! \param[in] hv1 the second vertex of the edge to find.
  //! \return the edge handle.
  poly_EdgeHandle
    FindEdge(const poly_VertexHandle& hv0,
             const poly_VertexHandle& hv1) const
  {
    return this->FindEdge( poly_Edge(hv0, hv1) );
  }

  //! Returns the common edge for the passed pair of triangles.
  //! \param[in] ht0 the first triangle.
  //! \param[in] ht1 the second triangle.
  //! \return the handle for the common edge or invalid handle
  //!         if there is no common edge.
  poly_EdgeHandle
    FindEdge(const poly_TriangleHandle ht0,
             const poly_TriangleHandle ht1) const
  {
    poly_Triangle<ElemTraits> t0, t1;
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

  //! Finds all boundary edges in the mesh. A boundary edge is an edge where
  //! the triangles with different `face IDs` meet or an edge not having
  //! exactly two owner triangles.
  //! \param[out] bndEdges the extracted boundary edges.
  //! \param[out] bndTris  the extracted boundary triangles.
  void
    FindBoundaryEdges(std::vector<poly_EdgeHandle>&     bndEdges,
                      std::vector<poly_TriangleHandle>& bndTris) const
  {
    // Extract edges from the computed links.
    for ( const auto& linkIt : __links )
    {
      const poly_EdgeHandle                   he  = linkIt.first;
      const std::vector<poly_TriangleHandle>& hts = linkIt.second;

      // Keep alive triangles only.
      std::vector<poly_TriangleHandle> alive;
      //
      for ( const auto& ht : hts )
      {
        if ( !__triangles[ht.iIdx].IsDeleted() )
          alive.push_back(ht);
      }

      if ( (alive.size() != 2) || (__triangles[alive[0].iIdx].GetFaceRef() != __triangles[alive[1].iIdx].GetFaceRef()) )
      {
        bndEdges.push_back(he);

        // Add boundary triangles to the result.
        for ( const auto& ht : alive )
          bndTris.push_back(ht);
      }
    }
  }

  //! Returns all edges that belong to the specified domain.
  //! \param[in]  domainId   the domain of interest.
  //! \param[out] innerEdges the extracted inner edges.
  //! \param[out] bndEdges   the extracted boundary edges.
  void
    FindDomainEdges(const int                     domainId,
                    std::vector<poly_EdgeHandle>& innerEdges,
                    std::vector<poly_EdgeHandle>& bndEdges) const
  {
    // Extract edges from the computed links.
    for ( const auto& linkIt : __links )
    {
      const poly_EdgeHandle                   he  = linkIt.first;
      const std::vector<poly_TriangleHandle>& hts = linkIt.second;

      // Count the in-domain triangles.
      int numDomainTris = 0;
      //
      for ( const auto& ht : hts )
      {
        if ( __triangles[ht.iIdx].IsDeleted() )
          continue;

        if ( __triangles[ht.iIdx].GetFaceRef() == domainId )
          numDomainTris++;
      }

      if ( numDomainTris == 1 )
        bndEdges.push_back(he);
      else if ( numDomainTris == 2 )
        innerEdges.push_back(he);
    }
  }

  //! Finds the vertex shared between the passed triangle and the
  //! given (presumably dangling) edge.
  //! \param[in]  ht   the triangle in question.
  //! \param[in]  he   the edge that presumably shares a single vertex
  //!                  the triangle `ht`.
  //! \param[out] vidx the index of the common vertex in range [0,2].
  //! \return the shared vertex or an invalid handle if there is no
  //!         such a vertex.
  poly_VertexHandle
    FindVertex(const poly_TriangleHandle ht,
               const poly_EdgeHandle     he,
               int&                      vidx) const
  {
    poly_Triangle<ElemTraits> t;
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

  //! Collapses the passed edge.
  //! \param[in] he            the edge to collapse.
  //! \param[in] checkBorderOn the Boolean flag indicating whether to check borders
  //!                          on edge collapse.
  //! \param[in] checkDegenOn  the Boolean flag indicating whether to check degeneracy
  //!                          of the resulting triangles.
  //! \param[in] prec          the precision to use for the degenracy check, if it is
  //!                          enabled.
  //! \param[in] domain        the optional CAD domain (a set of face IDs) to restrict
  //!                          edge collapse in.
  //! \return true in case of success, false -- otherwise.
  bool
    CollapseEdge(const poly_EdgeHandle          he,
                 const bool                     checkBorderOn = true,
                 const bool                     checkDegenOn  = true,
                 const double                   prec          = core_Precision::Resolution3D(),
                 const std::unordered_set<int>& domain        = std::unordered_set<int>())
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

          poly_Triangle<ElemTraits> t2Check;
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
          poly_Triangle<ElemTraits> t2Check;
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
      poly_Triangle<ElemTraits> t2Remove;
      this->GetTriangle(ht2Remove, t2Remove);

      // Get the vertex to survive (the one opposite to the collapsed edge).
      const poly_VertexHandle a = this->GetOppositeVertex(ht2Remove, he);

      // Add new edge.
      const poly_EdgeHandle sharedEdgeHandle = this->AddEdge(a, hVm);

      // Edit neighbor triangles.
      for ( const auto& th2Edit : ths2Edit )
      {
        // Skip the triangles that are supposed to be removed.
        if ( hts2Remove.find(th2Edit) != hts2Remove.end() )
          continue;

        poly_Triangle<ElemTraits>& t2Edit = this->ChangeTriangle(th2Edit);

        // Find a vertex `c` shared by a triangle to edit and the edge
        // being collapsed. That is basically the vertex where the
        // dangling edge `he` is attached to the target triangle.
        int ci = -1;
        const poly_VertexHandle c = this->FindVertex(th2Edit, he, ci);
        //
        if ( !c.IsValid() )
          continue;

        // Move the vertex `c` using the non-const reference to the triangle.
        t2Edit.hVertices[ci] = hVm;

        // Edit the edges of the modified triangle.
        for ( int ei = 0; ei < 3; ++ei )
        {
          poly_Edge& e2Edit = __edges[t2Edit.hEdges[ei].iIdx];

          // Check if this edge is going to move.
          int  vi              = -1;
          int  numVertAffected = 0;
          //
          for ( int k = 0; k < 3; ++k )
          {
            if ( e2Edit.hVertices[0] == t2Remove.hVertices[k] )
            {
              numVertAffected++;
              vi = 0;
            }

            if ( e2Edit.hVertices[1] == t2Remove.hVertices[k] )
            {
              numVertAffected++;
              vi = 1;
            }
          }

          // Edit edge.
          if ( numVertAffected == 1 )
          {
            e2Edit.hVertices[vi] = hVm;
          }

          // Substitute the edge with the shared one.
          if ( numVertAffected == 2 )
          {
            t2Edit.hEdges[ei] = sharedEdgeHandle;
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

  //! Splits the passed edge and refines its owner triangles.
  //! \param[in] he the edge to split.
  //! \return true in the case of success, false -- otherwise.
  bool SplitEdge(const poly_EdgeHandle he)
  {
    // Get the edge to split.
    poly_Edge e;
    if ( !this->GetEdge(he, e) )
      return false;

    // Get the neighbor triangles.
    std::vector<poly_TriangleHandle> hts;
    if ( !this->FindAdjacent(he, hts) )
      return false;

    // Check that we're not touching corners (imagine a box and the
    // edges at corners) as edge split is not going to be valid there.
    for ( int k = 0; k < 2; ++k )
    {
      // Find neighbors.
      bool                                  isBoundary = false;
      std::unordered_set<poly_VertexHandle> adjVerts;
      std::unordered_set<int>               faceRefs;
      std::unordered_set<int>               domain;
      //
      this->FindAdjacent(e.hVertices[0], adjVerts, isBoundary, faceRefs, domain);

      if ( faceRefs.size() > 2 )
        return false;
    }

    const poly_VertexHandle x = e.hVertices[0];
    const poly_VertexHandle y = e.hVertices[1];

    // Get new vertex position.
    t_xyz V[2];
    this->GetVertex(x, V[0]);
    this->GetVertex(y, V[1]);
    //
    t_xyz Vm = (V[0] + V[1])*0.5;

    // Add the new vertex.
    const poly_VertexHandle hVm = this->AddVertex(Vm);

    // Add edges.
    poly_EdgeHandle exm = this->AddEdge(x, hVm);
    poly_EdgeHandle eym = this->AddEdge(hVm, y);

    // New triangles sharing x, y.
    std::vector<poly_TriangleHandle> xths, yths;

    // Split the opposite triangles.
    for ( const auto& ht : hts )
    {
      const poly_VertexHandle hOppVert = this->GetOppositeVertex(ht, he);
      //
      if ( !hOppVert.IsValid() )
        continue;

      // Get triangle.
      poly_Triangle<ElemTraits> t;
     //
      if ( !this->GetTriangle(ht, t) || t.IsDeleted() )
        continue;

      /*          hOppVert
                 o
                 | \
                 |  \
                 |   \
              ex |    \ ey
                 |     \
                 |  eh  \
               x o---o---o y
                     hVm
      */

      // Find the existing edges.
      poly_EdgeHandle ex, ey;
      //
      for ( int j = 0; j < 3; ++j )
      {
        const poly_Edge& te = __edges[t.hEdges[j].iIdx];

        if ( ((te.hVertices[0] == x)        && (te.hVertices[1] == hOppVert)) ||
             ((te.hVertices[0] == hOppVert) && (te.hVertices[1] == x)) )
        {
          ex = t.hEdges[j];
        }

        if ( ((te.hVertices[0] == y)        && (te.hVertices[1] == hOppVert)) ||
             ((te.hVertices[0] == hOppVert) && (te.hVertices[1] == y)) )
        {
          ey = t.hEdges[j];
        }
      }

      // Add split edge.
      poly_EdgeHandle vertEdgeHandle = this->AddEdge(hVm, hOppVert);

      // Compute the reference normal to control the split.
      t_xyz refNorm;
      this->ComputeNormal(x, y, hOppVert, refNorm);

      // Compute norms to preserve orientations.
      t_xyz testNorm[2];
      this->ComputeNormal(x, hVm, hOppVert, testNorm[0]);
      this->ComputeNormal(hVm, y, hOppVert, testNorm[1]);

      // Split triangle.
      poly_TriangleHandle splitHts[2];
      //
      if ( testNorm[0].Dot(refNorm) > 0 )
      {
        splitHts[0] = this->AddTriangle( x, hVm, hOppVert, t.GetFaceRef() );
      }
      else
      {
        splitHts[0] = this->AddTriangle( hVm, x, hOppVert, t.GetFaceRef() );
      }
      //
      if ( testNorm[1].Dot(refNorm) > 0 )
      {
        splitHts[1] = this->AddTriangle( hVm, y, hOppVert, t.GetFaceRef() );
      }
      else
      {
        splitHts[1] = this->AddTriangle( y, hVm, hOppVert, t.GetFaceRef() );
      }

      // Add back references to the edges.
      __triangles[splitHts[0].iIdx].hEdges[0] = ex;
      __triangles[splitHts[0].iIdx].hEdges[1] = exm;
      __triangles[splitHts[0].iIdx].hEdges[2] = vertEdgeHandle;
      //
      __triangles[splitHts[1].iIdx].hEdges[0] = ey;
      __triangles[splitHts[1].iIdx].hEdges[1] = eym;
      __triangles[splitHts[1].iIdx].hEdges[2] = vertEdgeHandle;

      xths.push_back(splitHts[0]);
      yths.push_back(splitHts[1]);

      // Remove the original triangle.
      this->RemoveTriangle(ht);
    }

    // Insert new links.
    __links.insert({exm, xths});
    __links.insert({eym, yths});

    // Remove edge.
    this->RemoveEdge(he);

    return true;
  }

  //! Applies Laplacian smoothing to the mesh vertices.
  //! \param[in] iter       the number of smoothing steps.
  //! \param[in] tag        the subdomain ID to smooth.
  //! \param[in] checkInter the Boolean flag indicating whether to check for self-intersecting links.
  void Smooth(const int  iter,
              const int  tag,
              const bool checkInter)
  {
    std::unordered_set<int> domain = {tag};

    // Find adjacent vertices for each vertex.
    std::unordered_map< poly_VertexHandle, std::unordered_set<poly_VertexHandle> > adj;
    //
    for ( VertexIterator vit(this); vit.More(); vit.Next() )
    {
      const poly_VertexHandle vh = vit.Current();

      // Find neighbors.
      bool                                  isBoundary = false;
      std::unordered_set<poly_VertexHandle> adjVerts;
      std::unordered_set<int>               faceRefs;
      //
      this->FindAdjacent(vh, adjVerts, isBoundary, faceRefs, domain);

      if ( !isBoundary )
        adj.insert({vh, adjVerts});
    }

    int it = 0;
    std::unordered_map<poly_VertexHandle, t_xyz> origCoordsMap;

    // Move each vertex.
    while ( it++ < iter )
    {
      for ( const auto& vTuple : adj )
      {
        // Get the original coordinates.
        t_xyz origCoords;
        this->GetVertex(vTuple.first, origCoords);
        //
        if ( origCoordsMap.find(vTuple.first) == origCoordsMap.end() )
          origCoordsMap.insert({vTuple.first, origCoords});

        // Compute the new (average) position.
        t_xyz avrg;
        int   num = 0;
        //
        for ( const auto& nvs : vTuple.second )
        {
          t_xyz coord;
          this->GetVertex(nvs, coord);

          avrg += coord;
          ++num;
        }
        //
        if ( !num )
          continue;
        //
        avrg /= num;

        // Move vertex.
        this->ChangeVertex(vTuple.first).ChangeCoords() = avrg;
      }
    }

    // Check for possible intersections.
    if ( checkInter )
    {
      // Get all edge handles for intersection test.
      std::vector<poly_EdgeHandle> innerEdges, bndEdges;
      this->FindDomainEdges(tag, innerEdges, bndEdges);

      if ( this->AreSelfIntersecting(tag, innerEdges, bndEdges) )
      {
        // Restore the original positions.
        for ( const auto& tuple : origCoordsMap )
          this->ChangeVertex(tuple.first).ChangeCoords() = tuple.second;
      }
    }
  }

  //! Applies Laplacian smoothing to the mesh vertices.
  //! \param[in] iter       the number of smoothing steps.
  //! \param[in] domain     the optional face IDs to smooth.
  //! \param[in] checkInter the Boolean flag indicating whether to check for self-intersecting links.
  void
    Smooth(const int                      iter       = 1,
           const std::unordered_set<int>& domain     = std::unordered_set<int>(),
           const bool                     checkInter = false)
  {
    for ( auto tag : domain )
      this->Smooth(iter, tag, checkInter);
  }

public:

  //! Returns vertex by its handle.
  //! \param[in]  h      handle of a vertex to access.
  //! \param[out] vertex vertex.
  //! \return false if there is no such vertex.
  bool GetVertex(const poly_VertexHandle h,
                 poly_Vertex&            vertex) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( __vertices.size() ) ) return false;
    //
    vertex = __vertices[idx];
    return true;
  }

  //! Returns vertex by its handle as a coordinate triple.
  //! \param[in]  h      handle of a vertex to access.
  //! \param[out] vertex vertex.
  //! \return false if there is no such vertex.
  bool GetVertex(const poly_VertexHandle h,
                 t_xyz&                  vertex) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( __vertices.size() ) ) return false;
    //
    vertex = __vertices[idx];
    return true;
  }

  //! Returns non-const reference to a vertex by its handle.
  //! You have to be sure that such a vertex exists.
  //! \param[in] h handle of a vertex to access.
  //! \return non-const reference that you can use to edit.
  poly_Vertex& ChangeVertex(const poly_VertexHandle h)
  {
    return __vertices[h.iIdx];
  }

  //! Returns edge by its handle.
  //! \param[in]  h    handle of an edge to access.
  //! \param[out] edge edge.
  //! \return false if there is no such edge.
  bool GetEdge(const poly_EdgeHandle h,
               poly_Edge&            edge) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( __edges.size() ) ) return false;
    //
    edge = __edges[idx];
    return true;
  }

  //! Returns triangle by its handle.
  //! \param[in]  h        handle of a triangle to access.
  //! \param[out] triangle triangle.
  //! \return false if there is no such triangle.
  bool GetTriangle(const poly_TriangleHandle  h,
                   poly_Triangle<ElemTraits>& triangle) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( __triangles.size() ) ) return false;
    //
    triangle = __triangles[idx];
    return true;
  }

  //! Returns non-const reference to a triangle by its handle.
  //! You have to be sure that such a triangle exists.
  //! \param[in] h handle of a triangle to access.
  //! \return non-const reference that you can use to edit.
  poly_Triangle<ElemTraits>& ChangeTriangle(const poly_TriangleHandle h)
  {
    return __triangles[h.iIdx];
  }

  //! Returns quad by its handle.
  //! \param[in]  h    handle of a quad to access.
  //! \param[out] quad quad.
  //! \return false if there is no such quad.
  bool GetQuad(const poly_QuadHandle h,
               poly_Quad&            quad) const
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( __quads.size() ) ) return false;
    //
    quad = __quads[idx];
    return true;
  }

  //! Creates a new vertex and returns its handle.
  //! \return handle of the just added vertex.
  poly_VertexHandle AddVertex()
  {
    return this->AddVertex( core_XYZ::O() );
  }

  //! Creates a new vertex with the given coordinates and returns its handle.
  //! \param[in] x coordinate x of the vertex.
  //! \param[in] y coordinate y of the vertex.
  //! \param[in] z coordinate z of the vertex.
  //! \return handle of the just added vertex.
  poly_VertexHandle AddVertex(const double x,
                              const double y,
                              const double z)
  {
    return this->AddVertex( core_XYZ(x, y, z) );
  }

  //! Creates a new vertex and returns its handle.
  //! \param[in] coords coordinates of the vertex.
  //! \return handle of the just added vertex.
  poly_VertexHandle AddVertex(const core_XYZ& coords)
  {
    __vertices.push_back( poly_Vertex(coords) );
    poly_VertexHandle hVertex( int( __vertices.size() ) - 1 );
    return hVertex;
  }

  //! Creates a new invalid edge.
  //! \return handle of the just added edge.
  poly_EdgeHandle AddEdge()
  {
    poly_VertexHandle inv;
    return this->AddEdge(inv, inv);
  }

  //! Creates a new edge between the passed two vertices.
  //! \param[in] hStartV  start vertex.
  //! \param[in] hFinishV finish vertex.
  //! \return handle of the just added edge.
  poly_EdgeHandle AddEdge(const poly_VertexHandle hStartV,
                          const poly_VertexHandle hFinishV)
  {
    __edges.push_back( poly_Edge(hStartV, hFinishV) );
    poly_EdgeHandle hEdge( int( __edges.size() ) - 1 );
    return hEdge;
  }

  //! Adds the passed edge to the mesh.
  //! \return handle of the just added edge.
  poly_EdgeHandle AddEdge(const poly_Edge& edge)
  {
    __edges.push_back(edge);
    poly_EdgeHandle hEdge( int( __edges.size() ) - 1 );
    return hEdge;
  }

  //! Removes the edge with the given handle.
  //! \param[in] h handle of the edge to remove.
  //! \return true in case of success, false -- otherwise.
  bool RemoveEdge(const poly_EdgeHandle h)
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( __edges.size() ) ) return false;

    __edges[idx].SetDeleted();
    return true;
  }

  //! Creates a new invalid triangle.
  //! \return handle of the just added triangle.
  poly_TriangleHandle AddTriangle()
  {
    poly_VertexHandle inv;
    return this->AddTriangle(inv, inv, inv);
  }

  //! Creates a new triangle.
  //! \param[in] hV0 1-st vertex.
  //! \param[in] hV1 2-nd vertex.
  //! \param[in] hV2 3-rd vertex.
  //! \return handle of the just added triangle.
  poly_TriangleHandle AddTriangle(const poly_VertexHandle hV0,
                                  const poly_VertexHandle hV1,
                                  const poly_VertexHandle hV2)
  {
    __triangles.push_back( poly_Triangle<ElemTraits>(hV0, hV1, hV2) );
    poly_TriangleHandle hTriangle( int( __triangles.size() ) - 1 );

    // Add back references to the vertices.
    __vertices[hV0.iIdx].AddTriangleRef(hTriangle);
    __vertices[hV1.iIdx].AddTriangleRef(hTriangle);
    __vertices[hV2.iIdx].AddTriangleRef(hTriangle);

    return hTriangle;
  }

  //! Creates a new triangle with a back reference to a CAD face.
  //! \param[in] hV0 1-st vertex.
  //! \param[in] hV1 2-nd vertex.
  //! \param[in] hV2 3-rd vertex.
  //! \param[in] ref back-reference index.
  //! \return handle of the just added triangle.
  poly_TriangleHandle AddTriangle(const poly_VertexHandle hV0,
                                  const poly_VertexHandle hV1,
                                  const poly_VertexHandle hV2,
                                  const int               ref)
  {
    __triangles.push_back( poly_Triangle<ElemTraits>(hV0, hV1, hV2, ref) );
    poly_TriangleHandle hTriangle( int( __triangles.size() ) - 1 );

    // Add back references to the vertices.
    __vertices[hV0.iIdx].AddTriangleRef(hTriangle);
    __vertices[hV1.iIdx].AddTriangleRef(hTriangle);
    __vertices[hV2.iIdx].AddTriangleRef(hTriangle);

    return hTriangle;
  }

  //! Removes the triangle with the given handle.
  //! \param[in] h handle of the triangle to remove.
  //! \return true in case of success, false -- otherwise.
  bool RemoveTriangle(const poly_TriangleHandle h)
  {
    const int idx = h.GetIdx();
    if ( idx < 0 || idx > int( __triangles.size() ) ) return false;

    __triangles[idx].SetDeleted();
    return true;
  }

  //! Creates a new invalid quad.
  //! \return handle of the just added quad.
  poly_QuadHandle AddQuad()
  {
    poly_VertexHandle inv;
    return this->AddQuad(inv, inv, inv, inv);
  }

  //! Creates a new quad.
  //! \param[in] hV0 1-st vertex.
  //! \param[in] hV1 2-nd vertex.
  //! \param[in] hV2 3-rd vertex.
  //! \param[in] hV3 4-th vertex.
  //! \return handle of the just added quad.
  poly_QuadHandle AddQuad(const poly_VertexHandle hV0,
                          const poly_VertexHandle hV1,
                          const poly_VertexHandle hV2,
                          const poly_VertexHandle hV3)
  {
    __quads.push_back( poly_Quad(hV0, hV1, hV2, hV3) );
    poly_QuadHandle hQuad( int( __quads.size() ) - 1 );
    return hQuad;
  }

  //! \return number of vertices.
  int GetNumVertices() const
  {
    return int( __vertices.size() );
  }

  //! \return number of edges.
  int GetNumEdges() const
  {
    return int( __edges.size() );
  }

  //! Returns the number of triangles.
  //! \param[in] includeDeads whether to include dead ones.
  //! \return number of triangles.
  int GetNumTriangles(const bool includeDeads = false) const
  {
    if ( includeDeads )
      return int( __triangles.size() );

    int count = 0;
    for ( const auto& tri : __triangles )
      if ( !tri.IsDeleted() )
        count++;

    return count;
  }

  //! \return number of quads.
  int GetNumQuads() const
  {
    return int( __quads.size() );
  }

public:

  //! Iterator for vertices.
  class VertexIterator
  {
  public:

    //! Ctor accepting mesh.
    VertexIterator(const t_ptr<poly_Mesh<ElemTraits>>& mesh) : m_mesh(mesh), m_pos(0) {}

  public:

    bool More() const { return m_pos < m_mesh->__vertices.size(); }
    void Next() { m_pos++; }
    poly_VertexHandle Current() const { return poly_VertexHandle( int(m_pos) ); }

  private:

    t_ptr<poly_Mesh<ElemTraits>> m_mesh; //!< Mesh to iterate.
    size_t                       m_pos;  //!< Current position.
  };

  //! Iterator for edges.
  class EdgeIterator
  {
  public:

    //! Ctor accepting mesh.
    EdgeIterator(const t_ptr<poly_Mesh<ElemTraits>>& mesh) : m_mesh(mesh), m_pos(0) {}

  public:

    bool More() const { return m_pos < m_mesh->__edges.size(); }
    void Next() { m_pos++; }
    poly_EdgeHandle Current() const { return poly_EdgeHandle( int(m_pos) ); }

  private:

    t_ptr<poly_Mesh<ElemTraits>> m_mesh; //!< Mesh to iterate.
    size_t                       m_pos;  //!< Current position.
  };

  //! Iterator for triangles.
  class TriangleIterator
  {
  public:

    //! Ctor accepting mesh.
    TriangleIterator(const t_ptr<poly_Mesh<ElemTraits>>& mesh) : m_mesh(mesh), m_pos(0) {}

  public:

    bool More() const { return m_pos < m_mesh->__triangles.size(); }
    void Next() { m_pos++; }
    poly_TriangleHandle Current() const { return poly_TriangleHandle( int(m_pos) ); }

  private:

    t_ptr<poly_Mesh<ElemTraits>> m_mesh; //!< Mesh to iterate.
    size_t                       m_pos;  //!< Current position.
  };

  //! Iterator for quads.
  class QuadIterator
  {
  public:

    //! Ctor accepting mesh.
    QuadIterator(const t_ptr<poly_Mesh<ElemTraits>>& mesh) : m_mesh(mesh), m_pos(0) {}

  public:

    bool More() const { return m_pos < m_mesh->__quads.size(); }
    void Next() { m_pos++; }
    poly_QuadHandle Current() const { return poly_QuadHandle( int(m_pos) ); }

  private:

    t_ptr<poly_Mesh<ElemTraits>> m_mesh; //!< Mesh to iterate.
    size_t                       m_pos;  //!< Current position.
  };

protected:

  //! Updates the existing link by exchanging `htOld` with `htNew`.
  void
    updateLink(const poly_EdgeHandle     he,
               const poly_TriangleHandle htOld,
               const poly_TriangleHandle htNew)
  {
    auto link = __links.find(he);
    //
    if ( link == __links.end() )
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
    __links[he] = newTuple;
  }

public:

  std::vector<poly_Vertex>               __vertices;  //!< List of vertices.
  std::vector<poly_Edge>                 __edges;     //!< List of edges.
  std::vector<poly_Triangle<ElemTraits>> __triangles; //!< List of triangles.
  std::vector<poly_Quad>                 __quads;     //!< List of quads.

  //! Edges-to-triangles map.
  std::unordered_map< poly_EdgeHandle, std::vector<poly_TriangleHandle> > __links;

  //! CAD surface adapter.
  t_ptr<poly_SurfAdapter> __surfAdt;

};

//! Convenience shortcut for mesh with default traits.
typedef poly_Mesh<> t_mesh;

}

#endif
