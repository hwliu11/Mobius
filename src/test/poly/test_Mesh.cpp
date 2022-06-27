//-----------------------------------------------------------------------------
// Created on: 18 September 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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
#include <mobius/test_Mesh.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Poly includes
#include <mobius/poly_ReadOBJ.h>
#include <mobius/poly_ReadPLY.h>
#include <mobius/poly_ReadSTL.h>

//-----------------------------------------------------------------------------

// Filenames are specified relatively to MOBIUS_TEST_DATA environment variable.
#define filename_mesh_001 "mesh/mesh_001.stl"
#define filename_mesh_002 "mesh/mesh_002.stl"
#define filename_mesh_003 "mesh/mesh_003.stl"
#define filename_mesh_004 "mesh/mesh_004_binary.stl"
#define filename_mesh_005 "mesh/plate-with-quads_001.ply"
#define filename_mesh_006 "mesh/plate-with-quads_002.ply"
#define filename_mesh_007 "mesh/mesh_flip-edge_01.stl"
#define filename_mesh_008 "mesh/mesh_flip-edge_02.stl"
#define filename_mesh_009 "mesh/mesh_005.stl"
#define filename_mesh_010 "mesh/mesh_006.stl"
#define filename_mesh_011 "mesh/grabcad_obj1.obj"

//-----------------------------------------------------------------------------

bool mobius::test_Mesh::verifyMeshContents(const t_ptr<poly_Mesh>& mesh,
                                           const int               refNumVertices,
                                           const int               refNumEdges,
                                           const int               refNumTriangles,
                                           const int               refNumQuads)
{
  // Get the actual summary.
  const int numVertices  = mesh->GetNumVertices();
  const int numEdges     = mesh->GetNumEdges();
  const int numTriangles = mesh->GetNumTriangles();
  const int numQuads     = mesh->GetNumQuads();

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // Verify.
  if ( refNumVertices != numVertices )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Unexpected number of vertices (%1 expected, %2 actual)."
                                                          << refNumVertices << numVertices);
    return false;
  }
  //
  if ( refNumEdges != numEdges )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Unexpected number of edges (%1 expected, %2 actual)."
                                                          << refNumEdges << numEdges);
    return false;
  }
  //
  if ( refNumTriangles != numTriangles )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Unexpected number of triangles (%1 expected, %2 actual)."
                                                          << refNumTriangles << numTriangles);
    return false;
  }
  //
  if ( refNumQuads != numQuads )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Unexpected number of quads (%1 expected, %2 actual)."
                                                          << refNumQuads << numQuads);
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------

mobius::t_ptr<mobius::poly_Mesh>
  mobius::test_Mesh::readSTL(const char* filenameShort)
{
  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + filenameShort;

  // Prepare reader.
  poly_ReadSTL reader(cf->ProgressNotifier, nullptr);

  // Read STL.
  if ( !reader.Perform(filename) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "STL reader returned false.");
    return nullptr;
  }

  return reader.GetResult();
}

//-----------------------------------------------------------------------------

//! Common function to test STL reader.
mobius::outcome
  mobius::test_Mesh::testReadSTL(const int   funcID,
                                 const char* filenameShort,
                                 const int   refNumVertices,
                                 const int   refNumEdges,
                                 const int   refNumTriangles)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = readSTL(filenameShort);
  //
  if ( mesh.IsNull() )
    return res.failure();

  // Verify.
  const bool isOk = verifyMeshContents(mesh,
                                       refNumVertices,
                                       refNumEdges,
                                       refNumTriangles,
                                       0);

  return res.status(isOk);
}

//-----------------------------------------------------------------------------

//! Common function to test PLY reader.
mobius::outcome
  mobius::test_Mesh::testReadPLY(const int   funcID,
                                 const char* filenameShort,
                                 const int   refNumVertices,
                                 const int   refNumEdges,
                                 const int   refNumTriangles,
                                 const int   refNumQuads)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + filenameShort;

  // Prepare reader.
  poly_ReadPLY reader(cf->ProgressNotifier, nullptr);

  // Read PLY.
  if ( !reader.Perform(filename) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "PLY reader returned false.");
    return res.failure();
  }

  // Verify.
  const bool isOk = verifyMeshContents(reader.GetResult(),
                                       refNumVertices,
                                       refNumEdges,
                                       refNumTriangles,
                                       refNumQuads);

  return res.status(isOk);
}

//-----------------------------------------------------------------------------

//! Common function to test OBJ reader.
mobius::outcome
  mobius::test_Mesh::testReadOBJ(const int   funcID,
                                 const char* filenameShort,
                                 const int   refNumVertices,
                                 const int   refNumEdges,
                                 const int   refNumTriangles,
                                 const int   refNumQuads)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + filenameShort;

  // Prepare reader.
  poly_ReadOBJ reader(cf->ProgressNotifier, nullptr);

  // Read OBJ.
  if ( !reader.Perform(filename) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "OBJ reader returned false.");
    return res.failure();
  }

  // Verify.
  const bool isOk = verifyMeshContents(reader.GetResult(),
                                       refNumVertices,
                                       refNumEdges,
                                       refNumTriangles,
                                       refNumQuads);

  return res.status(isOk);
}

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::testCreateVertex(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = new poly_Mesh;

  // Add vertices and validate the returned handles.
  poly_VertexHandle hv0 = mesh->AddVertex();
  poly_VertexHandle hv1 = mesh->AddVertex();
  poly_VertexHandle hv2 = mesh->AddVertex();
  //
  if ( hv0.GetIdx() != 0 )
    return res.failure();
  //
  if ( hv1.GetIdx() != 1 )
    return res.failure();
  //
  if ( hv2.GetIdx() != 2 )
    return res.failure();

  // Get any vertex to check.
  poly_Vertex v0;
  //
  if ( !mesh->GetVertex(hv0, v0) )
    return res.failure();
  //
  if ( v0.X() != 0. )
    return res.failure();
  //
  if ( v0.Y() != 0. )
    return res.failure();
  //
  if ( v0.Z() != 0. )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario for reading STL.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::testReadSTL01(const int funcID)
{
  return testReadSTL(funcID,
                     filename_mesh_001,
                     1807, 0, 3609);
}

//-----------------------------------------------------------------------------

//! Test scenario for reading STL.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::testReadSTL02(const int funcID)
{
  return testReadSTL(funcID,
                     filename_mesh_002,
                     1620, 0, 3236);
}

//-----------------------------------------------------------------------------

//! Test scenario for reading STL.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::testReadSTL03(const int funcID)
{
  return testReadSTL(funcID,
                     filename_mesh_003,
                     2109, 0, 4199);
}

//-----------------------------------------------------------------------------

//! Test scenario for reading STL.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::testReadSTL04(const int funcID)
{
  return testReadSTL(funcID,
                     filename_mesh_004,
                     17379, 0, 34838);
}

//-----------------------------------------------------------------------------

//! Test scenario for reading PLY.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::testReadPLY01(const int funcID)
{
  return testReadPLY(funcID,
                     filename_mesh_005,
                     3274, 0, 662, 2842);
}

//-----------------------------------------------------------------------------

//! Test scenario for reading PLY.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::testReadPLY02(const int funcID)
{
  return testReadPLY(funcID,
                     filename_mesh_006,
                     1443, 0, 12, 1325);
}

//-----------------------------------------------------------------------------

//! Test scenario for reading OBJ.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::testReadOBJ01(const int funcID)
{
  return testReadOBJ(funcID,
                     filename_mesh_011,
                     9989, 0, 176, 9648);
}

//-----------------------------------------------------------------------------

//! Tests midpoint refinement.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::refineByMidpoint(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = new poly_Mesh;

  // Add vertices.
  poly_VertexHandle hv0 = mesh->AddVertex(0., 0., 0.);
  poly_VertexHandle hv1 = mesh->AddVertex(1., 0., 0.);
  poly_VertexHandle hv2 = mesh->AddVertex(0., 1., 0.);

  // Add triangle.
  poly_TriangleHandle ht = mesh->AddTriangle(hv0, hv1, hv2);

  // Refine triangle.
  if ( !mesh->RefineByMidpoint(ht) )
  {
    return res.failure();
  }

  // Validate the number of vertices.
  if ( mesh->GetNumVertices() != 4 )
  {
    return res.failure();
  }

  // Validate the number of triangles: one "dead" is there.
  if ( mesh->GetNumTriangles(true) != 4 )
  {
    return res.failure();
  }

  // Verify the "is deleted" flag.
  poly_Triangle t;
  if ( !mesh->GetTriangle(ht, t) )
  {
    return res.failure();
  }
  //
  if ( !t.IsDeleted() )
  {
    return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests the computation of links.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::computeLinks(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = new poly_Mesh;

  // Add vertices.
  poly_VertexHandle hv0 = mesh->AddVertex(0., 0., 0.);
  poly_VertexHandle hv1 = mesh->AddVertex(1., 0., 0.);
  poly_VertexHandle hv2 = mesh->AddVertex(0., 1., 0.);
  poly_VertexHandle hv3 = mesh->AddVertex(1., 1., 0.);

  // Add triangle.
  poly_TriangleHandle ht0 = mesh->AddTriangle(hv0, hv1, hv2);
  poly_TriangleHandle ht1 = mesh->AddTriangle(hv2, hv1, hv3);

  // Compute links.
  mesh->ComputeEdges();

  // One edge is shared.
  if ( mesh->GetNumEdges() != 5 )
    return res.failure();

  // Get triangles.
  std::vector<size_t> valences = {1, 2, 1, 1, 1};
  for ( poly_Mesh::EdgeIterator eit(mesh); eit.More(); eit.Next() )
  {
    const poly_EdgeHandle eh = eit.Current();

    std::vector<poly_TriangleHandle> hts;
    if ( !mesh->GetTriangles(eh, hts) )
      return res.failure();

    if ( hts.size() != valences[eh.GetIdx()] )
      return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests edge flipping.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::flipEdges01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = new poly_Mesh;

  // Add vertices.
  poly_VertexHandle hv_a = mesh->AddVertex(0., 0., 0.);
  poly_VertexHandle hv_x = mesh->AddVertex(1., 0., 0.);
  poly_VertexHandle hv_y = mesh->AddVertex(0., 1., 0.);
  poly_VertexHandle hv_b = mesh->AddVertex(1., 1., 0.);

  // Add triangle.
  poly_TriangleHandle ht0 = mesh->AddTriangle(hv_a, hv_x, hv_y);
  poly_TriangleHandle ht1 = mesh->AddTriangle(hv_x, hv_b, hv_y);

  // Compute links.
  mesh->ComputeEdges();

  // Flip edges.
  const int nbFlips = mesh->FlipEdges();
  //
  if ( nbFlips != 1 )
    return res.failure();

  // Check the number of triangles.
  const int nbTris = mesh->GetNumTriangles();
  //
  if ( nbTris != 2 )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests edge flipping.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::flipEdges02(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = new poly_Mesh;

  // Add vertices.
  poly_VertexHandle hv_a = mesh->AddVertex(-1.0, -1.0, 0.0);
  poly_VertexHandle hv_x = mesh->AddVertex( 0.0,  0.0, 0.0);
  poly_VertexHandle hv_y = mesh->AddVertex( 1.0,  0.0, 0.0);
  poly_VertexHandle hv_b = mesh->AddVertex(-1.0,  1.0, 0.0);

  // Add triangle.
  poly_TriangleHandle ht0 = mesh->AddTriangle(hv_a, hv_x, hv_y);
  poly_TriangleHandle ht1 = mesh->AddTriangle(hv_x, hv_b, hv_y);

  // Compute links.
  mesh->ComputeEdges();

  // Get edge that is impossible to flip.
  poly_EdgeHandle eh = mesh->FindEdge( poly_Edge(hv_x, hv_y) );
  //
  if ( eh.GetIdx() == Mobius_InvalidHandleIndex)
    return res.failure();

  if ( mesh->CanFlip(eh) )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests edge flipping.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::flipEdges03(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = new poly_Mesh;

  // Add vertices so that two triangles lie on the same straight line.
  poly_VertexHandle hv_a = mesh->AddVertex(-1.0, 0.0, 0.0);
  poly_VertexHandle hv_x = mesh->AddVertex( 0.0, 0.0, 0.0);
  poly_VertexHandle hv_y = mesh->AddVertex( 0.0, 1.0, 0.0);
  poly_VertexHandle hv_b = mesh->AddVertex( 1.0, 0.0, 0.0);

  // Add triangle.
  poly_TriangleHandle ht0 = mesh->AddTriangle(hv_a, hv_x, hv_y);
  poly_TriangleHandle ht1 = mesh->AddTriangle(hv_x, hv_b, hv_y);

  // Compute links.
  mesh->ComputeEdges();

  // Get edge that is impossible to flip.
  poly_EdgeHandle eh = mesh->FindEdge( poly_Edge(hv_x, hv_y) );
  //
  if ( eh.GetIdx() == Mobius_InvalidHandleIndex)
    return res.failure();

  if ( mesh->CanFlip(eh) )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests edge flipping.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::flipEdges04(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = readSTL(filename_mesh_007);
  //
  if ( mesh.IsNull() )
    return res.failure();

  // Flip all edges. The problematic one should have been skipped.
  mesh->ComputeEdges();
  const int nbFlips = mesh->FlipEdges();
  //
  if ( nbFlips )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests edge flipping.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::flipEdges05(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = readSTL(filename_mesh_008);
  //
  if ( mesh.IsNull() )
    return res.failure();

  t_xyz refNorm(0, 0, 1);

  mesh->ComputeEdges();
  mesh->FlipEdges();

  for ( poly_Mesh::TriangleIterator tit(mesh); tit.More(); tit.Next() )
  {
    t_xyz currNorm;
    mesh->ComputeNormal(tit.Current(), currNorm);

    if ( currNorm.Angle(refNorm) > core_Precision::Resolution3D() )
      return res.failure();
  }

  mesh->ComputeEdges();
  mesh->FlipEdges();

  for ( poly_Mesh::TriangleIterator tit(mesh); tit.More(); tit.Next() )
  {
    t_xyz currNorm;
    mesh->ComputeNormal(tit.Current(), currNorm);

    if ( currNorm.Angle(refNorm) > core_Precision::Resolution3D() )
      return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests finding adjacent triangles.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::findAdjacent01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = readSTL(filename_mesh_009);
  //
  if ( mesh.IsNull() )
    return res.failure();

  std::vector<poly_TriangleHandle> ths;

  // Find adjacent triangles.
  mesh->ComputeEdges();
  mesh->FindAdjacentByEdges(poly_TriangleHandle(1), ths);
  //
  if ( ths.size() != 3 )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests finding adjacent triangles.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::findAdjacent02(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = new poly_Mesh;

  // Add vertices so that two triangles lie on the same straight line.
  poly_VertexHandle hv_a = mesh->AddVertex(-1.0, 0.0, 0.0);
  poly_VertexHandle hv_x = mesh->AddVertex( 0.0, 0.0, 0.0);
  poly_VertexHandle hv_y = mesh->AddVertex( 0.0, 1.0, 0.0);
  poly_VertexHandle hv_b = mesh->AddVertex( 1.0, 0.0, 0.0);

  // Add triangle.
  poly_TriangleHandle ht0 = mesh->AddTriangle(hv_a, hv_x, hv_y);
  poly_TriangleHandle ht1 = mesh->AddTriangle(hv_x, hv_b, hv_y);

  std::unordered_set<poly_TriangleHandle> ths;

  // Find adjacent triangles.
  mesh->FindAdjacentByVertices(poly_TriangleHandle(1), ths);
  //
  if ( ths.size() != 2 )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests midedge refinement.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::refineByMidedges01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = new poly_Mesh;

  // Add vertices.
  poly_VertexHandle hv0 = mesh->AddVertex(0., 0., 0.);
  poly_VertexHandle hv1 = mesh->AddVertex(1., 0., 0.);
  poly_VertexHandle hv2 = mesh->AddVertex(0., 1., 0.);

  // Add triangle.
  poly_TriangleHandle ht = mesh->AddTriangle(hv0, hv1, hv2);

  // Compute edges.
  mesh->ComputeEdges();

  // Refine triangle.
  if ( !mesh->RefineByMidedges(ht) )
  {
    return res.failure();
  }

  // Validate the number of vertices.
  if ( mesh->GetNumVertices() != 6 )
  {
    return res.failure();
  }

  // Validate the number of edges: 3 are "dead".
  if ( mesh->GetNumEdges() != 12 )
  {
    return res.failure();
  }

  // Validate the number of triangles: one "dead" is there.
  if ( mesh->GetNumTriangles(true) != 5 )
  {
    return res.failure();
  }

  // Verify the "is deleted" flag.
  poly_Triangle t;
  if ( !mesh->GetTriangle(ht, t) )
  {
    return res.failure();
  }
  //
  if ( !t.IsDeleted() )
  {
    return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests midedge refinement.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Mesh::refineByMidedges02(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  t_ptr<poly_Mesh> mesh = readSTL(filename_mesh_010);
  //
  if ( mesh.IsNull() )
    return res.failure();

  // Compute edges.
  mesh->ComputeEdges();

  int numTris = mesh->GetNumTriangles();
  //
  for ( int idx = 0; idx < numTris; ++idx )
  {
    poly_TriangleHandle th(idx);
    poly_Triangle       t;

    // Get the next triangle to process.
    mesh->GetTriangle(th, t);
    //
    if ( t.IsDeleted() )
      continue;

    // Refine triangles.
    if ( !mesh->RefineByMidedges(th) )
    {
      return res.failure();
    }
  }

  // Validate the number of vertices.
  if ( mesh->GetNumVertices() != 11 )
  {
    return res.failure();
  }

  // Validate the number of edges.
  if ( mesh->GetNumEdges() != 27 )
  {
    return res.failure();
  }

  // Validate the number of triangles.
  if ( mesh->GetNumTriangles(true) != 15 )
  {
    return res.failure();
  }

  return res.success();
}
