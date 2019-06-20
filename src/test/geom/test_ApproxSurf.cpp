//-----------------------------------------------------------------------------
// Created on: 17 June 2019
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

// Own include
#include <mobius/test_ApproxSurf.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Geom includes
#include <mobius/geom_ApproxBSurf.h>
#include <mobius/geom_BuildAveragePlane.h>

// Core includes
#include <mobius/core_Precision.h>

// Standard includes
#include <fstream>

//-----------------------------------------------------------------------------

// Filenames are specified relatively to MOBIUS_TEST_DATA environment variable.
#define filename_points_001 "points/sampled-surf_01.xyz"
#define filename_points_002 "points/interior-nodes_03.xyz"
//
#define filename_bsurf_test02_initial "bsurf/bsurf_test02_initial.json"
#define filename_bsurf_test02_final "bsurf/bsurf_test02_final.json"

//-----------------------------------------------------------------------------

std::string mobius::test_ApproxSurf::jsonFromFile(const std::string& shortFilename)
{
  std::ifstream FILE(   core::str::slashed( core::env::MobiusTestData() )
                      + shortFilename );
  std::stringstream buffer;
  buffer << FILE.rdbuf();

  return buffer.str();
}

//-----------------------------------------------------------------------------

bool mobius::test_ApproxSurf::runtest(const std::string&    shortFilename,
                                      const bool            hasInitSurf,
                                      const t_ptr<t_bsurf>& initSurf,
                                      const int             uDegree,
                                      const int             vDegree,
                                      const double          lambda,
                                      const t_ptr<t_bsurf>& refSurf)
{
  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  /* ====================================
   *  Prepare point cloud to approximate
   * ==================================== */

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + shortFilename;

  // Read point cloud.
  t_ptr<t_pcloud> pts = new t_pcloud;
  //
  if ( !pts->Load(filename) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Cannot load point cloud.");
    return false;
  }

  /* =============
   *  Approximate
   * ============= */

  t_ptr<t_bsurf> finalSurf;

  if ( !hasInitSurf )
  {
    // Prepare approximation tool.
    geom_ApproxBSurf approx(pts, uDegree, vDegree, cf->ProgressNotifier);

    // Perform.
    if ( !approx.Perform(lambda) )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Approximation failed.");
      return false;
    }

    // Get result.
    finalSurf = approx.GetResult();
  }
  else
  {
    // Prepare approximation tool.
    geom_ApproxBSurf approx(pts, initSurf, cf->ProgressNotifier);

    // Perform.
    if ( !approx.Perform(lambda) )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Approximation failed.");
      return false;
    }

    // Get result.
    finalSurf = approx.GetResult();
  }

  /* ================================
   *  Compare with reference surface
   * ================================ */

  // Compare the constructed surface with the reference one.
  if ( !t_bsurf::Compare(finalSurf, refSurf, 1.0e-3, 1.0e-3) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Approximated surface deviates too much "
                                                             "from the expected surface.");
    //
    std::cout << ">>>" << std::endl;
    std::cout << finalSurf->DumpJSON().c_str() << std::endl;
    std::cout << "<<<" << std::endl;
    //
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ApproxSurf::testApprox01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // JSON definition for reference surface.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: CN,\
    domain: {\
        U_min: 0,\
        U_max: 1,\
        V_min: 0,\
        V_max: 1\
    },\
    flags: {\
        is_U_rational: 0,\
        is_V_rational: 0,\
        is_U_periodic: 0,\
        is_V_periodic: 0,\
        is_U_closed: 0,\
        is_V_closed: 0\
    },\
    properties: {\
        U_degree: 3,\
        V_degree: 3,\
        U_knots: [0, 0, 0, 0, 1, 1, 1, 1],\
        V_knots: [0, 0, 0, 0, 1, 1, 1, 1],\
        num_poles_in_U_axis: 4,\
        num_poles_in_V_axis: 4,\
        poles: {\
            u0: [[0.032490056641749421, 0.2075899743679602, 1.9692517034236938], [-2.1610849773994021, 3.3213484430736462, 2.424052345446388], [-2.4580507723861964, 2.5159951518711803, 1.6727959255608684], [-2.9837721413015816, 2.1833373643387626, 1.0670055667387794]],\
            u1: [[-1.3519757214666765, 4.5763346643997727, 2.4799372039817196], [-1.9194950580172998, 4.3300472812537789, 1.9007262443352204], [-2.3954055473737741, 3.8944613840018181, 1.2632610619292655], [-2.6683177996591958, 3.0394043793828414, 0.49670895306680457]],\
            u2: [[-0.20122657946475536, 3.7063668126372558, 1.3784751968203861], [-1.1980930364276527, 4.3472729081285379, 1.0722868060391737], [-1.5726530519628097, 3.7022586134705864, 0.3703726838362843], [-2.1147848834518719, 3.4035110461283038, -0.22498223374514542]],\
            u3: [[0.43646189429009408, 3.896576279882078, 0.60326934690426115], [-0.27972300518629734, 3.9574881573590877, 0.11859507322210901], [-1.0116543057930047, 4.0509380528398937, -0.3560660370286528], [-1.6552943489917789, 3.9619448293985293, -0.88687170900390211]]\
        }\
    }\
  }";

  const double lambda = 0.;

  if ( !runtest( filename_points_001, false, NULL, 3, 3, lambda, t_bsurf::Instance(json) ) )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ApproxSurf::testApprox02(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Construct initial surface.
  t_ptr<t_bsurf> initSurf = t_bsurf::Instance( jsonFromFile(filename_bsurf_test02_initial) );

  // Construct reference surface.
  t_ptr<t_bsurf> refSurf = t_bsurf::Instance( jsonFromFile(filename_bsurf_test02_final) );

  const double lambda = 0.001;

  if ( !runtest(filename_points_002, true, initSurf, 0, 0, lambda, refSurf) )
    return res.failure();

  return res.success();
}
