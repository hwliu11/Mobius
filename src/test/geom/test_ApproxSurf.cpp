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
#define filename_points_003 "points/sampled-surf_02.xyz"
#define filename_points_004 "points/sampled-surf_03.xyz"
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
            u0: [[1.4289262983286861, -2.7046506832572375, 1.4741943473683692], [-2.5036067307554499, 3.7186530917764213, 2.8467818597746177], [-2.4023667770615997, 1.8066567879329356, 1.6542862272831005], [-3.2707172476564836, 1.8982008874419889, 1.0783548051805991]],\
            u1: [[-1.707343200426078, 5.5855658094913219, 3.0249151011199293], [-1.9601515267657172, 4.4051671111898738, 2.0575593759728013], [-2.6694873803378627, 4.1681268916331025, 1.3805102867102719], [-2.7968852845462209, 2.7285828458214727, 0.333405859442509]],\
            u2: [[0.37471963161881389, 3.0927347509025092, 1.2572893926855522], [-1.219047181602916, 4.6832632805259093, 1.1426514647350037], [-1.4366611053558997, 3.4301396401002289, 0.15291555871421272], [-2.2337696776569214, 3.3744710425159208, -0.4683187084250468]],\
            u3: [[0.78389970531784758, 4.0567119591365941, 0.55345269390603302], [0.11854976222340048, 3.7287802154984262, -0.15156711095201186], [-0.96271441589533713, 4.2602845332727011, -0.59210634189329192], [-1.7151228497472255, 4.1122485467583623, -1.2417655034560287]]\
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

//-----------------------------------------------------------------------------

//! Test scenario 003.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ApproxSurf::testApprox03(const int funcID)
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
            u0: [[-0.22466040107566196, 0.71228462836520834, 2.5257129602298845], [-1.5078663964784826, 1.6610770897651825, 2.2135888766671838], [-2.8990141104158935, 2.832917952385122, 1.9701051167075661], [-3.1501889022979936, 1.6491437557418991, 1.0017106240963867]],\
            u1: [[-0.5368268673413551, 3.1668365144633426, 2.2805817611627699], [-2.4845490892200583, 5.4887707408224449, 2.3910246835962399], [-2.5023132704421158, 3.8226819744316707, 1.2742039840974673], [-2.7272226507908059, 2.5846335070762985, 0.28910727132980912]],\
            u2: [[-0.30853778650252972, 4.5046029041715387, 1.6917739544888173], [-0.97152410862231808, 4.1717870272056183, 0.98525111893804829], [-1.4837773037231003, 3.5274995204083193, 0.18287682987139381], [-2.3345151866478955, 3.5826493534055177, -0.40425446675227111]],\
            u3: [[1.0230536415383273, 3.5625295067189739, 0.40137429649288248], [0.087227816198583122, 3.7935031988624606, -0.13164943188988992], [-1.0121951354110832, 4.3625304068927653, -0.56064146761570866], [-1.6049541377259484, 3.8845983355512153, -1.311821976055412]]\
        }\
    }\
  }";

  const double lambda = 0.001;

  if ( !runtest( filename_points_001, false, NULL, 3, 3, lambda, t_bsurf::Instance(json) ) )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 004.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ApproxSurf::testApprox04(const int funcID)
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
        U_degree: 5,\
        V_degree: 5,\
        U_knots: [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],\
        V_knots: [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],\
        num_poles_in_U_axis: 6,\
        num_poles_in_V_axis: 6,\
        poles: {\
            u0: [[644.57064865135658, -23.136268140913117, -22.914894465686935], [619.99002994571345, -25.815337634666033, -29.223511591878019], [595.36339651381672, -29.20697846376914, -35.172211984550358], [570.887789161564, -30.259871172333064, -42.302204049090193], [546.38607515857211, -31.717044271561225, -49.227995817580847], [521.88362876427504, -33.185558970680901, -56.148058992479378]],\
            u1: [[641.41331981213625, -14.549921029324116, -13.749025787508071], [616.84737759763777, -17.001714468481673, -20.17243907672356], [592.25976756074363, -19.789049718011302, -26.426371632752776], [567.69150635847438, -22.276754423649489, -32.83164629488931], [543.21674096191089, -23.316608789166821, -39.968223970713659], [518.70142246281182, -24.984457365072615, -46.787604457089799]],\
            u2: [[638.26709415037487, -5.7916331987981735, -4.6700036310029516], [613.70679092369267, -8.156102740344469, -11.137523847356066], [589.20443270126202, -9.623252199808876, -18.05827659900676], [564.51913974439412, -13.923277995486437, -23.548155630575824], [540.03898433013262, -15.046600887647816, -30.642573710657945], [515.53300226208978, -16.569868005866727, -37.534981672859217]],\
            u3: [[634.91832668070174, -0.16985175667984465, 5.9932544861135861], [610.38483967760749, -2.1190528156796082, -0.6840160645212231], [585.73913174506788, -5.8060758987424439, -6.4835201441802557], [561.34733996231807, -5.5610241155230646, -14.269098419656981], [536.84976271601477, -6.9541363821275173, -21.227247040701748], [512.33360693942529, -8.6349508375807922, -28.040078516766314]],\
            u4: [[631.29606843401234, 1.2167271073255339, 18.795695309657582], [606.74294813583435, -1.0365096190181267, 12.271991874119182], [582.19852709857548, -3.1550321296857931, 5.6802448644742416], [557.77217952548358, -3.4451014066118577, -1.8350460524736498], [533.2468004872477, -5.2687447735925934, -8.5757352562531981], [508.77923486036832, -6.1971055318289316, -15.768627876230999]],\
            u5: [[627.73614543917256, 3.5686124648315696, 31.110563952365045], [603.18686211620627, 1.3747940348363754, 24.556848589497559], [578.64774244905891, -0.66163281166010945, 17.923635411034823], [554.17557642254474, -1.6612341352228819, 10.766726095888481], [529.73134634316762, -2.2282270686736005, 3.3913081333411852], [505.26180969084129, -3.1871106119850108, -3.786167558443934]]\
        }\
    }\
  }";

  const double lambda = 0.001;

  if ( !runtest( filename_points_003, false, NULL, 5, 5, lambda, t_bsurf::Instance(json) ) )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 005.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ApproxSurf::testApprox05(const int funcID)
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
            u0: [[637.44322937659081, -24.048896167909319, -23.631297602400402], [595.97584380820115, -26.849818471046127, -31.934849392829275], [554.58533898725057, -27.44136458519937, -40.916044218872642], [513.21460359737091, -27.464781510629454, -50.071491301853186]],\
            u1: [[632.43662945322387, 3.2314366646453192, -3.9019245428674001], [590.80201480112498, -4.3752679787136515, -10.731483381012859], [549.45818115896645, -3.6255896658982514, -20.124048346484706], [508.00744563131366, -5.9480290862591545, -28.57435676223561]],\
            u2: [[626.49506081080813, 3.6429057468613801, 24.068461739354724], [584.98358443917675, -0.42508507216886204, 16.153535552554128], [543.72925030509668, 2.8966023865833046, 5.9721035330795313], [502.28326589061891, 0.71070116375901837, -2.5200828384135852]],\
            u3: [[620.53777431685444, 3.602680249535283, 52.177388342466372], [578.92036375056227, -3.509618087182687, 45.196188968069052], [537.46835617452825, -5.8686115703347328, 36.757092136212272], [496.15908780774635, -4.125608022220395, 27.059862020736713]]\
        }\
    }\
  }";

  const double lambda = 0.001;

  if ( !runtest( filename_points_004, false, NULL, 3, 3, lambda, t_bsurf::Instance(json) ) )
    return res.failure();

  return res.success();
}
