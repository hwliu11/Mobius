//-----------------------------------------------------------------------------
// Created on: 20 August 2018
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
#include <mobius/test_FairSurf.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Geom includes
#include <mobius/geom_FairBSurf.h>

// Standard includes
#include <fstream>

//-----------------------------------------------------------------------------

// Filenames are specified relatively to MOBIUS_TEST_DATA environment variable.
#define filename_bsurf_001 "bsurf/bsurf_001.json"
#define filename_bsurf_002 "bsurf/bsurf_002.json"
#define filename_bsurf_003 "bsurf/bsurf_003.json"

//-----------------------------------------------------------------------------

//! Test scenario 001. This test function checks that mapping of indices is
//! done correctly.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairSurf::testIndices(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: C2,\
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
        V_degree: 1,\
        U_knots: [0, 0, 0, 0, 0.41551883124182798, 0.59974028777161115, 1, 1, 1, 1],\
        V_knots: [0, 0, 1, 1],\
        num_poles_in_U_axis: 6,\
        num_poles_in_V_axis: 2,\
        poles: {\
            u0: [[241.87747035573125, 62.485923112593042, 0], [307.49011857707512, 49.837701768719491, 0]],\
            u1: [[245.29391602546553, 57.035146252318057, 0], [306.57174310419981, 45.021517271473421, 0]],\
            u2: [[250.62433050286035, 49.413132207145885, 25], [303.8174241480333, 37.906450425886014, 0]],\
            u3: [[260.43129359394032, 37.406082628166004, -22], [295.65540308364263, 29.098297811030434, -30]],\
            u4: [[266.65853554251112, 30.811365428505532, 0], [288.81855843279288, 26.561552469297354, 0]],\
            u5: [[271.12648221343875, 26.517543665952715, 0], [284.16996047430831, 25.727029831960614, 0]]\
        }\
    }\
  }";

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  geom_FairBSurf F(surf, 0.0, NULL, NULL);

  // Test 1.
  {
    int i = 0, j = 0;
    F.GetIJ(0, i, j);
    //
    if ( i != 0 || j != 0 )
      return res.failure();
  }

  // Test 2.
  {
    int i = 0, j = 0;
    F.GetIJ(1, i, j);
    //
    if ( i != 0 || j != 1 )
      return res.failure();
  }

  // Test 3.
  {
    int i = 0, j = 0;
    F.GetIJ(2, i, j);
    //
    if ( i != 1 || j != 0 )
      return res.failure();
  }

  // Test 4.
  {
    int i = 0, j = 0;
    F.GetIJ(3, i, j);
    //
    if ( i != 1 || j != 1 )
      return res.failure();
  }

  // Test 5.
  {
    int i = 0, j = 0;
    F.GetIJ(4, i, j);
    //
    if ( i != 2 || j != 0 )
      return res.failure();
  }

  // Test 6.
  {
    if ( F.GetK(0, 0) != 0 )
      return res.failure();
  }

  // Test 7.
  {
    if ( F.GetK(0, 1) != 1 )
      return res.failure();
  }

  // Test 8.
  {
    if ( F.GetK(1, 0) != 2 )
      return res.failure();
  }

  // Test 9.
  {
    if ( F.GetK(1, 1) != 3 )
      return res.failure();
  }

  // Test 10.
  {
    if ( F.GetK(2, 0) != 4 )
      return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairSurf::testFairing01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: C2,\
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
        V_degree: 1,\
        U_knots: [0, 0, 0, 0, 0.41551883124182798, 0.59974028777161115, 1, 1, 1, 1],\
        V_knots: [0, 0, 1, 1],\
        num_poles_in_U_axis: 6,\
        num_poles_in_V_axis: 2,\
        poles: {\
            u0: [[241.87747035573125, 62.485923112593042, 0], [307.49011857707512, 49.837701768719491, 0]],\
            u1: [[245.29391602546553, 57.035146252318057, 0], [306.57174310419981, 45.021517271473421, 0]],\
            u2: [[250.62433050286035, 49.413132207145885, 25], [303.8174241480333, 37.906450425886014, 0]],\
            u3: [[260.43129359394032, 37.406082628166004, -22], [295.65540308364263, 29.098297811030434, -30]],\
            u4: [[266.65853554251112, 30.811365428505532, 0], [288.81855843279288, 26.561552469297354, 0]],\
            u5: [[271.12648221343875, 26.517543665952715, 0], [284.16996047430831, 25.727029831960614, 0]]\
        }\
    }\
  }";

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  // Compute initial bending energy.
  const double initEnergy = surf->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Initial bending energy: %1."
                                                          << surf->ComputeBendingEnergy() );

  // Perform fairing.
  geom_FairBSurf F(surf, 1.0, NULL, NULL);
  //
  F.AddPinnedPole(0, 0);
  //
  if ( !F.Perform() )
    return res.failure();

  // Get the result.
  const core_Ptr<bsurf>& result = F.GetResult();

  // Compute bending energy after fairing.
  const double resEnergy = result->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Resulting bending energy: %1"
                                                          << resEnergy );

  // Resulting energy should always be smaller.
  if ( resEnergy > initEnergy )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Energy is not decreasing (%1 -> %2)."
                                                           << initEnergy << resEnergy );
    return res.failure();
  }

  // Verify.
  const double refEnergy = 0.44307026;
  //
  if ( abs(resEnergy - refEnergy) > 1e-4 )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Final energy (%1) is too different from expected (%2)."
                                                           << resEnergy << refEnergy );
    return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 003.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairSurf::testFairing02(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: C2,\
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
        U_knots: [0, 0, 0, 0, 0.29336113425758786, 0.43111726937688855, 0.56937892352479658, 0.70895620453393871, 1, 1, 1, 1],\
        V_knots: [0, 0, 0, 0, 0.1294845791381016, 0.19381311402294302, 0.25684864948626557, 0.32344342186922237, 0.38757091516091147, 0.46113744994487255, 0.53072289321498334, 0.60701623322132947, 0.6730295123449288, 0.73969475105816018, 0.79956974333588082, 0.86300992056857029, 1, 1, 1, 1],\
        num_poles_in_U_axis: 8,\
        num_poles_in_V_axis: 16,\
        poles: {\
            u0: [[91.061186765174298, 1323.340243680999, 303.56014022245739], [92.336856821607284, 1326.8703457318184, 295.13158417987444], [92.098485132442789, 1332.7693272171589, 282.06134675234284], [91.185209564745463, 1340.6258913422685, 264.90872003727611], [90.617241671426868, 1346.5591197993431, 251.9030657333779], [88.101070660657712, 1352.6858177887418, 239.29916219276262], [84.034795005083936, 1359.3795312051677, 226.09252025002351], [80.61211073070325, 1366.0672057207414, 212.62392847885189], [76.597476877173762, 1373.1932675010648, 198.42976447851532], [73.235320939217985, 1380.0098781362542, 184.64735814181728], [69.858812113917679, 1386.7381694917126, 171.06830869077746], [68.411235967117577, 1392.7296582179047, 158.30900237147688], [63.977458762344881, 1399.0263037314978, 146.14627913734728], [59.127220290952529, 1407.4722109843108, 129.36254846001782], [56.166031152748779, 1413.7840131443418, 116.53576488627394], [53.998867801137052, 1418.1372474722573, 107.74259051100573]],\
            u1: [[89.650879605191903, 1314.9235756062644, 299.77754370334549], [89.466202085048451, 1318.6500892929964, 291.53538374905435], [89.680973344675508, 1324.4881246383209, 278.40730798601487], [89.623473370567822, 1332.2295899424316, 261.14545164256452], [89.457437222290707, 1338.1087599870875, 248.0884955073507], [86.958365561059452, 1344.2331581738463, 235.4824094370978], [84.805164600862767, 1350.66956974202, 222.03158614633895], [82.672192952207155, 1357.1837824251274, 208.39837781927784], [78.178080701118489, 1364.3743323711199, 194.26541356286521], [72.456335848991571, 1371.5082994535919, 180.78418083074715], [70.174760169169204, 1378.0893261346937, 167.06537611970515], [68.429177487004139, 1384.1208956964767, 154.34410680842115], [66.707283601250566, 1390.0528023988238, 141.83524375752765], [67.428402252111809, 1397.7493816257763, 124.34039526775351], [56.546865241651169, 1405.1264428429847, 112.52455038078342], [57.179263780558784, 1409.103145898777, 103.37404505657328]],\
            u2: [[87.148585790244809, 1302.2699670657089, 294.1158973696588], [87.830887816806438, 1305.8798750389738, 285.76307773494148], [87.672792169636054, 1311.7680596777427, 272.68259402053098], [86.794746970947699, 1319.6198854415704, 255.52547056573113], [85.975831048888253, 1325.5868655298632, 242.55184680830905], [83.732076891280187, 1331.6769244063271, 229.91317247981453], [81.512324753222771, 1338.1222868749142, 216.47084365942825], [78.950526214482906, 1344.6941752732703, 202.89237001070694], [76.358160832545195, 1351.6289469239812, 188.51667026925094], [75.184531598977287, 1358.151208380151, 174.45492439492389], [70.755510440063631, 1365.0210589593096, 161.0102156892041], [67.727188102597523, 1371.2251525134484, 148.45267291664317], [66.917198652889681, 1377.0344112620123, 135.82741605478148], [64.643762917610687, 1385.1337475690229, 118.71478699872752], [66.451916446475252, 1390.8040898393053, 105.2792532677454], [62.98104325287877, 1395.3326685762706, 96.652482027395635]],\
            u3: [[81.924861185793347, 1285.6380473107724, 286.80135575692316], [82.745083590228433, 1289.2294054761194, 278.4309322179418], [83.155986241208723, 1295.0410618360272, 265.27782260356076], [85.45649104369032, 1302.4653837538074, 247.71499485194764], [82.322827869521888, 1308.7436892428441, 235.03682119154101], [80.508173523248175, 1314.7760356970998, 222.34337734954073], [77.919457825317949, 1321.2710224726297, 208.94814236173602], [75.240358121769191, 1327.8586874669502, 195.38464081871268], [72.931183315763207, 1334.7553709767337, 180.97279515114946], [70.573286871895064, 1341.436912226196, 167.06220697313466], [69.484596447898838, 1347.8575000170567, 153.19114457424291], [68.918666963470244, 1353.7304103540848, 140.31930649075355], [67.03865699368653, 1359.6835831161143, 127.83062508704541], [66.244208446094703, 1367.5840008247851, 110.52922081689832], [65.029762143346176, 1373.6608721893406, 97.479486188897141], [65.100805674387132, 1377.7130755282124, 88.400631189094497]],\
            u4: [[78.19815596878064, 1273.4779370941594, 281.44738870162217], [78.75311446495185, 1277.104972326397, 273.11082296174078], [81.25227042368185, 1282.6357659280445, 259.69117252230626], [79.653899845166265, 1290.584472933373, 242.62599007884444], [78.204545250284013, 1296.6362448157952, 229.73283435821503], [76.575167610888016, 1302.6436722016888, 217.01574213690529], [74.934291938191819, 1309.0111778125095, 203.49952658538891], [73.210387596857885, 1315.4703723750522, 189.8141056569211], [70.304492848420765, 1322.4473126277903, 175.47842422919945], [68.527492132538299, 1329.0507254357356, 161.49369158507417], [67.385856321633284, 1335.478434195762, 147.62938703833422], [66.896637752138133, 1341.3410271833159, 134.74775771425092], [67.871990386546059, 1346.910163672613, 121.89462306077394], [66.651259835253114, 1354.8679148182289, 104.64762864433156], [66.258789296562796, 1360.834233320521, 91.492978536242632], [65.22468039483384, 1365.0350757921103, 82.555183167543191]],\
            u5: [[74.899896210297172, 1256.7011489639488, 273.93961909265568], [72.402105644777407, 1260.7387682563447, 265.99270064822832], [76.756851034945612, 1266.0199915764683, 252.33620617450848], [72.971197795257808, 1274.2628804413559, 235.55020448127956], [71.481547071807725, 1280.3200720119819, 222.66219208464966], [70.156850026041411, 1286.2865209248118, 209.90621099075818], [68.165409854051788, 1292.7011762333534, 196.43474084945123], [67.445773605629427, 1299.0253002536604, 182.62113698212866], [67.520038809853034, 1305.6014194272468, 167.90507339979646], [65.746063240686311, 1312.2044253635531, 153.91995463148834], [65.144247674586424, 1318.5595301907283, 139.98674841803114], [65.42177835542266, 1324.3189980887478, 127.00725262485916], [65.348952718602263, 1330.0291108855201, 114.28790552492291], [67.843661272613019, 1337.4871484804037, 96.566679270373527], [65.96412269787136, 1343.6534724268447, 83.601835801053639], [69.514109175496742, 1347.2377701202799, 74.078934899416112]],\
            u6: [[63.513707227328993, 1245.1350007240198, 269.36245032946266], [77.347859113868935, 1246.9760309399037, 259.33095268204556], [71.678522193198461, 1253.6054582333616, 246.95391370331984], [70.037194889612181, 1261.5599427678053, 229.8942141775201], [68.446980671646401, 1267.6306597763357, 217.01903752102643], [65.248983921212584, 1273.8490609404316, 204.50216096823814], [64.874343247375862, 1280.0462623773963, 190.82432550209867], [63.600822035758682, 1286.4448819859267, 177.08141849637278], [62.213291596199731, 1293.2176075553919, 162.55193583097571], [62.732908036350736, 1299.512133397036, 148.27406718207024], [64.359278214038838, 1305.5675550432588, 134.05645942387812], [64.029658875838777, 1311.4086824930127, 121.15445914640999], [65.253985605838736, 1316.9443328381913, 108.26954589566741], [63.283176714774136, 1325.0029668942409, 91.118290103842668], [74.260814962789965, 1329.440045575072, 76.51238014824574], [75.728096394175466, 1333.3044598050133, 67.255311901731829]],\
            u7: [[52.286941365018251, 1239.0308058145672, 267.28964258107879], [59.086397318145345, 1241.8179780028031, 258.15604063009778], [57.793589604192704, 1247.8587774415653, 245.22038948430318], [58.426621965521761, 1255.5073685643313, 227.87039485909733], [58.518366345973597, 1261.3518680304448, 214.78053608963199], [63.43210329097014, 1266.4792694142507, 201.22829271435327], [61.709356985652683, 1272.8577863385046, 187.72252698034021], [61.694417413566775, 1279.087131123212, 173.81897692074946], [60.898871373604806, 1285.7802368636549, 159.21393445151946], [61.094154158164081, 1292.1183844463822, 144.97746315600668], [61.895662589621359, 1298.2847471093689, 130.86513923937693], [63.429604042170318, 1303.875232157684, 117.72527747793042], [63.832690844608337, 1309.5213363976977, 104.94518578645324], [68.699587450276141, 1316.6603230054081, 86.921177796505603], [73.513772115287495, 1321.9263646129511, 73.101959760466656], [76.254722709252661, 1325.6194748000439, 63.682322716297641]]\
        }\
    }\
  }";

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  // Compute initial bending energy.
  const double initEnergy = surf->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Initial bending energy: %1."
                                                          << surf->ComputeBendingEnergy() );

  // Perform fairing.
  geom_FairBSurf F(surf, 1.0, NULL, NULL);
  //
  if ( !F.Perform() )
    return res.failure();

  // Get the result.
  const core_Ptr<bsurf>& result = F.GetResult();

  // Compute bending energy after fairing.
  const double resEnergy = result->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Resulting bending energy: %1"
                                                          << resEnergy );

  // Resulting energy should always be smaller.
  if ( resEnergy > initEnergy )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Energy is not decreasing (%1 -> %2)."
                                                           << initEnergy << resEnergy );
    return res.failure();
  }

  // Verify.
  const double refEnergy = 0.06209;
  //
  if ( abs(resEnergy - refEnergy) > 1e-4 )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Final energy (%1) is too different from expected (%2)."
                                                           << resEnergy << refEnergy );
    return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 004.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairSurf::testFairing03(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + filename_bsurf_001;

  // Read file.
  std::ifstream FILE(filename);
  std::stringstream buffer;
  buffer << FILE.rdbuf();

  // JSON definition.
  std::string json = buffer.str();

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  // Compute initial bending energy.
  const double initEnergy = surf->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Initial bending energy: %1."
                                                          << surf->ComputeBendingEnergy() );

  // Perform fairing.
  geom_FairBSurf F(surf, 1.0, NULL, NULL);
  //
  if ( !F.Perform() )
    return res.failure();

  // Get the result.
  const core_Ptr<bsurf>& result = F.GetResult();

  // Compute bending energy after fairing.
  const double resEnergy = result->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Resulting bending energy: %1"
                                                          << resEnergy );

  // Resulting energy should always be smaller.
  if ( resEnergy > initEnergy )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Energy is not decreasing (%1 -> %2)."
                                                           << initEnergy << resEnergy );
    return res.failure();
  }

  // Verify.
  const double refEnergy = 0.0049;
  //
  if ( abs(resEnergy - refEnergy) > 1e-4 )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Final energy (%1) is too different from expected (%2)."
                                                           << resEnergy << refEnergy );
    return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 005.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairSurf::testFairing04(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + filename_bsurf_002;

  // Read file.
  std::ifstream FILE(filename);
  std::stringstream buffer;
  buffer << FILE.rdbuf();

  // JSON definition.
  std::string json = buffer.str();

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  // Compute initial bending energy.
  const double initEnergy = surf->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Initial bending energy: %1."
                                                          << surf->ComputeBendingEnergy() );

  // Prepare fairing tool.
  geom_FairBSurf F(surf, 1.0, NULL, NULL);

  // Constraint borders.
  const int nPolesU = int( surf->GetPoles().size() );
  const int nPolesV = int( surf->GetPoles()[0].size() );
  //
  for ( int i = 0; i < nPolesU; ++i )
  {
    F.AddPinnedPole( i, 0 );
    F.AddPinnedPole( i, nPolesV - 1 );
  }
  //
  for ( int j = 0; j < nPolesV; ++j )
  {
    F.AddPinnedPole( 0, j );
    F.AddPinnedPole( nPolesU - 1, j );
  }

  // Perform fairing.
  if ( !F.Perform() )
    return res.failure();

  // Get the result.
  const core_Ptr<bsurf>& result = F.GetResult();

  // Compute bending energy after fairing.
  const double resEnergy = result->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Resulting bending energy: %1"
                                                          << resEnergy );

  // Resulting energy should always be smaller.
  if ( resEnergy > initEnergy )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Energy is not decreasing (%1 -> %2)."
                                                           << initEnergy << resEnergy );
    return res.failure();
  }

  // Verify.
  const double refEnergy = 453068.70181; // Energy of constrained surface is far from zero.
  //
  if ( abs(resEnergy - refEnergy) > 1e-4 )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Final energy (%1) is too different from expected (%2)."
                                                           << resEnergy << refEnergy );
    return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 006.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairSurf::testFairing05(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + filename_bsurf_003;

  // Read file.
  std::ifstream FILE(filename);
  std::stringstream buffer;
  buffer << FILE.rdbuf();

  // JSON definition.
  std::string json = buffer.str();

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  // Compute initial bending energy.
  const double initEnergy = surf->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Initial bending energy: %1."
                                                          << surf->ComputeBendingEnergy() );

  // Prepare fairing tool.
  geom_FairBSurf F(surf, 1.0, NULL, NULL);

  // Perform fairing.
  if ( !F.Perform() )
    return res.failure();

  // Get the result.
  const core_Ptr<bsurf>& result = F.GetResult();

  // Compute bending energy after fairing.
  const double resEnergy = result->ComputeBendingEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Resulting bending energy: %1"
                                                          << resEnergy );

  // Resulting energy should always be smaller.
  if ( resEnergy > initEnergy )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Energy is not decreasing (%1 -> %2)."
                                                           << initEnergy << resEnergy );
    return res.failure();
  }

  // Verify.
  const double refEnergy = 0.8874; // Energy of constrained surface is far from zero.
  //
  if ( abs(resEnergy - refEnergy) > 1e-4 )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Final energy (%1) is too different from expected (%2)."
                                                           << resEnergy << refEnergy );
    return res.failure();
  }

  return res.success();
}
