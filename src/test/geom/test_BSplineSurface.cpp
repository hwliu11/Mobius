//-----------------------------------------------------------------------------
// Created on: 15 June 2018
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
#include <mobius/test_BSplineSurface.h>

// geom includes
#include <mobius/geom_BSplineSurface.h>

//-----------------------------------------------------------------------------

//! Test scenario 001: evaluate B-surface in its domain.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineSurface::evalInDomain(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // Control points.
  std::vector< std::vector<xyz> >
    Q = { { xyz(0.0, 0.0, 0.0), xyz(0.0, 1.0, 0.0) },
          { xyz(1.0, 0.0, 0.0), xyz(1.0, 1.0, 0.0) },
          { xyz(2.0, 0.0, 0.0), xyz(2.0, 1.0, 0.0) } };

  // Knot vectors.
  const std::vector<double> U = {0, 0, 0.5, 1, 1};
  const std::vector<double> V = {0, 0, 1, 1};

  // Degrees.
  const int p = 1;
  const int q = 1;

  // Construct B-surface.
  core_Ptr<bsurf> surf = new bsurf(Q, U, V, p, q);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 0.5;
  const double v   = 0.5;
  //
  const xyz P_ref(1, 0.5, 0);

  // Evaluate.
  xyz P;
  surf->Eval(u, v, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002: evaluate B-surface out of its domain.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineSurface::evalOutDomain(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // Control points.
  std::vector< std::vector<xyz> >
    Q = { { xyz(0.0, 0.0, 0.0), xyz(0.0, 1.0, 0.0) },
          { xyz(1.0, 0.0, 0.0), xyz(1.0, 1.0, 0.0) },
          { xyz(2.0, 0.0, 0.0), xyz(2.0, 1.0, 0.0) } };

  // Knot vectors.
  const std::vector<double> U = {0, 0, 0.5, 1, 1};
  const std::vector<double> V = {0, 0, 1, 1};

  // Degrees.
  const int p = 1;
  const int q = 1;

  // Construct B-surface.
  core_Ptr<bsurf> surf = new bsurf(Q, U, V, p, q);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 1.5;
  const double v   = 1.5;
  //
  const xyz P_ref(3, 1.5, 0);

  // Evaluate.
  xyz P;
  surf->Eval(u, v, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Evaluates B-surface specified as JSON object.
//!
//! The surface is C0-continuous. It is evaluated in the middle point.
//!
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineSurface::evalJSON1(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: C0,\
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
        U_degree: 1,\
        V_degree: 1,\
        U_knots: [0, 0, 0.5, 1, 1],\
        V_knots: [0, 0, 0.5, 1, 1],\
        num_poles_in_U_axis: 3,\
        num_poles_in_V_axis: 3,\
        poles: {\
            u0: [[-10, -10, 0], [-10, 0, 0], [-10, 10, 0]],\
            u1: [[0, -10, 0], [0, 0, 10], [0, 10, 0]],\
            u2: [[10, -10, 0], [10, 0, 0], [10, 10, 0]]\
        }\
    }\
  }";

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 0.5;
  const double v   = 0.5;
  //
  const xyz P_ref(0, 0, 10);

  // Evaluate.
  xyz P;
  surf->Eval(u, v, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Evaluates B-surface specified as JSON object.
//!
//! The surface has irregular parameterization near the max value of its curvilinear
//! coordinate V = V_max = 5.1866714434994.
//!
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineSurface::evalJSON2(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: C2,\
    domain: {\
        U_min: 0,\
        U_max: 0.95031615985331297,\
        V_min: 2.7739444971105378,\
        V_max: 5.2866714434994\
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
        U_knots: [0, 0, 0, 0, 0.15215133237491321, 0.30430266474982642, 0.45645399712473961, 0.60860532949965274, 0.77946074467648285, 0.95031615985331297, 0.95031615985331297, 0.95031615985331297, 0.95031615985331297],\
        V_knots: [2.7739444971105378, 2.7739444971105378, 2.7739444971105378, 2.7739444971105378, 2.973752686968413, 3.3041696521871251, 3.6345866174058381, 3.9650035826245502, 4.2954205478432623, 4.6258375130619749, 4.9562544782806874, 5.2866714434994, 5.2866714434994, 5.2866714434994, 5.2866714434994],\
        num_poles_in_U_axis: 9,\
        num_poles_in_V_axis: 11,\
        poles: {\
            u0: [[2798.9455164305973, -564.0107501072988, 712.92358035843279], [2866.0958153498605, -566.86516373773884, 711.34840943413406], [3044.2739190469856, -574.94375338520831, 707.12535142508989], [3333.4582744417507, -590.77166362188404, 700.24959668275471], [3665.75713302945, -615.4012941362887, 692.74829403468834], [3998.0229949721997, -648.56699700760407, 686.12506726381753], [4327.4999409546645, -693.43081146947407, 681.52830937981673], [4661.1563125334014, -750.31560089700122, 678.88560100063648], [4960.9120004544548, -855.80928469976186, 686.86268394446665], [5202.6229458470998, -955.68640583188539, 697.50617166126085], [5229.8220700325737, -1058.8898967817131, 716.93503449971013]],\
            u1: [[2786.8032191605867, -601.09010925355085, 737.9883753135864], [2853.6862794525396, -603.61586285919668, 736.39954051909626], [3031.1574666274264, -610.76173505354575, 732.09513676350753], [3319.028061794223, -624.70616329027621, 724.65478882371872], [3649.9008054113278, -646.38997073254473, 715.68417576931927], [3980.7179363613009, -675.79323128853287, 706.69867073930311], [4309.3170277177051, -716.07548473598888, 698.7806474262444], [4640.6919327410287, -768.13341091857865, 692.03249720688825], [4946.8731591764517, -864.23243353942007, 693.97043948367059], [5178.6558879397608, -960.65911221900387, 701.16669661190952], [5229.8220700325737, -1058.8898967817131, 716.93503449971024]],\
            u2: [[2766.4247543998681, -678.35402023972233, 782.94313479641835], [2832.8601423019977, -680.25817991944859, 781.3189750193975], [3009.1473982313923, -685.6415474059329, 776.84414655063972], [3294.8176626522259, -696.04493646813637, 768.39337717894284], [3623.3108764124177, -712.19483039757267, 756.86139775672586], [3951.7278552909452, -734.45729805037104, 743.77658160836609], [4278.8660407789739, -765.85620379368913, 730.07672949814685], [4606.5177632392015, -808.13530287791457, 716.128098039007], [4923.3883811287624, -885.55311888792244, 707.37267375445549], [5139.302721776191, -972.49694693112633, 707.97292451189492], [5229.8220700325746, -1058.8898967817131, 716.93503449971013]],\
            u3: [[2747.5754465966966, -801.74639499791306, 837.54568985917638], [2813.5997405084795, -802.87355870592353, 835.77311925601578], [2988.8008049665841, -806.04991056540212, 830.82989803882879], [3272.4521423984033, -812.02720047507239, 820.91211099729503], [3598.7941616260487, -821.18862491009565, 806.31547218029459], [3925.1028421397377, -834.1313812710174, 788.60305237665568], [4250.9340789764474, -853.28379131270958, 768.45819007595753], [4575.5202792352102, -880.75855080443625, 746.38567090124025], [4901.9399275821797, -930.71810703772303, 725.39056791537087], [5106.0020356223267, -995.8789437426816, 716.62801515356739], [5229.8220700325737, -1058.8898967817131, 716.93503449971001]],\
            u4: [[2740.4445281314324, -934.41465101791846, 871.60774362345023], [2806.3181426472593, -934.87304933526252, 869.79572560675138], [2981.1227210310235, -936.153968381911, 864.71373114846006], [3264.0367006037541, -938.38746584318721, 854.2579045221712], [3589.64562555168, -941.63883870512734, 838.38796173049457], [3915.3379374428591, -946.41937898357492, 818.58326655697806], [4240.7466357794901, -954.16983632471704, 795.23509233422897], [4564.786565479968, -966.4701744254869, 768.72948485119741], [4894.2701875605471, -989.30103816807582, 740.30903956785914], [5098.4665457017927, -1024.891542423996, 723.72612091198084], [5229.8220700325737, -1058.8898967817131, 716.93503449971013]],\
            u5: [[2745.2199794700018, -1079.8475752233953, 889.71246336390254], [2811.2078257956427, -1079.7841667307814, 887.86601406621128], [2986.317638729231, -1079.5800579820927, 882.69928518864424], [3269.7981296791322, -1078.892369445131, 872.19041039523245], [3596.1201300409402, -1077.4202026996149, 856.32993176465493], [3922.7238808573397, -1075.215069709604, 836.45662901293292], [4248.6133769267626, -1072.281586679879, 812.6185986497627], [4574.7071344169717, -1068.7065351464989, 784.81047010509008], [4900.6294880342921, -1063.946207398111, 753.14221248495573], [5117.4230174915965, -1060.8929598624311, 729.48318711103423], [5229.8220700325746, -1058.8898967817131, 716.93503449971013]],\
            u6: [[2763.641472566735, -1236.8672900609779, 878.32734964323595], [2830.0471221669159, -1236.5128225990291, 876.74552892021154], [3006.2662815759954, -1235.4600587356158, 872.33440719685848], [3291.807410503935, -1233.0667841399429, 863.59225830802131], [3620.4991822808229, -1228.5536562654361, 850.66845400685361], [3949.7636663509697, -1221.0471988772299, 834.50510116516091], [4277.1685928886536, -1208.609134049341, 814.7504254086482], [4608.2902759889994, -1188.738751843091, 790.69677003169159], [4923.0633696314799, -1156.3096283152222, 762.05145347335235], [5166.7407778915504, -1104.5180234720119, 733.61972813707234], [5229.8220700325737, -1058.8898967817131, 716.93503449971013]],\
            u7: [[2786.2386397869227, -1347.7882860844377, 852.88494299262015], [2853.152291179128, -1347.4697580816623, 851.5857089173553], [3030.7188246609157, -1346.480076491978, 847.99542478459432], [3318.763090566038, -1344.110613452445, 841.29061329374701], [3650.2866682587924, -1339.1259816890299, 831.94376328058513], [3982.6463439074778, -1329.6505165273279, 820.5178260704746], [4311.8432986449779, -1312.0720624764849, 806.35273964433884], [4648.5616873057506, -1281.3415942363999, 788.03865190698434], [4950.1687797359164, -1230.771068469166, 765.57124495607832], [5222.5199717803953, -1139.2022947231969, 735.12273257278275], [5229.8220700325737, -1058.8898967817131, 716.93503449971013]],\
            u8: [[2799.9999865623431, -1401.9166801004526, 833.9211672781546], [2867.2223681697769, -1401.7258489284618, 832.83717033423216], [3045.60753873161, -1401.0640106744295, 829.86203204549713], [3335.1727105876639, -1399.2245059146978, 824.59098851264946], [3668.410222389793, -1394.6622793516422, 817.66231991812504], [4002.6310411514191, -1384.877086509435, 809.39114981582463], [4332.9098796233584, -1365.33744331056, 798.95536965762994], [4672.9560726785676, -1329.5306692263059, 784.53996145231145], [4966.6172503371636, -1270.390219343657, 766.42653008969262], [5255.8298617696564, -1157.573034130799, 735.51290549326927], [5229.8220700325737, -1058.8898967817131, 716.93503449971013]]\
        }\
    }\
  }";

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;

  // Test 1.
  {
    const double u = 0.5;
    const double v = 3;
    //
    const xyz P_ref(2966.1562033533373, -975.99025344537256, 868.77888598126765);

    // Evaluate.
    xyz P;
    surf->Eval(u, v, P);

    // Check.
    if ( (P - P_ref).Modulus() > eps )
      return res.failure();
  }

  // Test 2.
  {
    const double u = 0.5;
    const double v = 5;
    //
    const xyz P_ref(4936.6672840131978, -1017.8903156478241, 740.14824482133531);

    // Evaluate.
    xyz P;
    surf->Eval(u, v, P);

    // Check.
    if ( (P - P_ref).Modulus() > eps )
      return res.failure();
  }

  // Test 3.
  {
    const double u = 0.95031615985331297; // U_max.
    const double v = 5.2866714434994; // V_max.
    //
    const xyz P_ref(5229.8220700325737, -1058.8898967817131, 716.93503449971013);

    // Evaluate.
    xyz P;
    surf->Eval(u, v, P);

    // Check.
    if ( (P - P_ref).Modulus() > eps )
      return res.failure();
  }

  // Test 4.
  {
    const double u = 0.96; // > U_max.
    const double v = 5.29; // > V_max.
    //
    const xyz P_ref(5228.8089992117957, -1055.7995537824584, 716.37072890072977);

    // Evaluate.
    xyz P;
    surf->Eval(u, v, P);

    // Check.
    if ( (P - P_ref).Modulus() > eps )
      return res.failure();
  }

  return res.success();
}
