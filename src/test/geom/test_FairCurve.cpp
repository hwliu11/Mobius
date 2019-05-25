//-----------------------------------------------------------------------------
// Created on: 28 June 2018
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
#include <mobius/test_FairCurve.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Geom includes
#include <mobius/geom_FairBCurve.h>

// Standard includes
#include <math.h>

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_FairCurve::runtest(const int          funcID,
                                  const std::string& json,
                                  double             lambdas[4],
                                  double             refStrains[4])
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // Create a curve from the passed JSON.
  t_ptr<t_bcurve> crv = t_bcurve::Instance(json);

  double strain[4] = {0.0, 0.0, 0.0, 0.0};

  // Calculate original strain energy.
  const double initStrain = crv->ComputeStrainEnergy();
  //
  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Initial strain energy is %1."
                                                          << initStrain );

  // Perform fairing.
  for ( int i = 0; i < 4; ++i )
  {
    // Fair.
    geom_FairBCurve fairingTool(crv, lambdas[i], cf->ProgressNotifier, NULL);
    //
    if ( !fairingTool.Perform() )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Fairing failed for lambda %1."
                                                            << lambdas[i]);
      return res.failure();
    }

    // Get faired curve.
    const t_ptr<t_bcurve>& faired = fairingTool.GetResult();

    // Calculate strain energy for the faired curve.
    strain[i] = faired->ComputeStrainEnergy();
    //
    cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Strain energy for lambda %1 is %2."
                                                            << lambdas[i]
                                                            << core::str::to_string<double>(strain[i]) );

    // Basic verification that strain energy should decrease with increasing
    // fairing coefficient.
    if ( i > 0 )
      if ( strain[i] > strain[i-1] )
      {
        cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Strain energy is expected to decrease.");
        return res.failure();
      }
  }

  // Compare against the reference values.
  const double eps     = 1e-5;
  bool         isAllOk = true;
  
  for ( int i = 0; i < 4; ++i )
    if ( fabs(strain[i] - refStrains[i]) > eps )
    {
      cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Expected value %1 is different from actual %2."
                                                             << core::str::to_string<double>(refStrains[i])
                                                             << core::str::to_string<double>(strain[i]) );

      if ( isAllOk ) isAllOk = false;
    }

  SetVarDescr("time", res.time(), ID(), funcID);

  return isAllOk ? res.success() : res.failure();
}

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairCurve::test001(const int funcID)
{
  // JSON definition.
  std::string json =
  "{\
    entity: curve,\
    type: b-curve,\
    continuity: C2,\
    domain: {\
        U_min: 0,\
        U_max: 1\
    },\
    flags: {\
        is_rational: 0,\
        is_periodic: 0,\
        is_closed: 0\
    },\
    properties: {\
        degree: 3,\
        knots: [0, 0, 0, 0, 0.011207418387417345, 0.015610899960775582, 0.020157081640905451, 0.025232203631701988, 0.031050195732165695, 0.037703399378521484, 0.045218600139083187, 0.053595790961246219, 0.062866845039905542, 0.073036989090174601, 0.084099628089549183, 0.096007954297178164, 0.10880766534120474, 0.12248395259811055, 0.13695020816455009, 0.15206969343782023, 0.1677190916127014, 0.1839012915268019, 0.20052768940594534, 0.21761364555647747, 0.23506649201319921, 0.25287639992800115, 0.27096764239276783, 0.28930293919034566, 0.30783274206177069, 0.32652331265702128, 0.34535097396234365, 0.36430194162164237, 0.38338065839119823, 0.40258612821550566, 0.42191121132703624, 0.44134524587505813, 0.46091445563977035, 0.48062622247749093, 0.5004844359760785, 0.52046860859604205, 0.54060691959831353, 0.56092256837718046, 0.58139268900788676, 0.60193682245738811, 0.62245437463075615, 0.64280503044925597, 0.66282064553741771, 0.68234435508862956, 0.70119830344270306, 0.71934394623581321, 0.73674419033805749, 0.75345021806387802, 0.76950550812891017, 0.78492250133053698, 0.79979037748336468, 0.81410493968605435, 0.8279167290377405, 0.84123662867596849, 0.85406755033398307, 0.86641296831200643, 0.87829800523838009, 0.88970107982838853, 0.90058386084711872, 0.91094910289468567, 0.92086962206098155, 0.9304203003393221, 0.9395941502462688, 0.94837651896006647, 0.9567467637967465, 0.96468275719337238, 0.97217358029429946, 0.9792304177068164, 0.98591667620755385, 1, 1, 1, 1],\
        num_poles: 73,\
        poles: [[-47.245171837360445, -13.330112189819705, -7.6956009860000014], [-47.13998263088552, -13.747718724187145, -7.6777633561029477], [-47.090079783256073, -14.091746127152625, -7.615638911611037], [-47.226016646984299, -14.372118885793093, -7.3914466047161929], [-47.394037006045707, -14.469737452331472, -7.1892593365536115], [-47.638521242934459, -14.470028315202315, -6.9397925422363791], [-47.965812043600948, -14.371366666933412, -6.6359646015741109], [-48.386252840410371, -14.163309287323285, -6.2693115157766668], [-48.907732799975278, -13.829561421754871, -5.8329751165375434], [-49.538834929797737, -13.349299606524079, -5.3238615932401103], [-50.274246245629243, -12.71733356104474, -4.722196313510131], [-51.162399358986384, -11.85638822817195, -4.0853796335491728], [-52.166011162501349, -10.785030755884019, -3.3678342947259634], [-53.27768255724687, -9.4721489537518373, -2.5673221681187877], [-54.467764203608567, -7.9137813001384218, -1.6520194308463012], [-55.800012859958713, -5.9714277830957494, -0.76080573166820487], [-57.145386607981983, -3.7279463893641043, 0.24438889307381664], [-58.529526294920082, -1.2663847433093547, 1.3287151417992051], [-59.94731279318836, 1.5021107502355719, 2.4062564254467858], [-61.315235331976012, 4.4266482926799977, 3.6288147008688054], [-62.688457583235078, 7.6904895243818157, 4.7713707820252163], [-63.968438272375579, 11.089564932956142, 6.0633949728342822], [-65.201324978220455, 14.790766541548505, 7.2707313289602871], [-66.321226952046572, 18.633675552467789, 8.5687183621245016], [-67.343144472246578, 22.686248552954737, 9.8326920360790222], [-68.255385508787739, 26.885218223112243, 11.087488613611818], [-69.037106964735543, 31.200376021161009, 12.369108302855022], [-69.717558149806322, 35.640868483280912, 13.577143719019153], [-70.264097630887093, 40.174757453852457, 14.76930044114113], [-70.679912076824692, 44.789643849525596, 15.937293400481119], [-70.957557933393787, 49.492744647390175, 17.050046323353683], [-71.062306113988456, 54.295044713420431, 18.093329348001042], [-70.997671803260332, 59.160158723326262, 19.105090081981988], [-70.746776919228154, 64.089021270054829, 20.072310524992631], [-70.303817060607471, 69.070342568690734, 20.998337864500506], [-69.587538425997806, 74.13228771212745, 21.861707818266673], [-68.638057302365041, 79.222588355890565, 22.69852063076296], [-67.419278568518237, 84.340463623958911, 23.49756842144112], [-65.937652903981189, 89.462986546332317, 24.266291055026628], [-64.114153353970607, 94.596525616799653, 24.990164534092735], [-61.941852385368598, 99.704276052040385, 25.676097084515568], [-59.502363483812758, 104.72508449428291, 26.336813879103985], [-56.844380570805541, 109.61082337336099, 26.972466989559752], [-53.992358705943623, 114.31630334333666, 27.578810982120174], [-51.119287718297358, 118.7836622440544, 28.166441847169676], [-48.362312510777087, 122.9809669524203, 28.743385785467634], [-45.670427308597894, 126.90011052271349, 29.295482723020982], [-43.368594637736734, 130.5384247369349, 29.850953294267772], [-41.218100457535328, 133.92156509523454, 30.377559655408191], [-39.270908086294675, 137.06510747636327, 30.886159996017462], [-37.618548554325784, 139.99847953332565, 31.365685259225884], [-36.017333893531358, 142.72512554142554, 31.821156353546655], [-34.658887714576828, 145.27525777910031, 32.235055006593136], [-33.33607161987419, 147.64385383359112, 32.630444379948244], [-32.136374125793267, 149.8517020352927, 32.977548284698543], [-31.015939551893918, 151.89796229305858, 33.308567977248387], [-29.92189924296007, 153.78439136306091, 33.611077047087825], [-28.913923183947102, 155.52218687972751, 33.892243485654376], [-27.97582769179806, 157.11507451380172, 34.152005036031511], [-27.001316344860598, 158.5483386909612, 34.403768880499044], [-26.131434646589131, 159.84769124122616, 34.631057701727649], [-25.368861400831609, 161.02216482174674, 34.832251133221469], [-24.575993209868784, 162.05599585637316, 35.030794062021364], [-23.794950463751345, 162.96880455216379, 35.211692773890569], [-23.024537877318281, 163.76831779882329, 35.366585279368344], [-22.272092317551856, 164.45826611952899, 35.498844530832407], [-21.553587536415233, 165.04558871306529, 35.609779175966942], [-20.878794580158431, 165.53682241803267, 35.701178640623866], [-20.250886012007211, 165.93937808096214, 35.77443139532992], [-19.674559689088067, 166.26227323506066, 35.832055292765482], [-18.929135957596511, 166.60929909753787, 35.902919364422921], [-18.422039119741562, 166.78148392228982, 35.931423070124978], [-17.865858563935813, 166.86792295686121, 35.938999175999975]]\
    }\
  }";
  //
  double lambdas[4] = {1.e-6, 1.e-4, 1e-2, 1.0};
  //
  double refStrains[4] =
  {
    557466.92495228222,
    226406.56348683822,
    14681.235240821727,
    5.696594462993855
  };

  return runtest(funcID, json, lambdas, refStrains);
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairCurve::test002(const int funcID)
{
  // JSON definition.
  std::string json =
  "{\
    entity: curve,\
    type: b-curve,\
    continuity: C2,\
    domain: {\
        U_min: 0,\
        U_max: 1\
    },\
    flags: {\
        is_rational: 0,\
        is_periodic: 0,\
        is_closed: 0\
    },\
    properties: {\
        degree: 3,\
        knots: [0, 0, 0, 0, 0.086552659882147165, 0.12991610337519779, 0.17335320155205217, 0.21685563217354173, 0.26042585254349176, 0.30407265714329168, 0.34777085278678926, 0.39149953433338203, 0.43523418673002129, 0.47897080289576599, 0.52270370522331799, 0.56640842709462558, 0.61006420615718915, 0.65364724150077347, 0.6971411482848936, 0.74053183264471611, 0.78381534587472068, 0.82699997530461111, 0.87012510506088203, 0.91327212141610736, 1, 1, 1, 1],\
        num_poles: 24,\
        poles: [[-47.245171837360445, -13.330112189819705, -7.6956009860000014], [-48.543230124118843, -13.306669608460778, -9.0548748651367035], [-50.459513200569567, -13.369730780838193, -11.105462588842185], [-53.120158946442487, -12.996104686238775, -13.823850023214577], [-55.150254257225164, -12.568772592486884, -15.862474062919576], [-57.203359871504929, -11.986234017726066, -17.895288576625656], [-59.270272317910752, -11.31256769219627, -19.914758239378756], [-61.353784624428712, -10.384652377421357, -21.954429387420479], [-63.412427061251812, -9.8681941325851206, -23.887130582565415], [-65.484887191100427, -8.6415701682777151, -25.946336002597189], [-67.515871836644521, -7.9214371451977916, -27.932877743490458], [-69.579986873492558, -6.6484848190495196, -29.929327424093149], [-71.587490412051366, -5.7394937592878108, -31.921845681675919], [-73.587957963254155, -4.8105535030404427, -33.912492469390827], [-75.587001806797687, -3.5650858465993895, -35.901074175424633], [-77.550199361145332, -2.7154826020692231, -37.890802604771608], [-79.506697712655722, -1.9614739408569175, -39.881664289945746], [-81.455999191167351, -0.95767986837677421, -41.871399498463624], [-83.393251263788315, -0.60519666694352103, -43.865764450167248], [-85.334615926858959, 0.032062381828010633, -45.85932051207741], [-87.273965252953516, -0.12444161078877045, -47.859727806027777], [-89.874234912535812, -0.11430251061433312, -50.526657644448242], [-91.812683725642572, -1.3724906661027914, -52.533684302239614], [-93.087872209687532, -1.8918153723992683, -53.83992385900001]]\
    }\
  }";
  //
  double lambdas[4] = {1.e-6, 1.e-4, 1e-2, 1.0};
  //
  double refStrains[4] =
  {
    8157.8382702687977,
    4078.6390239309244,
    53.898272257029852,
    0.016608023546196223
  };

  return runtest(funcID, json, lambdas, refStrains);
}

//-----------------------------------------------------------------------------

//! Test scenario 003.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairCurve::test003(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // JSON definition.
  std::string json =
  "{\
    entity: curve,\
    type: b-curve,\
    continuity: C1,\
    domain: {\
        U_min: 0.059518850536300003,\
        U_max: 1.21316629075\
    },\
    flags: {\
        is_rational: 0,\
        is_periodic: 0,\
        is_closed: 0\
    },\
    properties: {\
        degree: 14,\
        knots: [0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.059518850536300003, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.191470268773, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.38307641587800001, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.47857483597299999, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.57431365048100003, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.67031188624000004, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 0.958197177023, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075, 1.21316629075],\
        num_poles: 93,\
        poles: [[-28.217986338620001, 15.126107877358001, -32.379999161000001], [-28.21798313871, 15.126106162064, -31.89274671842], [-28.217342455920001, 15.125762727326, -31.405495918620002], [-28.216165781579999, 15.125131977012, -30.918246680359999], [-28.214550415049999, 15.124266068063999, -30.430998900100001], [-28.212577984300001, 15.123208756513, -29.943752387610001], [-28.21031148746, 15.121993812556999, -29.456506853210001], [-28.20779841753, 15.120646694065, -28.969261919659999], [-28.20507756104, 15.119188192943, -28.482017165670001], [-28.202187024099999, 15.117638735219, -27.994772163029999], [-28.199171080719999, 15.116022054205001, -27.50752651965], [-28.196083398990002, 15.114366918101, -27.020279893070001], [-28.192984232579999, 15.112705625745001, -26.533031977589999], [-28.189929142939999, 15.111067960497, -26.045782441579998], [-28.182616226219999, 15.107147908729999, -24.850994581759998], [-28.178457062970001, 15.104918410451001, -24.14347069494], [-28.175015785439999, 15.103073731155, -23.435893896660001], [-28.172852057099998, 15.101913875596001, -22.72833859568], [-28.172520704709999, 15.101736255836, -22.020828120680001], [-28.174570080420001, 15.102834813244, -21.313317346510001], [-28.17954751956, 15.105502944155001, -20.605750468989999], [-28.188005302610001, 15.110036695681, -19.89812385386], [-28.200503073730001, 15.116736062254001, -19.190511755300001], [-28.217605452410002, 15.125903705240001, -18.48304091975], [-28.23987608949, 15.137841766847, -17.77582791919], [-28.267872198500001, 15.152848938505, -17.06892126864], [-28.302146143489999, 15.17122131218, -16.36231826773], [-28.363756636809999, 15.204247303371, -15.30419561465], [-28.38594945993, 15.216143653197999, -14.95234166797], [-28.409869375820001, 15.228965802256999, -14.6006133154], [-28.435543898230002, 15.242728499357, -14.24903107828], [-28.463000547499998, 15.257446496178, -13.897615480440001], [-28.492266831670001, 15.273134539060999, -13.54638704115], [-28.523370277560002, 15.289807382477999, -13.19536628659], [-28.556338388050001, 15.307479770578, -12.84457373417], [-28.591198691710002, 15.32616645853, -12.49402991068], [-28.627978693839999, 15.345882191579999, -12.14375533444], [-28.666705917520002, 15.366641722499001, -11.793770530230001], [-28.707407874449999, 15.388459799289, -11.444096018690001], [-28.750112082329998, 15.411351172432999, -11.09475232264], [-28.83969263741, 15.459370372805999, -10.395889100630001], [-28.88657921655, 15.484503684762, -10.04637134577], [-28.935403597339999, 15.51067574552, -9.6971945542039997], [-28.986063583749999, 15.537831773080001, -9.3483465808039998], [-29.03845697905, 15.565916986134001, -8.9998152802739995], [-29.092481587679998, 15.594876602000999, -8.6515885076019998], [-29.14803521236, 15.624655840177001, -8.3036541173899998], [-29.205015657970002, 15.655199917300999, -7.9559999646810002], [-29.263320727100002, 15.686454053166999, -7.6086139040890002], [-29.32284822439, 15.718363464616001, -7.2614837905799998], [-29.383495952920001, 15.750873370835, -6.914597478878], [-29.445161716769999, 15.783928989433999, -6.5679428238439996], [-29.507743319509999, 15.817475538913, -6.2215076802790001], [-29.634705590900001, 15.885533017929999, -5.5281139603240002], [-29.699090675000001, 15.920046314401, -5.18115650926], [-29.764308312120001, 15.955005896603, -4.8344128775659998], [-29.830372998360001, 15.990419535356001, -4.487888393195], [-29.897299227120001, 16.026294999244001, -4.1415883836590002], [-29.96510149669, 16.062640060873999, -3.7955181772579998], [-30.033794298130001, 16.099462486907999, -3.4496831011440001], [-30.1033921314, 16.136770051359001, -3.1040884838610001], [-30.173909487229999, 16.174570520555999, -2.758739652529], [-30.24536086446, 16.212871667647999, -2.4136419355030001], [-30.317760755889999, 16.251681260651999, -2.0688006602329998], [-30.391123658110001, 16.291007070831999, -1.724221154723], [-30.465464065740001, 16.330856867754999, -1.3799087467010001], [-30.766707837609999, 16.492337057585001, -0.0041409444351050003], [-31.00152082428, 16.618207363324, 1.0251545789109999], [-31.245498874820001, 16.748990554814998, 2.0517490732930002], [-31.498781215339999, 16.884761263738, 3.0755076110689998], [-31.761516401409999, 17.025599122024001, 4.096262783167], [-32.033909446540001, 17.171614027069001, 5.1137848225200004], [-32.316217434359999, 17.322943785842, 6.1278128483299996], [-32.608721366479998, 17.479739029550998, 7.1380937024040003], [-32.911695189539998, 17.642146604118, 8.1443966596480006], [-33.225385696339998, 17.810298803098, 9.146494040516], [-33.550010244070002, 17.984312138619, 10.144119571509], [-33.885771992850003, 18.164295514254, 11.136938088012], [-34.232885544859997, 18.350363966126999, 12.124581980701], [-34.909297436049997, 18.712951115783, 13.976772845033], [-35.23604615539, 18.88810310285, 14.84248426003], [-35.572533990449998, 19.068475693180002, 15.703487066403], [-35.9154621793, 19.252300602702, 16.56101226126], [-36.278487656099998, 19.446898560396999, 17.408850501593001], [-36.625129136730003, 19.632713961482001, 18.263135781169002], [-37.025867965730001, 19.847527969209999, 19.091966493451], [-37.384890232289997, 20.039980027700999, 19.938074873133001], [-37.795526253159998, 20.260099374847002, 20.760094628998999], [-38.189605745149997, 20.471343680164001, 21.587498984534999], [-38.604112408600002, 20.693537865972001, 22.404092809721998], [-39.023863603439999, 20.918543356512998, 23.216443298034001], [-39.453024189099999, 21.148592702927999, 24.022520412782001], [-39.890637057219998, 21.383172852354001, 24.822628488732001]]\
    }\
  }";
  //
  double lambdas[4] = {1.e-6, 1.e-4, 1e-2, 1.0};
  //
  double refStrains[4] =
  {
    871.96998138905803,
    738.40080786367969,
    288.54715576416146,
    0.22093998743026982
  };

  return runtest(funcID, json, lambdas, refStrains);
}
