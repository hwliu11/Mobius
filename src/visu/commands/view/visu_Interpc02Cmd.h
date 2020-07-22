//-----------------------------------------------------------------------------
// Created on: 14 May 2014
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

#ifndef visu_Interpc02Cmd_HeaderFile
#define visu_Interpc02Cmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ActorPositionCloud.h>
#include <mobius/visu_ActorVectorField.h>
#include <mobius/visu_ViewCmd.h>

// geom includes
#include <mobius/geom_PositionCloud.h>
#include <mobius/geom_InterpolateCurve.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Test command on interpolation.
class visu_Interpc02Cmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker  [in] instance of Picker.
  visu_Interpc02Cmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                      const t_ptr<visu_Picker>&      Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_Interpc02Cmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  virtual std::string Name() const
  {
    return "Interpolation 02";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    std::cout << this->Name() << std::endl;

    const bool use_derivs_start  = this->Arg<int>(0, 0) > 0;
    const bool use_derivs_end    = this->Arg<int>(1, 0) > 0;
    const bool use_derivs_start2 = this->Arg<int>(0, 0) == 2;
    const bool use_derivs_end2   = this->Arg<int>(1, 0) == 2;
    const int  DEG               = this->Arg<int>(2, 3);

    t_xyz D0(-80.0, 0.0, 0.0);
    t_xyz Dn(-40.0, 0.0, 0.0);
    t_xyz D20(500, 1000.0, 0.0);
    t_xyz D2n(500, 1000.0, 0.0);

    // Prepare poles for B-spline curve
    /*core_XYZ Ps[] = { core_XYZ( 1.0, 1.0, 0.0 ),
                        core_XYZ( 2.0, 0.0, 0.0 ),
                        core_XYZ( 3.0, 1.0, 0.0 ),
                        core_XYZ( 3.0, 8.0, 0.0 ),
                        core_XYZ( 4.0, 10.0, 0.0 ),
                        core_XYZ( 5.0, 9.0, 0.0 ),
                        core_XYZ( 4.0, 7.0, 0.0 ),
                        core_XYZ( 0.0, 5.0, 0.0 ),
                        core_XYZ( 0.0, 3.0, 0.0 ),
                        core_XYZ( 3.0, 2.0, 0.0 ),
                        core_XYZ( 6.0, 3.0, 0.0 ),
                        core_XYZ( 6.0, 5.0, 0.0 ),
                        core_XYZ( 4.0, 6.0, 0.0 ),
                        core_XYZ( 3.0, 6.0, 0.0 ),
                        core_XYZ( 2.0, 5.0, 0.0 ),
                        core_XYZ( 2.0, 4.0, 0.0 ),
                        core_XYZ( 3.0, 3.0, 0.0 ) };*/
    t_xyz Ps[] = { t_xyz( 1.0, 1.0, 0.0 ),
                 t_xyz( 2.0, 0.0, 0.0 ),
                 t_xyz( 3.0, 1.0, 0.0 ),
                 t_xyz( 3.0, 1.5, 0.0 ),
                 t_xyz( 3.0, 2.0, 0.0 ),
                 t_xyz( 3.0, 3.0, 0.0 ),
                 t_xyz( 3.0, 4.0, 0.0 ),
                 t_xyz( 3.0, 8.0, 0.0 ),
                 t_xyz( 4.0, 10.0, 0.0 ),
                 t_xyz( 5.0, 9.0, 0.0 ),
                 t_xyz( 4.0, 7.0, 0.0 ),
                 t_xyz( 0.0, 5.0, 0.0 ),
                 t_xyz( 0.0, 3.0, 0.0 ),
                 t_xyz( 3.0, 2.0, 0.0 ),
                 t_xyz( 6.0, 3.0, 0.0 ),
                 t_xyz( 6.0, 5.0, 0.0 ),
                 t_xyz( 4.0, 6.0, 0.0 ),
                 t_xyz( 3.0, 6.0, 0.0 ),
                 t_xyz( 2.0, 5.0, 0.0 ),
                 t_xyz( 2.0, 4.0, 0.0 ),
                 t_xyz( 3.0, 3.0, 0.0 ),
                        /*t_xyz( 1.0, 1.0, 0.0 )*/ // the same as 1-st, just for tests
    };

    std::vector<t_xyz> Poles;
    const int numPts = sizeof(Ps)/sizeof(t_xyz);
    for ( int n = 0; n < numPts; ++n )
      Poles.push_back(Ps[n]);

    //----------------------------------------------------------------------------
    t_ptr<t_pcloud> dataCloud = new t_pcloud();
    dataCloud->SetPoints(Poles);
    t_ptr<visu_Actor> dataCloudActor = new visu_ActorPositionCloud( dataCloud, visu_ColorRGB<GLubyte>(255, 255, 0) );
    t_ptr<visu_ActorPositionCloud>::DownCast(dataCloudActor)->SetPointSize(5);

    t_ptr<visu_Actor> ActorInterp2_Uniform,
                        ActorInterp2_ChordLength,
                        ActorInterp2_Centri,
                        ActorInterp2_Poly;

    {
      bspl_ParamsSelection paramsType2 = ParamsSelection_Uniform;
      bspl_KnotsSelection knotsType2 = KnotsSelection_Average;

      geom_InterpolateCurve Interp2(Poles, DEG, paramsType2, knotsType2);
      try
      {
        Interp2.Perform();
      }
      catch ( std::exception ex )
      {
        std::cout << "Exception occurred: " << ex.what() << std::endl;
        return false;
      }

      if ( Interp2.GetErrorCode() != geom_InterpolateCurve::ErrCode_NoError )
      {
        std::cout << "Error: Qr interpolation failed" << std::endl;
        return false;
      }

      // Access constructed B-spline curve
      const t_ptr<t_bcurve>& QrCrv2 = Interp2.GetResult();

      ActorInterp2_Uniform = new visu_ActorBSplCurve(QrCrv2, visu_ColorRGB<GLubyte>(255, 0, 0), false, true);
    }
    {
      bspl_ParamsSelection paramsType2 = ParamsSelection_ChordLength;
      bspl_KnotsSelection knotsType2 = KnotsSelection_Average;

      geom_InterpolateCurve Interp2(Poles, DEG, paramsType2, knotsType2);
      try
      {
        Interp2.Perform();
      }
      catch ( std::exception ex )
      {
        std::cout << "Exception occurred: " << ex.what() << std::endl;
        return false;
      }

      if ( Interp2.GetErrorCode() != geom_InterpolateCurve::ErrCode_NoError )
      {
        std::cout << "Error: Qr interpolation failed" << std::endl;
        return false;
      }

      // Access constructed B-spline curve
      const t_ptr<t_bcurve>& QrCrv2 = Interp2.GetResult();

      ActorInterp2_ChordLength = new visu_ActorBSplCurve(QrCrv2, visu_ColorRGB<GLubyte>(0, 0, 255), false, true);
    }
    {
      bspl_ParamsSelection paramsType2 = ParamsSelection_Centripetal;
      bspl_KnotsSelection knotsType2 = KnotsSelection_Average;

      geom_InterpolateCurve Interp2;
      Interp2.Init(Poles,
                   use_derivs_start  ? D0  : t_xyz(),
                   use_derivs_end    ? Dn  : t_xyz(),
                   use_derivs_start2 ? D20 : t_xyz(),
                   use_derivs_end2   ? D2n : t_xyz(),
                   DEG,
                   paramsType2,
                   knotsType2);

      try
      {
        Interp2.Perform();
      }
      catch ( std::exception ex )
      {
        std::cout << "Exception occurred: " << ex.what() << std::endl;
        return false;
      }

      if ( Interp2.GetErrorCode() != geom_InterpolateCurve::ErrCode_NoError )
      {
        std::cout << "Error: Qr interpolation failed" << std::endl;
        return false;
      }

      // Access constructed B-spline curve
      const t_ptr<t_bcurve>& QrCrv2 = Interp2.GetResult();

      ActorInterp2_Centri = new visu_ActorBSplCurve(QrCrv2, visu_ColorRGB<GLubyte>(0, 255, 0), false, (use_derivs_start || use_derivs_end ? false : true), false);

      this->CmdRepo()->Vars().RegisterVariable( "crv", QrCrv2.Access() );
    }
    //{
    //  bspl_ParamsSelection paramsType2 = ParamsSelection_Centripetal;
    //  bspl_KnotsSelection knotsType2 = KnotsSelection_Average;

    //  geom_InterpolateCurve Interp2(Poles, 1, paramsType2, knotsType2);
    //  try
    //  {
    //    Interp2.Perform();
    //  }
    //  catch ( std::exception ex )
    //  {
    //    std::cout << "Exception occurred: " << ex.what() << std::endl;
    //    return false;
    //  }

    //  if ( Interp2.GetErrorCode() != geom_InterpolateCurve::ErrCode_NoError )
    //  {
    //    std::cout << "Error: Qr interpolation failed" << std::endl;
    //    return false;
    //  }

    //  // Access constructed B-spline curve
    //  const t_ptr<geom_BSplineCurve>& QrCrv2 = Interp2.GetResult();

    //  ActorInterp2_Poly = new visu_ActorBSplCurve(QrCrv2, visu_ColorRGB<GLubyte>(255, 0, 0));
    //}
    //----------------------------------------------------------------------------

    this->Scene()->Add(dataCloudActor);
    this->Scene()->Add(ActorInterp2_Uniform);
    this->Scene()->Add(ActorInterp2_ChordLength);
    this->Scene()->Add(ActorInterp2_Centri, "crv");
    //this->Scene()->Add(ActorInterp2_Poly);

    // Show derivative D1 vectors if any
    if ( use_derivs_start || use_derivs_end )
    {
      t_ptr<geom_VectorField>
        tangency_field = new geom_VectorField( new t_pcloud(Poles) );

      double xMin, xMax, yMin, yMax, zMin, zMax;
      tangency_field->GetCloud()->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
      const double d = ( t_xyz(xMin, yMin, zMin) - t_xyz(xMax, yMax, zMax) ).Modulus();

      if ( use_derivs_start )
        tangency_field->AddVector(0, D0);

      if ( use_derivs_end )
        tangency_field->AddVector(Poles.size() - 1, Dn);

      t_ptr<visu_ActorVectorField>
        tangency_field_actor = new visu_ActorVectorField(tangency_field, QrRed, d / 10.0);

      this->Scene()->Add( tangency_field_actor.Access() );
    }

    // Show derivative D2 vectors if any
    if ( use_derivs_start2 || use_derivs_end2 )
    {
      t_ptr<geom_VectorField>
        d2_field = new geom_VectorField( new t_pcloud(Poles) );

      double xMin, xMax, yMin, yMax, zMin, zMax;
      d2_field->GetCloud()->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
      const double d = ( t_xyz(xMin, yMin, zMin) - t_xyz(xMax, yMax, zMax) ).Modulus();

      if ( use_derivs_start )
        d2_field->AddVector(0, D20);

      if ( use_derivs_end )
        d2_field->AddVector(Poles.size() - 1, D2n);

      t_ptr<visu_ActorVectorField>
        d2_field_actor = new visu_ActorVectorField(d2_field, QrBlue, d / 10.0);

      this->Scene()->Add( d2_field_actor.Access() );
    }

    this->Scene()->InstallAxes();

    return true;
  }

};

}

#endif
