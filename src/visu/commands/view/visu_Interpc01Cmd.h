//-----------------------------------------------------------------------------
// Created on: 30 April 2014
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

#ifndef visu_Interpc01Cmd_HeaderFile
#define visu_Interpc01Cmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ActorPositionCloud.h>
#include <mobius/visu_ViewCmd.h>

// geom includes
#include <mobius/geom_PositionCloud.h>
#include <mobius/geom_InterpolateCurve.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Test command on interpolation.
class visu_Interpc01Cmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker  [in] instance of Picker.
  visu_Interpc01Cmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                      const t_ptr<visu_Picker>&      Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_Interpc01Cmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  virtual std::string Name() const
  {
    return "Interpolation 01";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    std::cout << "Interpolation sample #1" << std::endl;

    const int DEG = this->Arg<int>(0, 3);

    // Prepare poles for B-spline curve
    t_xyz P2( 1.0,  1.0, 0.0 );
    t_xyz P4( 8.0, -2.0, 0.0 );
    t_xyz P5( 15.0, -6.0, 0.0 );
    t_xyz P6( 16.0, -8.0, 0.0 );
    t_xyz P7( 16.5, -7.0, 0.0 );
    t_xyz P8( 52.0, -1.0, 0.0 );
    std::vector<t_xyz> Poles1;
    Poles1.push_back(P2);
    Poles1.push_back(P4);
    Poles1.push_back(P5);
    Poles1.push_back(P6);
    Poles1.push_back(P7);
    Poles1.push_back(P8);

    //----------------------------------------------------------------------------
    t_ptr<t_pcloud> dataCloud = new t_pcloud();
    dataCloud->SetPoints(Poles1);
    t_ptr<visu_Actor> dataCloudActor = new visu_ActorPositionCloud( dataCloud, visu_ColorRGB<GLubyte>(255, 255, 0) );
    t_ptr<visu_ActorPositionCloud>::DownCast(dataCloudActor)->SetPointSize(8);

    bspl_ParamsSelection paramsType = ParamsSelection_Centripetal;
    bspl_KnotsSelection knotsType = KnotsSelection_Average;

    geom_InterpolateCurve Interp(Poles1, DEG, paramsType, knotsType);
    try
    {
      Interp.Perform();
    }
    catch ( std::exception ex )
    {
      std::cout << "Exception occurred: " << ex.what() << std::endl;
      return false;
    }

    if ( Interp.GetErrorCode() != geom_InterpolateCurve::ErrCode_NoError )
    {
      std::cout << "Error: Qr interpolation failed" << std::endl;
      return false;
    }

    // Access constructed B-spline curve
    const t_ptr<t_bcurve>& QrCrv = Interp.GetResult();

    t_ptr<visu_Actor> ActorInterp = new visu_ActorBSplCurve(QrCrv);
    //----------------------------------------------------------------------------

    //----------------------------------------------------------------------------
    t_ptr<t_pcloud> dataCloud2 = new t_pcloud();
    dataCloud2->SetPoints(Poles1);
    t_ptr<visu_Actor> dataCloudActor2 = new visu_ActorPositionCloud( dataCloud2, visu_ColorRGB<GLubyte>(255, 255, 0) );
    t_ptr<visu_ActorPositionCloud>::DownCast(dataCloudActor2)->SetPointSize(5);

    t_ptr<visu_Actor> ActorInterp2_Uniform,
                        ActorInterp2_ChordLength,
                        ActorInterp2_Centri,
                        ActorInterp2_Poly;

    {
      bspl_ParamsSelection paramsType2 = ParamsSelection_Uniform;
      bspl_KnotsSelection knotsType2 = KnotsSelection_Average;

      geom_InterpolateCurve Interp2(Poles1, DEG, paramsType2, knotsType2);
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

      ActorInterp2_Uniform = new visu_ActorBSplCurve(QrCrv2);
    }
    {
      bspl_ParamsSelection paramsType2 = ParamsSelection_ChordLength;
      bspl_KnotsSelection knotsType2 = KnotsSelection_Average;

      geom_InterpolateCurve Interp2(Poles1, DEG, paramsType2, knotsType2);
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

      ActorInterp2_ChordLength = new visu_ActorBSplCurve(QrCrv2, visu_ColorRGB<GLubyte>(255, 0, 0), false, true);
    }
    {
      bspl_ParamsSelection paramsType2 = ParamsSelection_Centripetal;
      bspl_KnotsSelection knotsType2 = KnotsSelection_Average;

      geom_InterpolateCurve Interp2(Poles1, DEG, paramsType2, knotsType2);
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

      ActorInterp2_Centri = new visu_ActorBSplCurve(QrCrv2, visu_ColorRGB<GLubyte>(0, 255, 0), false, true);
    }
    {
      bspl_ParamsSelection paramsType2 = ParamsSelection_Centripetal;
      bspl_KnotsSelection knotsType2 = KnotsSelection_Average;

      geom_InterpolateCurve Interp2(Poles1, 1, paramsType2, knotsType2);
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

      ActorInterp2_Poly = new visu_ActorBSplCurve(QrCrv2, visu_ColorRGB<GLubyte>(255, 0, 0));
    }
    //----------------------------------------------------------------------------

    this->Scene()->Add(dataCloudActor2);
    this->Scene()->Add(ActorInterp2_ChordLength);
    this->Scene()->Add(ActorInterp2_Centri);
    //this->Scene()->Add(ActorInterp2_Poly);

    return true;
  }

};

}

#endif
