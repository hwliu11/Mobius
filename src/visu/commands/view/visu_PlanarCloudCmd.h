//-----------------------------------------------------------------------------
// Created on: 24 December 2014
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

#ifndef visu_PlanarCloudCmd_HeaderFile
#define visu_PlanarCloudCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorPositionCloud.h>
#include <mobius/visu_ViewCmd.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Planar cloud.
class visu_PlanarCloudCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker [in] instance of Picker.
  visu_PlanarCloudCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                        const t_ptr<visu_Picker>& Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_PlanarCloudCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  inline virtual std::string Name() const
  {
    return "Planar test cloud";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    std::cout << this->Name() << std::endl;

    // Build cloud
    t_ptr<t_pcloud> Cloud = new t_pcloud();
    for ( int i = -10; i <= 10; ++i )
      for ( int j = -10; j <= 10; ++j )
        Cloud->AddPoint( t_xyz(i, j, 0.0) );

    // Prepare Actor
    visu_ColorRGB<GLubyte> color;
    visu_ColorSelector::ColorByIndex_d( this->Arg<int>(1, 3), color );
    t_ptr<visu_ActorPositionCloud> Actor = new visu_ActorPositionCloud(Cloud, color);
    Actor->SetPointSize( (GLfloat) this->Arg<int>(0, 2) );

    /* ================
     *  Populate scene
     * ================ */

    this->Scene()->Add( Actor.Access() );
    this->Scene()->InstallAxes();

    return true;
  }

};

}

#endif
