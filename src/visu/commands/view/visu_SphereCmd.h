//-----------------------------------------------------------------------------
// Created on: 05 September 2014
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

#ifndef visu_SphereCmd_HeaderFile
#define visu_SphereCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorSphereSurface.h>
#include <mobius/visu_ViewCmd.h>

// geom includes
#include <mobius/geom_SphereSurface.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Draws sphere.
class visu_SphereCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker  [in] instance of Picker.
  visu_SphereCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                 const t_ptr<visu_Picker>&      Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_SphereCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  virtual std::string Name() const
  {
    return "Sphere";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    // Read translation
    const double pos_x = this->Arg<double>(1, 0.0);
    const double pos_y = this->Arg<double>(2, 0.0);
    const double pos_z = this->Arg<double>(3, 0.0);

    // Read axis of rotation
    const double roto_x = this->Arg<double>(4, 0.0);
    const double roto_y = this->Arg<double>(5, 0.0);
    const double roto_z = this->Arg<double>(6, 1.0);

    // Read angle of rotation
    const double roto_ang = this->Arg<double>(7, 0.0);

    // Prepare quaternion representing rotation around axis
    core_Quaternion Qn(t_xyz(roto_x, roto_y, roto_z), roto_ang);

    // Prepare transformation
    core_IsoTransform T( Qn,
                         t_xyz(pos_x, pos_y, pos_z) );

    // Create sphere
    t_ptr<geom_SphereSurface>
      S = new geom_SphereSurface( this->Arg<double>(0, 1.0), T );

    // Actor for sphere
    t_ptr<visu_Actor> actorSphere = new visu_ActorSphereSurface(S);

    // Add Actor to Scene
    this->Scene()->Add(actorSphere);

    // Install axes
    this->Scene()->InstallAxes();

    return true;
  }

};

}

#endif
