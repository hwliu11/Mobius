//-----------------------------------------------------------------------------
// Created on: 27 February 2015
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

#ifndef visu_PolesCmd_HeaderFile
#define visu_PolesCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ActorBSplSurface.h>
#include <mobius/visu_ViewCmd.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Switches on/off visualization of control polygon.
class visu_PolesCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker [in] instance of Picker.
  visu_PolesCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                  const t_ptr<visu_Picker>& Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_PolesCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  inline virtual std::string Name() const
  {
    return "Control polygon";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    std::cout << this->Name() << std::endl;
    if ( this->Argc() != 2 )
    {
      std::cout << "Error: b-curve/surface name expected" << std::endl;
      return false;
    }

    // Get Qr variable to access data asked for conversion
    std::string         qr_name  = this->Arg<std::string>(0, "#error");
    t_ptr<visu_Actor> qr_actor = this->Scene()->FindActor(qr_name);
    if ( qr_actor.IsNull() )
    {
      std::cout << "Error: there is no actor with name " << qr_name << std::endl;
      return false;
    }

    t_ptr<visu_ActorBSplCurve>
      bCurveActor = t_ptr<visu_ActorBSplCurve>::DownCast( qr_actor.Access() );

    t_ptr<visu_ActorBSplSurface>
      bSurfActor = t_ptr<visu_ActorBSplSurface>::DownCast( qr_actor.Access() );

    if ( bCurveActor.IsNull() && bSurfActor.IsNull() )
    {
      std::cout << "Error: b-curve/surface expected" << std::endl;
      return false;
    }

    const bool on = this->Arg<int>(1, 1) > 0;
    std::cout << "Control poles visualization is " << (on ? "on" : "off") << std::endl;

    if ( !bCurveActor.IsNull() )
      bCurveActor->SetPoles(on);
    else if ( !bSurfActor.IsNull() )
      bSurfActor->SetPoles(on);

    return true;
  }

};

}

#endif
