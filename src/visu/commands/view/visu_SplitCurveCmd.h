//-----------------------------------------------------------------------------
// Created on: 06 March 2015
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

#ifndef visu_SplitCurveCmd_HeaderFile
#define visu_SplitCurveCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ViewCmd.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Test command on curve splitting.
class visu_SplitCurveCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker [in] instance of Picker.
  visu_SplitCurveCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                       const t_ptr<visu_Picker>& Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_SplitCurveCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  inline virtual std::string Name() const
  {
    return "Split curve";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    if ( this->Argc() < 2 )
    {
      std::cout << "Error: b-curve name and parameter value are" << std::endl;
      return false;
    }

    // Resolve arguments
    std::string  name = this->Arg(0);
    const double    u = this->Arg<double>(1, 0.0);

    // Find target curve in the global context
    t_ptr<t_bcurve> crv = t_ptr<t_bcurve>::DownCast( this->CmdRepo()->Vars().FindVariable(name) );
    if ( crv.IsNull() )
    {
      std::cout << "b-curve is expected" << std::endl;
      return false;
    }

    // Split curve
    std::vector< t_ptr<t_bcurve> > slices;
    if ( !crv->Split(u, slices) )
    {
      std::cout << "Splitting failed" << std::endl;
      return false;
    }

    for ( size_t s = 0; s < slices.size(); ++s )
    {
      std::string slice_name = name + "_" + core::str::to_string(s + 1);

      // Rebind actor
      t_ptr<visu_ActorBSplCurve> crv_actor = new visu_ActorBSplCurve(slices[s], s == 0 ? QrBlue : QrYellow, false, false, false);
      this->Scene()->Add( crv_actor.Access(), slice_name);

      // Rebind variable
      this->CmdRepo()->Vars().RegisterVariable( slice_name, slices[s].Access() );
    }

    return true;
  }

};

}

#endif
