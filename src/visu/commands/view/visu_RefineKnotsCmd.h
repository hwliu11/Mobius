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

#ifndef visu_RefineKnotsCmd_HeaderFile
#define visu_RefineKnotsCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ViewCmd.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Test command on knot refinement.
class visu_RefineKnotsCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker [in] instance of Picker.
  visu_RefineKnotsCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                        const t_ptr<visu_Picker>& Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_RefineKnotsCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  inline virtual std::string Name() const
  {
    return "Refine knot vector";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    if ( this->Argc() < 2 )
    {
      std::cout << "Error: b-curve name and X refinement vector are expected" << std::endl;
      return false;
    }

    // Resolve arguments
    std::string  name = this->Arg(0);
    std::vector<double> X;
    for ( int i = 1; i < (int) this->Argc(); ++i )
      X.push_back( this->Arg<double>(i, 0.0) );

    // Find target curve in the global context
    t_ptr<t_bcurve> crv = t_ptr<t_bcurve>::DownCast( this->CmdRepo()->Vars().FindVariable(name) );
    if ( crv.IsNull() )
    {
      std::cout << "b-curve is expected" << std::endl;
      return false;
    }

    // Refine knot
    if ( !crv->RefineKnots(X) )
    {
      std::cout << "Knot refinement failed" << std::endl;
      return false;
    }

    const std::vector<t_xyz>& poles = crv->GetPoles();
    for ( size_t p = 0; p < poles.size(); ++p )
    {
      std::cout << "P[" << p << "] = (" << poles[p].X()
                << ", " << poles[p].Y() << ", " << poles[p].Z() << ")" << std::endl;
    }

    const std::vector<double>& knots = crv->GetKnots();
    for ( size_t k = 0; k < knots.size(); ++k )
    {
      std::cout << "U[" << k << "] = (" << knots[k] << std::endl;
    }

    // Rebind actor
    t_ptr<visu_ActorBSplCurve> crv_actor = new visu_ActorBSplCurve(crv, QrRed, false, false, false);
    this->Scene()->Add(crv_actor.Access(), name);

    // Rebind variable
    this->CmdRepo()->Vars().RegisterVariable( name, crv.Access() );
    return true;
  }

};

}

#endif
