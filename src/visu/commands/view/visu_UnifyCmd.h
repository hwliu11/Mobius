//-----------------------------------------------------------------------------
// Created on: 09 March 2015
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

#ifndef visu_UnifyCmd_HeaderFile
#define visu_UnifyCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ViewCmd.h>

// bspl includes
#include <mobius/bspl_KnotMultiset.h>
#include <mobius/bspl_UnifyKnots.h>

// geom includes
#include <mobius/geom_SectionLine.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Test command on unification of curves.
class visu_UnifyCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker [in] instance of Picker.
  visu_UnifyCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                  const t_ptr<visu_Picker>& Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_UnifyCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  inline virtual std::string Name() const
  {
    return "Unification of curves";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    if ( this->Argc() < 3 )
    {
      std::cout << "Error: b-curve 1, b-curve 2 [, b-curve 3, ...]" << std::endl;
      return false;
    }

    // Collect b-curves
    std::vector<std::string> names;
    std::vector< t_ptr<t_sline> > sections;
    for ( size_t arg = 0; arg < this->Argc(); ++arg )
    {
      // Find target curve in the global context
      t_ptr<t_sline> sct = t_ptr<t_sline>::DownCast( this->CmdRepo()->Vars().FindVariable( this->Arg(arg) ) );
      if ( sct.IsNull() )
      {
        std::cout << "Section with b-curve is expected" << std::endl;
        return false;
      }

      names.push_back( this->Arg(arg) );
      sections.push_back(sct);
    }

    // Collect knot vector
    std::vector< std::vector<double> > U_all;
    for ( size_t c = 0; c < sections.size(); ++c )
    {
      std::vector<double> U = sections[c]->Curve->GetKnots();
      U_all.push_back(U);

      // Dump knots
      std::cout << "Curve " << (c + 1) << ": ";
      for ( size_t j = 0; j < U.size(); ++j )
      {
        std::cout << U[j] << "\t";
      }
      std::cout << std::endl;
    }

    // Compute extension
    bspl_UnifyKnots Unify;
    std::vector< std::vector<double> > X = Unify(U_all);

    // Unify knots
    for ( size_t c = 0; c < sections.size(); ++c )
    {
      const t_ptr<t_bcurve>& crv = sections[c]->Curve;
      crv->RefineKnots(X[c]);
      std::vector<double> U = crv->GetKnots();

      // Dump knots
      std::cout << "Curve [refined] " << (c + 1) << ": ";
      for ( size_t j = 0; j < U.size(); ++j )
      {
        std::cout << U[j] << "\t";
      }
      std::cout << std::endl;

      // Rebind actor
      t_ptr<visu_ActorBSplCurve> crv_actor = new visu_ActorBSplCurve(crv, QrRed, false, false, false);
      this->Scene()->Add(crv_actor.Access(), names[c]);

      // Rebind variable
      this->CmdRepo()->Vars().RegisterVariable( names[c], sections[c].Access() );
    }

    return true;
  }

};

}

#endif
