//-----------------------------------------------------------------------------
// Created on: 05 March 2015
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
#include <mobius/visu_BSplBasisCmd.h>

// geom includes
#include <mobius/geom_BSplineCurve.h>

//! Executes command.
//! \return true in case of success, false -- otherwise.
bool mobius::visu_BSplBasisCmd::Execute()
{
  std::cout << this->Name() << std::endl;
  if ( this->Argc() != 1 )
  {
    std::cout << "Error: degree or b-curve name expected" << std::endl;
    return false;
  }

  std::string name = this->Arg(0);
  t_ptr<core_OBJECT> obj = this->CmdRepo()->Vars().FindVariable(name);

  std::vector<double> U;
  int nKnots = 0;
  int deg    = 0;
  //
  if ( obj.IsNull() && core::str::is_number(name) )
  {
    deg = core::str::to_number<int>(name);
    std::cout << "Degree is " << deg << std::endl;

    nKnots = 2*(deg + 1);
    U.reserve(nKnots);

    for ( int k = 0; k < deg + 1; ++k )
      U[k] = 0.0;
    for ( int k = deg + 1; k < 2*(deg + 1); ++k )
      U[k] = 1.0;
  }
  else if ( !obj.IsNull() )
  {
    t_ptr<t_bcurve> crv = t_ptr<t_bcurve>::DownCast(obj);
    if ( crv.IsNull() )
    {
      std::cout << "Only b-curves are accepted in this command" << std::endl;
      return false;
    }

    deg = crv->GetDegree();
    std::cout << "Degree of your curve is " << deg << std::endl;

    std::vector<double> U_vec = crv->GetKnots();
    nKnots = (int) U_vec.size();
  }
  else
  {
    std::cout << "We have no idea what you mean when saying '" << name << "'..." << std::endl;
    return false;
  }

  t_ptr<visu_ActorBSplBasis>
    Actor = new visu_ActorBSplBasis(U, deg);

  this->Scene()->Add( Actor.Access() );
  //this->Scene()->InstallAxes();

  return true;
}
