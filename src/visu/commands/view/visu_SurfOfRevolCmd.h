//-----------------------------------------------------------------------------
// Created on: 11 January 2023
//-----------------------------------------------------------------------------
// Copyright (c) 2023-present, Sergey Slyadnev
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

#ifndef visu_SurfOfRevolCmd_HeaderFile
#define visu_SurfOfRevolCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorSurfaceOfRevolution.h>
#include <mobius/visu_ViewCmd.h>

// geom includes
#include <mobius/geom_SurfaceOfRevolution.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Command for construction of surfaces of revolution.
class visu_SurfOfRevolCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker [in] instance of Picker.
  visu_SurfOfRevolCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                      const t_ptr<visu_Picker>&      Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_SurfOfRevolCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  inline virtual std::string Name() const
  {
    return "Surface of revolution";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    std::cout << this->Name().c_str() << std::endl;

    // generatrix
    t_ptr<t_bcurve> c = this->curve();
    //
    t_ptr<visu_ActorBSplCurve>
      c_actor = new visu_ActorBSplCurve(c,
                                        visu_ColorRGB<GLubyte>(255, 0, 0),
                                        false,
                                        true);

    // surface of revolution
    t_ptr<t_surfRevol> surf = new t_surfRevol(c, t_axis( t_xyz::O(), t_xyz::OX() ) );
    //
    t_ptr<visu_ActorSurfaceOfRevolution>
      surf_actor = new visu_ActorSurfaceOfRevolution(surf);

    this->Scene()->Add( c_actor.Access() );
    this->Scene()->Add( surf_actor.Access() );

    return true;
  }

private:

  //! Constructs c(u).
  //! \return constructed curve.
  t_ptr<t_bcurve> curve() const
  {
    const double U[] = {0, 0, 0, 1, 2, 2.5, 3, 4, 5, 5, 5};
    const int p = 2;

    const t_xyz Poles[] = { t_xyz( 0.0, 5.2, 0.0),
                            t_xyz( 1.0, 2.0, 0.0),
                            t_xyz( 3.0, 2.2, 0.0),
                            t_xyz( 5.0, 0.2, 0.0),
                            t_xyz( 8.0, 2.2, 0.0),
                            t_xyz(10.0, 1.8, 0.0),
                            t_xyz(11.0, 2.2, 0.0),
                            t_xyz(14.0, 7.9, 0.0) };
    //
    std::vector<t_xyz> PolesVector;
    for ( size_t i = 0; i < sizeof(Poles)/sizeof(t_xyz); ++i )
      PolesVector.push_back(Poles[i]);

    return new t_bcurve(PolesVector, U, sizeof(U)/sizeof(double), p);
  }

};

}

#endif
