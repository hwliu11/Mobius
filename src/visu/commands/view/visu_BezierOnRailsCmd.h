//-----------------------------------------------------------------------------
// Created on: 20 June 2014
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

#ifndef visu_BezierOnRailsCmd_HeaderFile
#define visu_BezierOnRailsCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBezierOnRailsSurface.h>
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ViewCmd.h>

// bspl includes
#include <mobius/bspl_ConstLaw.h>

// geom includes
#include <mobius/geom_BSplineCurve.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Command for construction of Bezier On Rails.
class visu_BezierOnRailsCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker  [in] instance of Picker.
  visu_BezierOnRailsCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                          const t_ptr<visu_Picker>&      Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_BezierOnRailsCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  virtual std::string Name() const
  {
    return "Bezier On Rails";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    std::cout << this->Name().c_str() << std::endl;

    /* =========================
     *  Create necessary curves
     * ========================= */

    // r(u)
    t_ptr<t_bcurve> r = this->curve_r();
    t_ptr<visu_ActorBSplCurve>
      r_actor = new visu_ActorBSplCurve(r,
                                          visu_ColorRGB<GLubyte>(0, 0, 255),
                                          false,
                                          true);

    // q(u)
    t_ptr<t_bcurve> q = this->curve_q();
    t_ptr<visu_ActorBSplCurve>
      q_actor = new visu_ActorBSplCurve(q,
                                          visu_ColorRGB<GLubyte>(0, 255, 0),
                                          false,
                                          true);

    // c(u)
    t_ptr<t_bcurve> c = this->curve_c();
    t_ptr<visu_ActorBSplCurve>
      c_actor = new visu_ActorBSplCurve(c,
                                          visu_ColorRGB<GLubyte>(255, 0, 0),
                                          false,
                                          true);

    /* ================
     *  Create surface
     * ================ */

    // Constant law
    t_ptr<bspl_ConstLaw> CLaw = new bspl_ConstLaw( this->Arg<double>(0, 1.0) );

    // Create surface on rails
    t_ptr<geom_BezierOnRailsSurface>
      S = new geom_BezierOnRailsSurface( r.Access(), c.Access(), q.Access(), CLaw.Access() );
    //
    t_ptr<visu_ActorBezierOnRailsSurface>
      S_actor = new visu_ActorBezierOnRailsSurface(S);

    /* ==============
     *  Adjust scene
     * ============== */

    this->Scene()->Add( r_actor.Access() );
    this->Scene()->Add( q_actor.Access() );
    this->Scene()->Add( c_actor.Access() );
    this->Scene()->Add( S_actor.Access() );

    //this->Scene()->InstallAxes();

    return true;
  }

private:

  //! Constructs r(u).
  //! \return constructed curve.
  t_ptr<t_bcurve> curve_r() const
  {
    const double U[] = {0, 0, 0, 1, 2, 2.5, 3, 4, 5, 5, 5};
    const int p = 2;

    const t_xyz Poles[] = { t_xyz( 0.0, 2.2, 0.0),
                          t_xyz( 1.0, 2.0, 0.0),
                          t_xyz( 3.0, 2.2, 0.0),
                          t_xyz( 5.0, 1.9, 0.0),
                          t_xyz( 8.0, 2.2, 0.0),
                          t_xyz(10.0, 1.8, 0.0),
                          t_xyz(11.0, 2.2, 0.0),
                          t_xyz(14.0, 1.9, 0.0) };
    //
    std::vector<t_xyz> PolesVector;
    for ( size_t i = 0; i < sizeof(Poles)/sizeof(t_xyz); ++i )
      PolesVector.push_back(Poles[i]);

    return new t_bcurve(PolesVector, U, sizeof(U)/sizeof(double), p);
  }

  //! Constructs q(u).
  //! \return constructed curve.
  t_ptr<t_bcurve> curve_q() const
  {
    const double U[] = {0, 0, 0, 0, 1.4, 3.2, 4.3, 5, 5, 5, 5};
    const int p = 3;

    const t_xyz Poles[] = { t_xyz( 0.0, -2.2, 0.0),
                          t_xyz( 1.0, -2.0, 0.0),
                          t_xyz( 3.0, -2.4, 0.0),
                          t_xyz( 5.0, -1.9, 0.0),
                          t_xyz( 8.0, -2.4, 0.0),
                          t_xyz(10.0, -1.8, 0.0),
                          t_xyz(11.0, -2.2, 0.0),
                          t_xyz(14.0, -1.7, 0.0) };
    //
    std::vector<t_xyz> PolesVector;
    for ( size_t i = 0; i < sizeof(Poles)/sizeof(t_xyz); ++i )
      PolesVector.push_back(Poles[i]);

    return new t_bcurve(PolesVector, U, sizeof(U)/sizeof(double), p);
  }

  //! Constructs c(u).
  //! \return constructed curve.
  t_ptr<t_bcurve> curve_c() const
  {
    const double U[] = {0, 0, 0, 1.1, 1.4, 2, 3.5, 4, 5, 5, 5};
    const int p = 2;

    const t_xyz Poles[] = { t_xyz( 0.0,  0.0, -2.0),
                          t_xyz( 1.0,  0.1, -2.7),
                          t_xyz( 3.0, -0.1, -2.2),
                          t_xyz( 5.0, -0.1, -3.2),
                          t_xyz( 8.0, -0.2, -3.1),
                          t_xyz(10.0, -0.1, -2.0),
                          t_xyz(11.0,  0.1, -3.2),
                          t_xyz(14.0,  0.0, -2.0) };
    //
    std::vector<t_xyz> PolesVector;
    for ( size_t i = 0; i < sizeof(Poles)/sizeof(t_xyz); ++i )
      PolesVector.push_back(Poles[i]);

    return new t_bcurve(PolesVector, U, sizeof(U)/sizeof(double), p);
  }

};

}

#endif
