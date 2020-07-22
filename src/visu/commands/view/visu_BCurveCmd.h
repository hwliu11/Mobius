//-----------------------------------------------------------------------------
// Created on: 19 December 2014
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

#ifndef visu_BCurveCmd_HeaderFile
#define visu_BCurveCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBSplCurve.h>
#include <mobius/visu_ViewCmd.h>

// geom includes
#include <mobius/geom_BSplineCurve.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Test command on creation of B-spline curve.
class visu_BCurveCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker  [in] instance of Picker.
  visu_BCurveCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                   const t_ptr<visu_Picker>&      Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_BCurveCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  virtual std::string Name() const
  {
    return "B-spline curve";
  }

public:

  //! Executes command.
  //! \return true in case of success, false -- otherwise.
  virtual bool Execute()
  {
    std::cout << this->Name() << std::endl;

    const std::vector<std::string>& args = this->Arguments();
    if ( args.size() < 8 )
    {
      std::cout << "Invalid number of arguments" << std::endl;
      return false;
    }

    /* ====================================
     *  Construct Quaoar 3D B-spline curve
     * ==================================== */

    if ( args[0] != "-knots" )
    {
      std::cout << "Expected -knots key not found" << std::endl;
      return false;
    }

    int nextIdx = 1;

    // Read knots
    bool degDetected = false, stop = false;
    std::vector<double> U;
    do
    {
      if ( args[nextIdx] == "-degree" )
        stop = degDetected = true;
      else
        U.push_back( core::str::to_number<double>(args[nextIdx]) );

      ++nextIdx;
      if ( nextIdx == (int) (args.size() - 1) )
        stop = true;
    }
    while ( !stop );

    if ( !degDetected )
    {
      std::cout << "Expected -degree key not found" << std::endl;
      return false;
    }

    // Read degree
    const int p = core::str::to_number<int>(args[nextIdx++]);

    if ( args[nextIdx++] != "-poles" )
    {
      std::cout << "Expected -poles key not found" << std::endl;
      return false;
    }

    // Read poles
    stop = false;
    std::vector<t_xyz> Poles;
    do
    {
      const double x = core::str::to_number<double>(args[nextIdx++]);
      const double y = core::str::to_number<double>(args[nextIdx++]);
      const double z = core::str::to_number<double>(args[nextIdx++]);

      t_xyz P(x, y, z);
      Poles.push_back(P);

      if ( nextIdx == (int) args.size() )
        stop = true;
    }
    while ( !stop );

    /* ======================================
     *  Check control equation m = n + p + 1
     * ====================================== */

    const int m = (int) U.size() - 1;
    const int n_plus_p_plus_1 = (int) Poles.size() + p;

    if ( m != n_plus_p_plus_1 )
    {
      std::cout << "Error: m = n + p + 1 is not conformed" << std::endl;
      std::cout << "\tm is " << m << std::endl;
      std::cout << "\tn is " << Poles.size() << std::endl;
      std::cout << "\tn + p + 1 is " << n_plus_p_plus_1 << std::endl;
      return false;
    }

    /* ======================
     *  Build B-spline curve
     * ====================== */

    // Construct B-spline curve
    t_ptr<t_bcurve> QrCrv = new t_bcurve(Poles, U, p);

    // Actor
    t_ptr<visu_Actor> QrCrvActor = new visu_ActorBSplCurve(QrCrv, QrWhite, false, false);

    //-------------------------------------------------------------------------
    // Prepare scene
    //-------------------------------------------------------------------------

    this->Scene()->Add(QrCrvActor);
    //this->Scene()->InstallAxes();

    return true;
  }

};

}

#endif
