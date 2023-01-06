//-----------------------------------------------------------------------------
// Created on: 05 January 2023
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
#include <mobius/geom_ReadAstra.h>

// Standard includes
#include <fstream>

using namespace mobius;

//-----------------------------------------------------------------------------

#define Astra_CurveArraySize 7

//-----------------------------------------------------------------------------

namespace {

  //! Curve DTO.
  struct t_curveData
  {
    std::string name; //!< Curve name.
    int         npts; //!< Number of points.

    //! Default ctor.
    t_curveData() : npts(0) {}
  };

  //! Checks if the passed line tokens represent a curve.
  bool IsCurve(const std::vector<std::string>& tokens,
               t_curveData&                    data)
  {
    if ( tokens.size() != Astra_CurveArraySize )
      return false;

    if ( tokens[1] != core::str::to_string<int>(Astra_CurveArraySize) )
      return false;

    data = t_curveData();
    data.name = tokens[0];
    data.npts = core::str::extract_int(tokens[2]);
    return true;
  }
}

//-----------------------------------------------------------------------------

bool geom_ReadAstra::Perform(const std::string& filename)
{
  std::ifstream FILE(filename);
  //
  if ( !FILE.is_open() )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot open ASTRA file.");
    return false;
  }

  enum Mode
  {
    Mode_Scan = 1,
    Mode_Curve
  };

  // Loop over the file.
  ::t_curveData curveData;
  Mode          mode = Mode_Scan;
  //
  while ( !FILE.eof() )
  {
    char str[1024];
    FILE.getline(str, 1024);

    // Save tokens to vector.
    std::vector<std::string> tokens;
    std::istringstream iss(str);
    std::copy( std::istream_iterator<std::string>(iss),
               std::istream_iterator<std::string>(),
               std::back_inserter< std::vector<std::string> >(tokens) );

    if ( !tokens.size() )
      continue;

    if ( ::IsCurve(tokens, curveData) )
    {
      m_progress.SendLogMessage(MobiusInfo(Normal) << "Next curve is '%1'."
                                                   << curveData.name);

      mode = Mode_Curve;
      continue;
    }

    // TODO: NYI

  } // Until EOF.

  FILE.close();
  return true;
}
