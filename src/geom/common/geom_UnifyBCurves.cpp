//-----------------------------------------------------------------------------
// Created on: 26 December 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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
#include <mobius/geom_UnifyBCurves.h>

// BSpl includes
#include <mobius/bspl_UnifyKnots.h>

//-----------------------------------------------------------------------------

bool mobius::geom_UnifyBCurves::AreCompatible() const
{
  if ( m_curves.size() < 2 )
  {
    m_progress.SendLogMessage(MobiusWarn(Normal) << "Not enough curves.");
    return true;
  }

  // Check compatibility of curves.
  bool areCompatible = true;
  int  ref_degree    = 0;
  std::vector<double> ref_U;
  //
  for ( int c = 0; c < int( m_curves.size() ); ++c )
  {
    const t_ptr<t_bcurve>& crv = m_curves[c];
    //
    if ( crv.IsNull() )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Null curve passed (index %1)." << c);
      return false;
    }

    if ( c == 0 )
    {
      ref_degree = crv->GetDegree();
      ref_U      = crv->GetKnots();
    }
    else
    {
      // Check if the second, the third, etc. curves are of the same degree
      // as the first one.
      if ( crv->GetDegree() != ref_degree )
      {
        areCompatible = false;
        break;
      }

      // Check if the second, the third, etc. curves are have the same knot
      // vectors as the first one.
      const std::vector<double>& curr_U = crv->GetKnots();
      //
      if ( curr_U != ref_U )
      {
        areCompatible = false;
        break;
      }
    }
  }

  return areCompatible;
}

//-----------------------------------------------------------------------------

bool mobius::geom_UnifyBCurves::Perform()
{
  if ( m_curves.size() < 2 )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Not enough curves.");
    return false;
  }

  // Check compatibility of curves.
  bool areCompatible = this->AreCompatible();

  // Now if the curves are not compatible, it is time to make them such.
  if ( !areCompatible )
  {
    // Normalize and collect knot vectors.
    std::vector< std::vector<double> > U_all;
    for ( size_t c = 0; c < m_curves.size(); ++c )
    {
      // Normalize.
      m_curves[c]->ReparameterizeLinear(0.0, 1.0);

      // Get knots.
      const std::vector<double>& U = m_curves[c]->GetKnots();
      U_all.push_back(U);

#if defined COUT_DEBUG
      // Dump knots
      std::cout << "Curve " << (c + 1) << ": ";
      for ( size_t j = 0; j < U.size(); ++j )
      {
        std::cout << U[j] << "\t";
      }
      std::cout << std::endl;
#endif
    }

    // Compute complementary knots.
    bspl_UnifyKnots Unify;
    std::vector< std::vector<double> > X = Unify(U_all);

    // Unify knots.
    for ( size_t c = 0; c < m_curves.size(); ++c )
    {
      m_curves[c]->RefineKnots(X[c]);

#if defined COUT_DEBUG
      const std::vector<double>& U = m_curves[c]->Knots();

      // Dump knots
      std::cout << "Curve [refined] " << (c + 1) << ": ";
      for ( size_t j = 0; j < U.size(); ++j )
      {
        std::cout << U[j] << "\t";
      }
      std::cout << std::endl;
#endif
    }
  }

  return true;
}
