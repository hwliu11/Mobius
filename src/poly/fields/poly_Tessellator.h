//-----------------------------------------------------------------------------
// Created on: 27 January 2020
//-----------------------------------------------------------------------------
// Copyright (c) 2020-present, Sergey Slyadnev
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

#ifndef poly_Tessellator_HeaderFile
#define poly_Tessellator_HeaderFile

// Poly includes
#include <mobius/poly_Mesh.h>

// Core includes
#include <mobius/core_OPERATOR.h>
#include <mobius/core_Precision.h>

// Standard includes
#include <map>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Base class for all DDF tessellators.
class poly_Tessellator : public core_OPERATOR
{
public:

  //! Ctor with initialization.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusPoly_EXPORT
    poly_Tessellator(core_ProgressEntry progress = nullptr,
                     core_PlotterEntry  plotter  = nullptr);

  //! Dtor.
  mobiusPoly_EXPORT virtual
    ~poly_Tessellator();

public:

  //! Performs polygonal approximation.
  //! \param[in] isoValue isolevel of the implicit function to reconstruct.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    Perform(const double isoValue);

public:

  //! \return constructed mesh.
  const t_ptr<t_mesh>& GetResult() const
  {
    return m_result;
  }

private:

  //! Implementation of perform() is kept for the subclasses.
  //! \param[in] isoValue isolevel of the implicit function to reconstruct.
  //! \return true in case of success, false -- otherwise.
  virtual bool perform(const double isoValue) = 0;

protected:

  //! Compare functor for coordinate tuples.
  struct t_vertexComparator
  {
    bool operator()(const t_xyz& point1,
                    const t_xyz& point2) const
    {
      if ( point1.X() < point2.X() - core_Precision::Resolution3D() )
      {
        return true;
      }
      if ( point1.X() > point2.X() + core_Precision::Resolution3D() )
      {
        return false;
      }
      if ( point1.Y() < point2.Y() - core_Precision::Resolution3D() )
      {
        return true;
      }
      if ( point1.Y() > point2.Y() + core_Precision::Resolution3D() )
      {
        return false;
      }
      if ( point1.Z() < point2.Z() - core_Precision::Resolution3D() )
      {
        return true;
      }

      return false;
    }
  };

  //! Map to store the set of indices for 3D mesh point.
  typedef std::map<t_xyz, int, t_vertexComparator> t_vertexMap;

protected:

  t_ptr<t_mesh> m_result; //!< Reconstructed mesh.

};

}

#endif
