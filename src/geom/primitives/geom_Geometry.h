//-----------------------------------------------------------------------------
// Created on: 23 May 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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

#ifndef geom_Geometry_HeaderFile
#define geom_Geometry_HeaderFile

// Geometry includes
#include <mobius/geom.h>

// Core includes
#include <mobius/core_IsoTransformChain.h>
#include <mobius/core_IPlotter.h>
#include <mobius/core_IProgressNotifier.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Base class for 3D geometric entities.
//!
//! \todo complete description
class geom_Geometry : public core_OBJECT
{
public:

  virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const = 0;

public:

  //! Sets transformations to apply on the geometric primitive.
  //! \param tChain [in] transformation chain to set.
  void SetTransformChain(const core_IsoTransformChain& tChain)
  {
    m_tChain = tChain;
  }

  //! Returns transformation chain associated with the geometric primitive.
  //! \return transformation chain.
  const core_IsoTransformChain& GetTransformChain() const
  {
    return m_tChain;
  }

  //! Sets diagnostic tools for the object.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  void SetDiagnosticTools(core_ProgressEntry progress,
                          core_PlotterEntry  plotter)
  {
    m_progress = progress;
    m_plotter  = plotter;
  }

// Construction & destruction:
protected:

  mobiusGeom_EXPORT
    geom_Geometry( const core_IsoTransformChain& tChain = core_IsoTransformChain() );

  mobiusGeom_EXPORT virtual
    ~geom_Geometry();

protected:

  core_IsoTransformChain m_tChain; //!< Transformation chain.

protected:

  mutable core_ProgressEntry m_progress; //!< Progress notifier.
  mutable core_PlotterEntry  m_plotter;  //!< Imperative plotter.

};

};

#endif
