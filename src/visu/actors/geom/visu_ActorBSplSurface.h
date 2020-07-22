//-----------------------------------------------------------------------------
// Created on: 07 February 2014
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

#ifndef visu_ActorBSplSurface_HeaderFile
#define visu_ActorBSplSurface_HeaderFile

// visu includes
#include <mobius/visu_ActorInsensitiveSurface.h>

// geom includes
#include <mobius/geom_BSplineSurface.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL Actor dedicated to visualization of B-spline
//! surfaces.
class visu_ActorBSplSurface : public visu_ActorInsensitiveSurface
{
public:

  mobiusVisu_EXPORT
    visu_ActorBSplSurface(const t_ptr<t_bsurf>& Surf);

  mobiusVisu_EXPORT virtual
    ~visu_ActorBSplSurface();

public:

  mobiusVisu_EXPORT virtual void
    GL_Draw();

public:

  //! Enables curvature "combs" diagram in U directions.
  //! \param on [in] true/false;
  void SetHedgehog_U(const bool on)
  {
    m_bHedgehog_U = on;
  }

  //! Enables curvature "combs" diagram in V directions.
  //! \param on [in] true/false;
  void SetHedgehog_V(const bool on)
  {
    m_bHedgehog_V = on;
  }

  //! Enables/disables visualization of control polygon.
  //! \param on [in] true/false;
  void SetPoles(const bool on)
  {
    m_bNoPoly = !on;
  }

protected:

  mobiusVisu_EXPORT void
    fillIsolineProps(const t_ptr<t_bcurve>&             Iso,
                     std::vector< std::vector<t_xyz> >& Points,
                     std::vector< std::vector<t_xyz> >& Curvatures);

private:

  //! U-isos.
  std::vector< std::vector<t_xyz> > m_isoU;

  //! Curvatures of U-isos.
  std::vector< std::vector<t_xyz> > m_isoU_N;

  //! V-isos.
  std::vector< std::vector<t_xyz> > m_isoV;

  //! Curvatures of V-isos.
  std::vector< std::vector<t_xyz> > m_isoV_N;

  //! Trigger for curvature "combs" diagram in U direction.
  bool m_bHedgehog_U;

  //! Trigger for curvature "combs" diagram in V direction.
  bool m_bHedgehog_V;

  //! Indicates whether to render the control polygon.
  bool m_bNoPoly;

};

}

#endif
