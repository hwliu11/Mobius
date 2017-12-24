//-----------------------------------------------------------------------------
// Created on: 25 February 2015
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

#ifndef geom_SectionLine_HeaderFile
#define geom_SectionLine_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_PositionCloud.h>

namespace mobius {

//! Auxiliary structure representing one cross-section being a subject of
//! reverse engineering. In a simplest case we need to have just several
//! ordered points which can be then interpolated (or approximated) for
//! subsequent skinning (or any different reconstruction method). But for
//! complex solutions where several patches are reconstructed it is desirable
//! to store additional information, like real curve, order ID etc.
class geom_SectionLine : public core_OBJECT
{
public:

  //---------------------------------------------------------------------------
  // Construction / destruction:

  geom_SectionLine() : core_OBJECT(), ID(-1) {}
  geom_SectionLine(const int id,
                   const std::vector<xyz>& pts) : core_OBJECT(), ID(id), Pts( new pcloud(pts) ) {}
  geom_SectionLine(const int id,
                   const Ptr<pcloud>& pts) : core_OBJECT(), ID(id), Pts(pts) {}

public:

  //---------------------------------------------------------------------------
  // Members:

  int         ID;    //!< ID of the section.
  Ptr<pcloud> Pts;   //!< Original points.
  Ptr<bcurve> Curve; //!< Reconstructed curve.

  //! Optional ordered slices.
  std::vector< Ptr<geom_SectionLine> > Slices;

public:

  //---------------------------------------------------------------------------
  // Services:

  mobiusGeom_EXPORT void
    Split(const xyz& pnt,
          bool&      splitting_done,
          bool&      pnt_belongs);

  mobiusGeom_EXPORT void
    Split(const size_t idx,
          bool&        splitting_done);

  mobiusGeom_EXPORT void
    Split(const std::vector<xyz>& pts,
          bool&                   splitting_done);

  mobiusGeom_EXPORT void
    Split(const std::vector<size_t>& indices,
          bool&                      splitting_done);

  mobiusGeom_EXPORT void
    SplitAndDiscrete(const double t,
                     const int    num_pts,
                     bool&        splitting_done);

  mobiusGeom_EXPORT void
    Cut(const std::vector<xyz>& pts,
        bool&                   cutting_done);

  mobiusGeom_EXPORT void
    InterpolateAndDiscrete(const int num_pts,
                           bool&     interp_done);

  mobiusGeom_EXPORT void
    Discrete(const int num_pts);

  mobiusGeom_EXPORT void
    Interpolate(const int deg,
                bool&     interp_done);

  mobiusGeom_EXPORT bool
    IsChainOf(const int         num_segments,
              std::vector<xyz>& joints) const;

public:

  mobiusGeom_EXPORT static void
    Split(const Ptr<geom_SectionLine>& source,
          const size_t                 idx,
          Ptr<geom_SectionLine>&       slice_before,
          Ptr<geom_SectionLine>&       slice_after,
          bool&                        splitting_done);

protected:

  mobiusGeom_EXPORT int
    find_index(const xyz& pnt) const;

  mobiusGeom_EXPORT static int
    find_index(const Ptr<geom_SectionLine>& source,
               const xyz&                   pnt);

};

//! Handy shortcut for section line type name.
typedef geom_SectionLine sline;

};

#endif
