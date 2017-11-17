//-----------------------------------------------------------------------------
// Created on: 25 February 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
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
