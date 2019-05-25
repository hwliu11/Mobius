//-----------------------------------------------------------------------------
// Created on: 03 March 2015
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

// Own include
#include <mobius/geom_SectionLine.h>

// Core includes
#include <mobius/core_Precision.h>

// Geometry includes
#include <mobius/geom_InterpolateCurve.h>

// STL includes
#include <algorithm>

//! Attempts to split section line by the given point. If succeeds, two
//! slices are returned in the order of the natural line's orientation.
//! \param pnt            [in]  point to split the line by.
//! \param splitting_done [out] true in case of success, false -- otherwise.
//! \param pnt_belong     [out] true if the passed point belongs to the line.
void mobius::geom_SectionLine::Split(const xyz& pnt,
                                     bool&      splitting_done,
                                     bool&      pnt_belongs)
{
  const int pnt_idx = this->find_index(pnt);

  // Check if the passed point belongs to the section
  if ( pnt_idx == -1 )
  {
    splitting_done = false;
    pnt_belongs    = false;
    return;
  }

  pnt_belongs = true;
  this->Split( (size_t) pnt_idx, splitting_done );
}

//! Attempts to split section line by the given point index. If succeeds, two
//! slices are returned in the order of the natural line's orientation.
//! \param idx            [in]  index of the point to split the line by.
//! \param splitting_done [out] true in case of success, false -- otherwise.
void mobius::geom_SectionLine::Split(const size_t idx,
                                     bool&        splitting_done)
{
  ptr<geom_SectionLine> sct_ptr = this,
                              slice_before, slice_after;
  Split(sct_ptr, idx, slice_before, slice_after, splitting_done);

  this->Slices.clear();
  this->Slices.push_back(slice_before);
  this->Slices.push_back(slice_after);
}

//! Splits the section line by the given collection of points. The
//! resulting slices in their natural order (as points follow in the
//! original section) are pushed to the output collection.
//! \param pts            [in]  points to split by.
//! \param splitting_done [out] true in case of success, false -- otherwise.
void mobius::geom_SectionLine::Split(const std::vector<xyz>& pts,
                                     bool&                   splitting_done)
{
  // Find indices of all points
  std::vector<size_t> indices;
  for ( size_t p = 0; p < pts.size(); ++p )
  {
    const xyz& pnt = pts[p];
    const int idx = this->find_index(pnt);
    if ( idx != -1 )
      indices.push_back( (size_t) idx );
  }

  // Sort indices so that the points are in the natural section's order
  std::sort( indices.begin(), indices.end() );

  // Clean up existing slices
  this->Slices.clear();

  // Split section point by point
  ptr<geom_SectionLine> slice_before,
                        slice_after,
                        current = this;
  for ( int p = 0; p < int( indices.size() ); ++p )
  {
    const int idx_in_slice = find_index( current, this->Pts->GetPoint(indices[p]) );

    this->Split(current, idx_in_slice, slice_before, slice_after, splitting_done);

    if ( !splitting_done )
      return;

    this->Slices.push_back(slice_before);
    current = slice_after;
  }
  this->Slices.push_back(current);
}

//! Splits the section line by the given collection of points. The
//! resulting slices in their natural order (as points follow in the
//! original section) are pushed to the output collection.
//! \param pts            [in]  points to split by.
//! \param splitting_done [out] true in case of success, false -- otherwise.
void mobius::geom_SectionLine::Split(const std::vector<size_t>& indices,
                                     bool&                      splitting_done)
{
  std::vector<xyz> pts;
  for ( int i = 0; i < int( indices.size() ); ++i )
    pts.push_back( this->Pts->GetPoint(indices[i]) );

  this->Split(pts, splitting_done);
}

//! Splits section by parameter and generates uniform point series on each
//! slice.
//! \param t              [in] parameter.
//! \param num_pts        [in] number of points in the section to generate.
//! \param splitting_done [out] true in case of success, false -- otherwise.
void mobius::geom_SectionLine::SplitAndDiscrete(const double t,
                                                const int    num_pts,
                                                bool&        splitting_done)
{
  //---------------------------------------------
  // Build parametric polyline in order to split
  //---------------------------------------------

  geom_InterpolateCurve InterpCrv(this->Pts->GetPoints(), 1,
                                  ParamsSelection_ChordLength,
                                  KnotsSelection_Average);
  try
  {
    InterpCrv.Perform();
  }
  catch ( ... )
  {
    splitting_done = false;
    return;
  }

  const ptr<bcurve>& crv = InterpCrv.GetResult();

  ptr<geom_SectionLine> slice_before = new geom_SectionLine(this->ID, new pcloud);
  ptr<geom_SectionLine> slice_after  = new geom_SectionLine(this->ID, new pcloud);

  //-------------------------------------------------------
  // Generate new sections with uniform point distribution
  //-------------------------------------------------------

  // Populate slice before
  {
    const double uMin = 0.0;
    const double uMax = t;
    const double step = (uMax - uMin) / (num_pts - 1);
    for ( int pnt_idx = 0; pnt_idx < num_pts; ++pnt_idx )
    {
      double u = uMin + pnt_idx*step;

      if ( fabs(u - uMax) < 1.0e-10 )
        u = uMax; // Just to compensate round off errors

      xyz C;
      crv->Eval(u, C);
      slice_before->Pts->AddPoint(C);
    }
  }

  // Populate slice after
  {
    const double uMin = t;
    const double uMax = 1.0;
    const double step = (uMax - uMin) / (num_pts - 1);
    for ( int pnt_idx = 0; pnt_idx < num_pts; ++pnt_idx )
    {
      double u = uMin + pnt_idx*step;

      if ( fabs(u - uMax) < 1.0e-10 )
        u = uMax; // Just to compensate round off errors

      xyz C;
      crv->Eval(u, C);
      slice_after->Pts->AddPoint(C);
    }
  }

  splitting_done = true;
  this->Slices.clear();
  this->Slices.push_back(slice_before);
  this->Slices.push_back(slice_after);
}

//! More sophisticated way to extract slices from the section line. This
//! function will cut off all segments lying between two adjacent points.
//! If there is just one point, the line is simply split in two slices.
//! \param pts          [in]  break points.
//! \param cutting_done [out] true in case of success, false -- otherwise.
void mobius::geom_SectionLine::Cut(const std::vector<xyz>& pts,
                                   bool&                   cutting_done)
{
  std::vector<size_t> belonging_indices;
  for ( size_t p = 0; p < pts.size(); ++p )
  {
    const int p_idx = this->find_index(pts[p]);
    if ( p_idx != -1 )
      belonging_indices.push_back( (size_t) p_idx );
  }

  // No points, do nothing
  if ( !belonging_indices.size() )
  {
    cutting_done = false;
    return;
  }

  // One point leads to simple splitting
  if ( belonging_indices.size() == 1 )
  {
    ptr<geom_SectionLine> slice_before, slice_after;
    this->Split(belonging_indices[0], cutting_done);
    return;
  }

  // Split section by points
  bool splitting_done = false;
  this->Split(belonging_indices, splitting_done);
  if ( !splitting_done )
  {
    cutting_done = false;
    return;
  }

  // Clean up slices storing intermediate collection in local variable
  std::vector< ptr<geom_SectionLine> > pre_slices = this->Slices;
  this->Slices.clear();

  // Ok
  cutting_done = true;

  // Skip each even slice
  for ( size_t s = 0; s < pre_slices.size(); s += 2 )
    this->Slices.push_back(pre_slices[s]);
}

//! Interpolates section and re-creates its point data as a uniform
//! distribution of points on the interpolant.
//! \param num_pts      [in]  number of points to use.
//! \param cutting_done [out] true in case of success, false -- otherwise.
void mobius::geom_SectionLine::InterpolateAndDiscrete(const int num_pts,
                                                      bool&     interp_done)
{
  //---------------------------
  // Build parametric polyline
  //---------------------------

  this->Interpolate(1, interp_done);
  if ( !interp_done )
    return;

  //-----------------------------------------
  // Generate new uniform point distribution
  //-----------------------------------------

  this->Pts = new pcloud;

  const double uMin = this->Curve->GetMinParameter();
  const double uMax = this->Curve->GetMaxParameter();
  const double step = (uMax - uMin) / (num_pts - 1);
  for ( int pnt_idx = 0; pnt_idx < num_pts; ++pnt_idx )
  {
    double u = uMin + pnt_idx*step;

    if ( fabs(u - uMax) < 1.0e-10 )
      u = uMax; // Just to compensate round off errors

    xyz C;
    this->Curve->Eval(u, C);
    this->Pts->AddPoint(C);
  }
}

//! Creates section's point data as a uniform distribution of points on
//! the interpolant.
//! \param num_pts [in] number of points to use.
void mobius::geom_SectionLine::Discrete(const int num_pts)
{
  this->Pts = new pcloud;
  if ( this->Curve.IsNull() )
    return;

  const double uMin = this->Curve->GetMinParameter();
  const double uMax = this->Curve->GetMaxParameter();
  const double step = (uMax - uMin) / (num_pts - 1);
  for ( int pnt_idx = 0; pnt_idx < num_pts; ++pnt_idx )
  {
    double u = uMin + pnt_idx*step;

    if ( fabs(u - uMax) < 1.0e-10 )
      u = uMax; // Just to compensate round off errors

    xyz C;
    this->Curve->Eval(u, C);
    this->Pts->AddPoint(C);
  }
}

//! Interpolates section.
//! \param deg [in] degree for interpolation.
//! \param cutting_done [out] true in case of success, false -- otherwise.
void mobius::geom_SectionLine::Interpolate(const int deg,
                                           bool&     interp_done)
{
  if ( this->Pts->GetNumberOfPoints() <= 1 )
  {
    interp_done = false;
    return;
  }

  geom_InterpolateCurve InterpCrv(this->Pts->GetPoints(), deg,
                                  ParamsSelection_ChordLength,
                                  KnotsSelection_Average);
  try
  {
    InterpCrv.Perform();
  }
  catch ( ... )
  {
    interp_done = false;
    return;
  }

  interp_done = true;
  this->Curve = InterpCrv.GetResult();
}

//! Checks if the slices of this section (if any) form a continuous
//! chain of the desired number of segments.
//! \param num_segments [in]  desired number of segments.
//! \param joints       [out] connection of joint points.
//! \return true/false.
bool mobius::geom_SectionLine::IsChainOf(const int         num_segments,
                                         std::vector<xyz>& joints) const
{
  if ( (int) this->Slices.size() != num_segments )
    return false;

  for ( size_t s = 0; s < this->Slices.size() - 1; ++s )
  {
    const xyz& A = this->Slices[s]->Pts->GetPoint(this->Slices[s]->Pts->GetNumberOfPoints() - 1);
    const xyz& B = this->Slices[s + 1]->Pts->GetPoint(0);

    if ( !(A - B).IsOrigin( core_Precision::Resolution3D()) )
      return false;

    joints.push_back(A);
  }

  return true;
}

//! Attempts to split the passed section line by the given point index. If
//! succeeds, two slices are returned in the order of the natural line's
//! orientation.
//! \param source         [in]  source section.
//! \param idx            [in]  index of the point to split the line by.
//! \param slice_before   [out] section slice preceding the point.
//! \param slice_after    [out] section slice after the point.
//! \param splitting_done [out] true in case of success, false -- otherwise.
void mobius::geom_SectionLine::Split(const ptr<geom_SectionLine>& source,
                                     const size_t                 idx,
                                     ptr<geom_SectionLine>&       slice_before,
                                     ptr<geom_SectionLine>&       slice_after,
                                     bool&                        splitting_done)
{
  const size_t n_pts = source->Pts->GetNumberOfPoints();

  // Check if the passed point lies on a border
  if ( idx == 0 || idx == n_pts - 1 )
  {
    splitting_done = false;
    return;
  }

  // Construct slice before the point
  slice_before = new geom_SectionLine(source->ID, new pcloud);
  for ( int p = 0; p <= idx; ++p )
    slice_before->Pts->AddPoint( source->Pts->GetPoint(p) );

  // Construct slice after the point
  slice_after = new geom_SectionLine(source->ID, new pcloud);
  for ( int p = idx; p < n_pts; ++p )
    slice_after->Pts->AddPoint( source->Pts->GetPoint(p) );

  // Ok
  splitting_done = true;
}

//! Finds index of the point in the section line by coordinates.
//! \param pnt [in] point to locate in the line.
//! \return 0-based index or -1 if the point was not found.
int mobius::geom_SectionLine::find_index(const xyz& pnt) const
{
  const double prec = core_Precision::Resolution3D();
  for ( int p = 0; p < this->Pts->GetNumberOfPoints(); ++p )
  {
    const xyz& sct_pt = this->Pts->GetPoint(p);
    if ( (sct_pt - pnt).Modulus() < prec )
      return (int) p;
  }
  return -1;
}

//! Finds index of the point in the given section line by coordinates.
//! \param source [in] section line to analyze.
//! \param pnt    [in] point to locate in the line.
//! \return 0-based index or -1 if the point was not found.
int mobius::geom_SectionLine::find_index(const ptr<geom_SectionLine>& source,
                                         const xyz&                         pnt)
{
  const double prec = core_Precision::Resolution3D();
  for ( int p = 0; p < source->Pts->GetNumberOfPoints(); ++p )
  {
    const xyz& sct_pt = source->Pts->GetPoint(p);
    if ( (sct_pt - pnt).Modulus() < prec )
      return (int) p;
  }
  return -1;
}
