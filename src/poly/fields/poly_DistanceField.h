//-----------------------------------------------------------------------------
// Created on: 12 December 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

#ifndef poly_DistanceField_HeaderFile
#define poly_DistanceField_HeaderFile

// Poly includes
#include <mobius/poly_DistanceFunc.h>
#include <mobius/poly_SVO.h>

// Core includes
#include <mobius/core_IProgressNotifier.h>
#include <mobius/core_IPlotter.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Distance field represented by voxelization and its associated real
//! function to calculate the distance values.
class poly_DistanceField : public poly_RealFunc
{
public:

  //! Checks if the passed SVO node is completely inside of the initial shape.
  //! \param[in] pNode SVO node to check.
  //! \return true/false.
  mobiusPoly_EXPORT static bool
    IsIn(poly_SVO* pNode);

  //! Checks if the passed SVO node is completely outside of the initial shape.
  //! \param[in] pNode SVO node to check.
  //! \return true/false.
  mobiusPoly_EXPORT static bool
    IsOut(poly_SVO* pNode);

  //! Checks if the passed SVO node crosses zero isosurface level, i.e., it
  //! captures the boundary of the initial shape.
  //! \param[in] pNode SVO node to check.
  //! \return true/false.
  mobiusPoly_EXPORT static bool
    IsZeroCrossing(poly_SVO* pNode);

public:

  //! Ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusPoly_EXPORT
    poly_DistanceField(core_ProgressEntry progress = nullptr,
                       core_PlotterEntry  plotter  = nullptr);

  //! Ctor with initialization.
  //! \param[in] octree   octree to handle.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusPoly_EXPORT
    poly_DistanceField(poly_SVO*          pRoot,
                       core_ProgressEntry progress = nullptr,
                       core_PlotterEntry  plotter  = nullptr);

  //! Dtor.
  //! CAUTION: this dtor does not destroy the octree.
  mobiusPoly_EXPORT virtual
    ~poly_DistanceField();

public:

  //! Builds distance field with the specified spatial resolution for the
  //! passed distance function.
  //! \param[in] minCellSize min allowed voxel size.
  //! \param[in] maxCellSize max allowed voxel size.
  //! \param[in] precision   precision of implicit function approximation.
  //! \param[in] func        driving function.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    Build(const double                    minCellSize,
          const double                    maxCellSize,
          const double                    precision,
          const t_ptr<poly_DistanceFunc>& func);

public:

  //! Evaluates the distance field as a conventional trivariate function.
  //! \param[in] x X coordinate of the argument point.
  //! \param[in] y Y coordinate of the argument point.
  //! \param[in] z Z coordinate of the argument point.
  //! \return function value.
  mobiusPoly_EXPORT virtual double
    Eval(const double x, const double y, const double z) const;

public:

  //! \return root SVO node.
  poly_SVO* GetRoot()
  {
    return m_pRoot;
  }

  //! Allows to set a new root node for the distance field. Use with
  //! care as this setter does absolutely nothing in terms of memory
  //! management.
  //! \param[in] pRoot new root SVO node to set.
  void SetRoot(poly_SVO* pRoot)
  {
    m_pRoot = pRoot;
  }

  //! Creates shallow copy of this distance field.
  //! \return copy of the field pointing to the same octree.
  t_ptr<poly_DistanceField> ShallowCopy() const
  {
    t_ptr<poly_DistanceField> res = new poly_DistanceField;
    res->SetRoot(m_pRoot);
    return res;
  }

protected:

  poly_SVO*          m_pRoot;    //!< Root voxel.
  core_ProgressEntry m_progress; //!< Progress notifier.
  core_PlotterEntry  m_plotter;  //!< Imperative plotter.

};

}

#endif
