//-----------------------------------------------------------------------------
// Created on: 10 December 2019
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

#ifndef poly_SVO_HeaderFile
#define poly_SVO_HeaderFile

// Poly includes
#include <mobius/poly.h>

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! A single node in the Sparse Voxel Octree data structure. Each corner of
//! the SVO node is associated with a scalar value.
class poly_SVO
{
public:

  //! Checks if the passed corner ID is valid.
  //! \param[in] id ID to check.
  //! \return true/false.
  mobiusPoly_EXPORT static bool
    IsValidCornerId(const size_t id);

  //! Returns the ID of one of the 8-th cell corners. This ID is determined
  //! by the passed locations of the corresponding `x`, `y` and `z` coordinates.
  //! The passed arguments may have values 0 and 1, hence there are 8
  //! combinations (8 corners).
  //!
  //! The returned ID is derived with the following rule:
  //!
  //! \verbatim
  //!  ID = nx | (ny << 1) | (nz << 2)
  //! \endverbatim
  //!
  //! So it gives:
  //!
  //! \verbatim
  //!  nx | ny | nz || id
  //! ====+====+====++===
  //!   0 |  0 |  0 || 0
  //! ----+----+----++---
  //!   1 |  0 |  0 || 1
  //! ----+----+----++---
  //!   0 |  1 |  0 || 2
  //! ----+----+----++----
  //!   1 |  1 |  0 || 3
  //! ----+----+----++---
  //!   0 |  0 |  1 || 4
  //! ----+----+----++---
  //!   1 |  0 |  1 || 5
  //! ----+----+----++---
  //!   0 |  1 |  1 || 6
  //! ----+----+----++---
  //!   1 |  1 |  1 || 7
  //! ----+----+----++---
  //! \endverbatim
  //!
  //! \param[in] nx X location (0 for min, 1 for max).
  //! \param[in] ny Y location (0 for min, 1 for max).
  //! \param[in] nz Z location (0 for min, 1 for max).
  //! \return ID of the corner in range [0,7].
  mobiusPoly_EXPORT static size_t
    GetCornerID(const size_t nx,
                const size_t ny,
                const size_t nz);

  //! Given the corner ID in range [0,7], this static function returns the
  //! location of the corner in a cell (vertex for octant, octant for voxel).
  //!
  //! The location is derived with the following rule (opposite to the
  //! rule which is used to derive the ID by location):
  //!
  //! \verbatim
  //!  nx = (id >> 0) & 1
  //!  ny = (id >> 1) & 1
  //!  nz = (id >> 2) & 1
  //! \endverbatim
  //!
  //! \verbatim
  //!  id || nx | ny | nz
  //! ====++====+====+====
  //!   0 ||  0 |  0 |  0
  //! ----++----+----+----
  //!   1 ||  1 |  0 |  0
  //! ----++----+----+----
  //!   2 ||  0 |  1 |  0
  //! ----++----+----+----
  //!   3 ||  1 |  1 |  0
  //! ----++----+----+----
  //!   4 ||  0 |  0 |  1
  //! ----++----+----+----
  //!   5 ||  1 |  0 |  1
  //! ----++----+----+----
  //!   6 ||  0 |  1 |  1
  //! ----++----+----+----
  //!   7 ||  1 |  1 |  1
  //! ----++----+----+----
  //! \endverbatim
  //!
  //! \param[in]  id ID of the corner.
  //! \param[out] nx location along OX world axis.
  //! \param[out] ny location along OY world axis.
  //! \param[out] nz location along OZ world axis.
  mobiusPoly_EXPORT static void
    GetCornerLocation(const size_t id,
                      size_t&      nx,
                      size_t&      ny,
                      size_t&      nz);

public:

  //! Default ctor.
  mobiusPoly_EXPORT
    poly_SVO();

  //! Ctor accepting corners.
  //! \param[in] cornerMin min corner point.
  //! \param[in] cornerMax max corner point.
  mobiusPoly_EXPORT
    poly_SVO(const t_xyz& cornerMin,
             const t_xyz& cornerMax);

  //! Dtor.
  mobiusPoly_EXPORT virtual
    ~poly_SVO();

public:

  //! Releases the memory occupied by this SVO node with recursive treatment
  //! on children.
  mobiusPoly_EXPORT void
    Release();

  //! Checks if the node is initialized with the scalar values. At construction
  //! time, the SVO node gets infinity values as scalars. If these values have
  //! not been changed later on, this method will return false.
  //! \return true/false.
  mobiusPoly_EXPORT bool
    HasScalars();

  //! Sets scalar value for the corner with the passed ID.
  //! \param[in] id ID of the corner in range [0,7].
  //! \param[in] s  scalar value to set.
  mobiusPoly_EXPORT void
    SetScalar(const size_t id,
              const double s);

  //! Returns the scalar value associated with the passed corner ID.
  //! \param[in] id ID of the corner in range [0,7].
  //! \return scalar value.
  mobiusPoly_EXPORT double
    GetScalar(const size_t id) const;

  //! Returns true if this SVO node has no children.
  //! \return true/false.
  mobiusPoly_EXPORT bool
    IsLeaf() const;

  //! Splits this node down to 8 octants.
  //! \return false if this nodes already has children.
  mobiusPoly_EXPORT bool
    Split();

  //! Sets the child SVO node for the octant having the given index in
  //! range [0,7]. This method should be used after Split() invocation
  //! to initialize the allocated pointers with the SVO structures constructed
  //! separately.
  //! \param[in] id     ID of the child to set.
  //! \param[in] pChild raw pointer to the SVO node to set as a child.
  mobiusPoly_EXPORT void
    SetChild(const size_t id, poly_SVO* pChild);

  //! Returns the child SVO node by its index in range [0,7]. If there are
  //! no children in the current node, null pointer is returned.
  //! \param[in] id ID of the child.
  //! \return pointer to the child node.
  mobiusPoly_EXPORT poly_SVO*
    GetChild(const size_t id) const;

  //! Attempts to find a child SVO node by the specified path. If a node
  //! corresponding to the given path is not accessible, null pointer is
  //! returned.
  //! \param[in] path ordered set of indices to access the node.
  //! \return pointer to the requested node.
  mobiusPoly_EXPORT poly_SVO*
    FindChild(const std::vector<size_t>& path) const;

  //! For the passed `xyz` point, this method evaluates the scalar field which
  //! is defined in the current SVO node. For that, the method goes down by the
  //! SVO hierarchy until it reaches the leaf voxel containing the relevant
  //! field values.
  //! \param[in] P point of interest.
  //! \return interpolated field value.
  mobiusPoly_EXPORT double
    Eval(const t_xyz& P) const;

  //! Calculates the memory occupied by the SVO hierarchy starting from this
  //! node in bytes.
  //! \param[out] numNodes num of corner points.
  //! \return memory in bytes.
  mobiusPoly_EXPORT unsigned long long
    GetMemoryInBytes(int& numNodes) const;

public:

  //! \return min corner which is equal to P0 point.
  const t_xyz& GetCornerMin() const
  {
    return m_cornerMin;
  }

  //! Sets new coordinates for the min corner point.
  void SetCornerMin(const t_xyz& P)
  {
    m_cornerMin = P;
  }

  //! \return max corner which is equal to P7 point.
  const t_xyz& GetCornerMax() const
  {
    return m_cornerMax;
  }

  //! Sets new coordinates for the max corner point.
  void SetCornerMax(const t_xyz& P)
  {
    m_cornerMax = P;
  }

  //! \return corner point P0 (the notation is the same as for VTK_VOXEL).
  const t_xyz& GetP0() const
  {
    return m_cornerMin;
  }

  //! \return corner point P1 (the notation is the same as for VTK_VOXEL).
  t_xyz GetP1() const
  {
    return t_xyz( m_cornerMax.X(), m_cornerMin.Y(), m_cornerMin.Z() );
  }

  //! \return corner point P2 (the notation is the same as for VTK_VOXEL).
  t_xyz GetP2() const
  {
    return t_xyz( m_cornerMin.X(), m_cornerMax.Y(), m_cornerMin.Z() );
  }

  //! \return corner point P3 (the notation is the same as for VTK_VOXEL).
  t_xyz GetP3() const
  {
    return t_xyz( m_cornerMax.X(), m_cornerMax.Y(), m_cornerMin.Z() );
  }

  //! \return corner point P4 (the notation is the same as for VTK_VOXEL).
  t_xyz GetP4() const
  {
    return t_xyz( m_cornerMin.X(), m_cornerMin.Y(), m_cornerMax.Z() );
  }

  //! \return corner point P5 (the notation is the same as for VTK_VOXEL).
  t_xyz GetP5() const
  {
    return t_xyz( m_cornerMax.X(), m_cornerMin.Y(), m_cornerMax.Z() );
  }

  //! \return corner point P6 (the notation is the same as for VTK_VOXEL).
  t_xyz GetP6() const
  {
    return t_xyz( m_cornerMin.X(), m_cornerMax.Y(), m_cornerMax.Z() );
  }

  //! \return corner point P7 (the notation is the same as for VTK_VOXEL).
  const t_xyz& GetP7() const
  {
    return m_cornerMax;
  }

  //! \return cell size as a distance between P0 and P7.
  double GetCellSize() const
  {
    return (m_cornerMax - m_cornerMin).Modulus();
  }

protected:

  poly_SVO** m_pChildren;  //!< Child octree nodes.
  double     m_scalars[8]; //!< Stored scalar values.

  /*
    NOTICE: we store the corner points of each cell for convenience. It is
            not really necessary to have them here as the corners can be deduced
            from the root node if we know its dimensions. At the same time,
            we have found that the convenience of having the explicit corner
            points as a part of cell definition outweights the memory overheads.
  */

  t_xyz m_cornerMin;  //!< Min corner of the SVO box.
  t_xyz m_cornerMax;  //!< Max corner of the SVO box.

};

}

#endif