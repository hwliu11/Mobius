//-----------------------------------------------------------------------------
// Created on: 25 June 2021
//-----------------------------------------------------------------------------
// Copyright (c) 2021-present, Sergey Slyadnev
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
#include <mobius/core_UV.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! A single node in our quadtree data structure.
class poly_QuadtreeNode
{
public:

  //! Checks if the passed corner ID is valid.
  //! \param[in] id ID to check.
  //! \return true/false.
  mobiusPoly_EXPORT static bool
    IsValidCornerId(const size_t id);

  //! Returns the ID of one of the 4 cell corners. This ID is determined
  //! by the passed locations of the corresponding `x` and `y` coordinates.
  //! The passed arguments may have values 0 and 1, hence there are 4
  //! combinations (4 corners).
  //!
  //! The returned ID is derived with the following rule:
  //!
  //! \verbatim
  //!  ID = nx | (ny << 1)
  //! \endverbatim
  //!
  //! So it gives:
  //!
  //! \verbatim
  //!  nx | ny || id
  //! ====+====++===
  //!   0 |  0 || 0
  //! ----+----++---
  //!   1 |  0 || 1
  //! ----+----++---
  //!   0 |  1 || 2
  //! ----+----++---
  //!   1 |  1 || 3
  //! ----+----++---
  //! \endverbatim
  //!
  //! \param[in] nx X location (0 for min, 1 for max).
  //! \param[in] ny Y location (0 for min, 1 for max).
  //! \return ID of the corner in range [0,3].
  mobiusPoly_EXPORT static size_t
    GetCornerID(const size_t nx,
                const size_t ny);

  //! Given the corner ID in range [0,3], this static function returns the
  //! location of the corner in a cell.
  //!
  //! The location is derived with the following rule (opposite to the
  //! rule which is used to derive the ID by location):
  //!
  //! \verbatim
  //!  nx = (id >> 0) & 1
  //!  ny = (id >> 1) & 1
  //! \endverbatim
  //!
  //! \verbatim
  //!  id || nx | ny
  //! ====++====+====
  //!   0 ||  0 |  0
  //! ----++----+----
  //!   1 ||  1 |  0
  //! ----++----+----
  //!   2 ||  0 |  1
  //! ----++----+----
  //!   3 ||  1 |  1
  //! ----++----+----
  //! \endverbatim
  //!
  //! \param[in]  id ID of the corner.
  //! \param[out] nx location along OX world axis.
  //! \param[out] ny location along OY world axis.
  mobiusPoly_EXPORT static void
    GetCornerLocation(const size_t id,
                      size_t&      nx,
                      size_t&      ny);

public:

  //! Default ctor.
  mobiusPoly_EXPORT
    poly_QuadtreeNode();

  //! Ctor accepting corners.
  //! \param[in] cornerMin min corner point.
  //! \param[in] cornerMax max corner point.
  mobiusPoly_EXPORT
    poly_QuadtreeNode(const t_uv& cornerMin,
                      const t_uv& cornerMax);

  //! Dtor. It is not virtual to save 8 bytes of memory.
  mobiusPoly_EXPORT
    ~poly_QuadtreeNode();

public:

  //! Releases the memory occupied by this quadtree node with recursive treatment
  //! on children.
  mobiusPoly_EXPORT void
    Release();

  //! Returns true if this quadtree node has no children.
  //! \return true/false.
  mobiusPoly_EXPORT bool
    IsLeaf() const;

  //! Splits this node into 4 cells.
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

  //! Returns the depth of the octree starting from the 0-th node. This method
  //! consults the 0-th children only, so it is only applicable when your SVO
  //! is complete (i.e., the voxelization is uniform).
  //! \return depth through the 0-th nodes.
  mobiusPoly_EXPORT unsigned
    GetDepth0() const;

  //! For the passed `xyz` point, this method evaluates the scalar field which
  //! is defined in the current SVO node. For that, the method goes down by the
  //! SVO hierarchy until it reaches the leaf voxel containing the relevant
  //! field values.
  //! \param[in] P       point of interest.
  //! \param[in] bndOnly indicates whether the scalar field should only be
  //!                    evaluated for the zero-crossing nodes. If so, then
  //!                    for the negative nodes, the value of -1 is returned;
  //!                    for the positive nodes, the value of 1 is returned.
  //! \return interpolated field value.
  mobiusPoly_EXPORT double
    Eval(const t_xyz& P,
         const bool   bndOnly = false) const;

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

  //! Recursively visits SVO nodes to add leaves to the passed output
  //! collection.
  //! \param[in]  pNode  SVO node to visit.
  //! \param[in]  sm     scalar membership classifier.
  //! \param[out] leaves collected leaves.
  mobiusPoly_EXPORT void
    getLeaves(const poly_SVO*               pNode,
              const int                     sm,
              std::vector<const poly_SVO*>& leaves) const;

protected:

  poly_SVO** m_pChildren;  //!< Child octree nodes (8 bytes).
  double     m_scalars[8]; //!< Stored scalar values (64 bytes).

  /*
    NOTICE: we store the corner points of each cell for convenience. It is
            not really necessary to have them here as the corners can be deduced
            from the root node if we know its dimensions. At the same time,
            we have found that the convenience of having the explicit corner
            points as a part of cell definition outweights the memory overheads.
  */

  /* 48 bytes for corners */
  t_xyz m_cornerMin;  //!< Min corner of the SVO box.
  t_xyz m_cornerMax;  //!< Max corner of the SVO box.

};

}

#endif
