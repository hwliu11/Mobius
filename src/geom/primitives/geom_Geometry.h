//-----------------------------------------------------------------------------
// Created on: 23 May 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_Geometry_HeaderFile
#define geom_Geometry_HeaderFile

// Geometry includes
#include <mobius/geom.h>

// Core includes
#include <mobius/core_IsoTransformChain.h>
#include <mobius/core_Ptr.h>

namespace mobius {

//! Base class for 3D geometric entities.
//!
//! \todo complete description
class geom_Geometry : public core_OBJECT
{
public:

  virtual void
    Bounds(double& xMin, double& xMax,
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
  const core_IsoTransformChain& TransformChain() const
  {
    return m_tChain;
  }

// Construction & destruction:
protected:

  mobiusGeom_EXPORT
    geom_Geometry( const core_IsoTransformChain& tChain = core_IsoTransformChain() );

  mobiusGeom_EXPORT virtual
    ~geom_Geometry();

protected:

  core_IsoTransformChain m_tChain; //!< Transformation chain.

};

};

#endif
