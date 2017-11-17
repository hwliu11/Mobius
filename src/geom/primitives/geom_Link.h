//-----------------------------------------------------------------------------
// Created on: 23 May 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_Link_HeaderFile
#define geom_Link_HeaderFile

// Geometry includes
#include <mobius/geom_Geometry.h>
#include <mobius/geom_Point.h>

namespace mobius {

//! Represents simple oriented link between two 3D points.
//!
//! \todo complete description
class geom_Link : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Link(const xyz& P1,
              const xyz& P2);

  mobiusGeom_EXPORT virtual
    ~geom_Link();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  //! Returns the first connection point.
  //! \return first point.
  const xyz& P1() const
  {
    return m_p1;
  }

  //! Returns the second connection point.
  //! \return second point.
  const xyz& P2() const
  {
    return m_p2;
  }

private:

  xyz m_p1; //!< First point.
  xyz m_p2; //!< Second point.

};

};

#endif
