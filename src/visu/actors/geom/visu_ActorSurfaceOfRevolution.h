//-----------------------------------------------------------------------------
// Created on: 11 January 2023
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef visu_ActorSurfaceOfRevolution_HeaderFile
#define visu_ActorSurfaceOfRevolution_HeaderFile

// visu includes
#include <mobius/visu_ActorInsensitiveSurface.h>

// geom includes
#include <mobius/geom_SurfaceOfRevolution.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL Actor dedicated to visualization of surfaces
//! of revolution.
class visu_ActorSurfaceOfRevolution : public visu_ActorInsensitiveSurface
{
public:

  mobiusVisu_EXPORT
    visu_ActorSurfaceOfRevolution(const t_ptr<geom_SurfaceOfRevolution>& Surf);

  mobiusVisu_EXPORT virtual
    ~visu_ActorSurfaceOfRevolution();

public:

  mobiusVisu_EXPORT virtual void
    GL_Draw();

};

}

#endif
