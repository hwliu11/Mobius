//-----------------------------------------------------------------------------
// Created on: 15 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef visu_ActorKleinBottle_HeaderFile
#define visu_ActorKleinBottle_HeaderFile

// visu includes
#include <mobius/visu_ActorInsensitiveSurface.h>

// geom includes
#include <mobius/geom_KleinBottle.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL Actor dedicated to visualization of Klein bottle
//! surfaces.
class visu_ActorKleinBottle : public visu_ActorInsensitiveSurface
{
public:

  mobiusVisu_EXPORT
    visu_ActorKleinBottle(const t_ptr<geom_KleinBottle>& Surf);

  mobiusVisu_EXPORT virtual
    ~visu_ActorKleinBottle();

public:

  mobiusVisu_EXPORT virtual void
    GL_Draw();

};

}

#endif
