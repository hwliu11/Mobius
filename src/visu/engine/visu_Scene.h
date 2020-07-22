//-----------------------------------------------------------------------------
// Created on: 19 July 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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

#ifndef visu_Scene_HeaderFile
#define visu_Scene_HeaderFile

// visu includes
#include <mobius/visu_ActorCartesianAxes.h>
#include <mobius/visu_Camera.h>
#include <mobius/visu_UniqueName.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL scene.
class visu_Scene : public core_OBJECT
{
public:

  mobiusVisu_EXPORT
    visu_Scene();

  mobiusVisu_EXPORT
    visu_Scene(const t_ptr<visu_Camera>& camera);

  mobiusVisu_EXPORT virtual
    ~visu_Scene();

public:

  mobiusVisu_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

  mobiusVisu_EXPORT void
    GL_Draw();

  mobiusVisu_EXPORT void
    Add(const t_ptr<visu_Actor>& actor,
        const std::string& name = "");

  mobiusVisu_EXPORT TActorVec
    Actors() const;

  mobiusVisu_EXPORT std::vector<std::string>
    ActorNames() const;

  mobiusVisu_EXPORT t_ptr<visu_Actor>
    FindActor(const std::string& name) const;

  mobiusVisu_EXPORT void
    Clear();

  mobiusVisu_EXPORT void
    Clear(const std::string& actorName);

  mobiusVisu_EXPORT void
    InstallAxes();

  mobiusVisu_EXPORT void
    SetPickingRay(const t_ptr<visu_Actor>& actor);

  mobiusVisu_EXPORT const t_ptr<visu_Camera>&
    Camera() const;

  mobiusVisu_EXPORT void
    SetCamera(const t_ptr<visu_Camera>& camera);

private:

  //! Collection of Actors to draw.
  TStringActorMap m_actors;

  //! Axes Actor takes a special status in Scene.
  t_ptr<visu_ActorCartesianAxes> m_axesActor;

  //! Actor for picking ray.
  t_ptr<visu_Actor> m_pickActor;

  //! Camera.
  t_ptr<visu_Camera> m_camera;

};

}

#endif
