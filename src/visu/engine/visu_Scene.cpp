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

// Windows
#include <windows.h>

// GL includes
#include <gl/gl.h>
#include <gl/glu.h>

// Include glext.h provided by Khronos group
#include <mobius/glext.h>

// Own include
#include <mobius/visu_Scene.h>

#define INF 1.0e4
#define QrSCENE_AxesFactor 0.5
#define QrSCENE_BigRange 10000
#define QrSCENE_NoName_Actor "actor"

//! Constructor.
mobius::visu_Scene::visu_Scene() : core_OBJECT()
{
}

//! Constructs Scene with Camera.
//! \param camera [in] Camera to set.
mobius::visu_Scene::visu_Scene(const t_ptr<visu_Camera>& camera) : core_OBJECT()
{
  this->SetCamera(camera);
}

//! Destructor.
mobius::visu_Scene::~visu_Scene()
{
}

//! This methods creates a special Actor representing cartesian axes.
//! Call it after your scene is assembled as the size of the axes depend
//! on the scene's size.
void mobius::visu_Scene::InstallAxes()
{
  double x_min, x_max, y_min, y_max, z_min, z_max;
  this->GetBounds(x_min, x_max, y_min, y_max, z_min, z_max);
  const double maxSize = max( abs(x_max - x_min),
                              max( abs(y_max - y_min),
                                   abs(z_max - z_min) ) );

  // Prepare Actor for cartesian axes
  // TODO: do not reallocate, better adjust!
  m_axesActor = new visu_ActorCartesianAxes(maxSize*QrSCENE_AxesFactor);
}

//! Sets Actor dedicated to visualization of picking ray.
//! \param actor [in] actor to set.
void mobius::visu_Scene::SetPickingRay(const t_ptr<visu_Actor>& actor)
{
  m_pickActor = actor;
}

//! Returns Camera associated with Scene.
//! \return Camera.
const mobius::t_ptr<mobius::visu_Camera>&
  mobius::visu_Scene::Camera() const
{
  return m_camera;
}

//! Associates Camera with Scene.
//! \param camera [in] Camera to associate.
void mobius::visu_Scene::SetCamera(const t_ptr<visu_Camera>& camera)
{
  m_camera = camera;
}

//! Calculates boundary box for this scene.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_Scene::GetBounds(double& xMin, double& xMax,
                                   double& yMin, double& yMax,
                                   double& zMin, double& zMax) const
{
  double x_min = INF, x_max = -INF;
  double y_min = INF, y_max = -INF;
  double z_min = INF, z_max = -INF;

  for ( TStringActorMap::const_iterator it = m_actors.begin(); it != m_actors.end(); ++it )
  {
    double ax_min, ax_max, ay_min, ay_max, az_min, az_max;
    const t_ptr<visu_Actor>& actor = it->second;
    actor->GetBounds(ax_min, ax_max, ay_min, ay_max, az_min, az_max);

    if ( ax_min < x_min )
      x_min = ax_min;
    if ( ax_max > x_max )
      x_max = ax_max;
    if ( ay_min < y_min )
      y_min = ay_min;
    if ( ay_max > y_max )
      y_max = ay_max;
    if ( az_min < z_min )
      z_min = az_min;
    if ( az_max > z_max )
      z_max = az_max;
  }

  // Set results
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}

//! Draws all actors.
void mobius::visu_Scene::GL_Draw()
{
  glClearColor(0.1f, 0.1f, 0.1f, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Enable depth testing
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LEQUAL);
  glDepthRange(0.0f, 2.0f);

  // Enable multisampling for antialiasing
  glEnable(GL_MULTISAMPLE_ARB);

  // Enable smoothing
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Enable alpha testing for preventing "dark noise" on rotation.
  // Well, I have to study OpenGL better...
  glAlphaFunc(GL_GREATER, 0.5f);
  glEnable(GL_ALPHA_TEST);

  //---------------------------------------------------------------------------
  // Enable lighting
  //---------------------------------------------------------------------------

  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  // Light components
  GLfloat ambientLight[] = { 0.7f, 0.7f, 0.7f, 1.0f };
  GLfloat diffuseLight[] = { 0.3f, 0.3f, 0.3f, 1.0f };

  // Set up and enable light 0
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
  glEnable(GL_LIGHT0);

  // Set up source position
  GLfloat lightPos[] = { 0.f, 0.0f, 0.0f, 1000.0f };
  glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

  // Set up material
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

  //---------------------------------------------------------------------------
  // Draw actors
  //---------------------------------------------------------------------------

  // Apply Camera by pushing its matrices
  m_camera->Push();

  // Draw Actors
  for ( TStringActorMap::const_iterator it = m_actors.begin(); it != m_actors.end(); ++it )
  {
    const t_ptr<visu_Actor>& actor = it->second;

    // Draw
    actor->GL_Draw();

    // Highlight
    actor->GL_Hili();
  }

  // Draw axes
  if ( !m_axesActor.IsNull() )
    m_axesActor->GL_Draw();

  // Draw picking ray Actor
  if ( !m_pickActor.IsNull() )
    m_pickActor->GL_Draw();

  // Pop Camera's matrices
  m_camera->Pop();

  //---------------------------------------------------------------------------
  // Draw addendum stuff
  //---------------------------------------------------------------------------

  #pragma region GL lines
  glPushAttrib(GL_ENABLE_BIT);
  glLineStipple(1, 0x1);
  glEnable(GL_LINE_STIPPLE);
  glLineWidth(4);
  glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(QrSCENE_BigRange, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, QrSCENE_BigRange, 0.0);
  glEnd();
  glPopAttrib();
  #pragma endregion

  // Disable multisampling
  glDisable(GL_MULTISAMPLE_ARB);

  // Disable blending
  glDisable(GL_BLEND);

  glFlush();
}

//! Adds the passed Actor to the list of Actors to draw.
//! \param actor [in] Actor to add.
//! \param name [in] name of the Actor.
void mobius::visu_Scene::Add(const t_ptr<visu_Actor>& actor,
                             const std::string& name)
{
  std::string actor_name;
  if ( name.empty() )
    actor_name = visu_UniqueName::Generate( this->ActorNames(), QrSCENE_NoName_Actor );
  else
    actor_name = name;

  // If such actor exists, rebind it
  if ( !this->FindActor(actor_name).IsNull() )
    this->Clear(actor_name);

  m_actors.insert( TStringActorPair(actor_name, actor) );
}

//! Returns all model Actors.
//! \return collection of Actors.
mobius::TActorVec mobius::visu_Scene::Actors() const
{
  TActorVec result;
  for ( TStringActorMap::const_iterator mit = m_actors.begin(); mit != m_actors.end(); ++mit )
    result.push_back(mit->second);

  return result;
}

//! Returns all Actor names.
//! \return collection of Actor names.
std::vector<std::string>
  mobius::visu_Scene::ActorNames() const
{
  std::vector<std::string> names;
  for ( TStringActorMap::const_iterator mit = m_actors.begin(); mit != m_actors.end(); ++mit )
    names.push_back(mit->first);

  return names;
}

//! Finds Actor by name.
//! \param name [in] Actor name.
//! \return found Actor or null.
mobius::t_ptr<mobius::visu_Actor>
  mobius::visu_Scene::FindActor(const std::string& name) const
{
  TStringActorMap::const_iterator it = m_actors.find(name);
  if ( it == m_actors.cend() )
    return NULL;

  return it->second;
}

//! Cleans up the Scene.
void mobius::visu_Scene::Clear()
{
  m_actors.clear();
  m_axesActor.Nullify();
}

//! Erases actor with the given name from the scene.
//! \param actorName [in] actor name.
void mobius::visu_Scene::Clear(const std::string& actorName)
{
  if ( !actorName.length() )
    this->Clear();
  else
  {
    TStringActorMap::const_iterator cit = m_actors.find(actorName);
    if ( cit != m_actors.end() )
      m_actors.erase(cit);
  }
}
