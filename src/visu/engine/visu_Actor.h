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

#ifndef visu_Actor_HeaderFile
#define visu_Actor_HeaderFile

// visu includes
#include <mobius/visu_DataSet.h>

// core includes
#include <mobius/core_Ptr.h>

// geom includes
#include <mobius/geom_Line.h>

// STL includes
#include <map>
#include <vector>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL Actor rendered on Scene.
class visu_Actor : public core_OBJECT
{
public:

  mobiusVisu_EXPORT
    visu_Actor();

  mobiusVisu_EXPORT virtual
    ~visu_Actor();

public:

  virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const = 0;

  virtual void
    GL_Draw() = 0;

public:

  virtual t_ptr<visu_DataSet>
    IntersectWithLine(const t_ptr<geom_Line>&) = 0;

  virtual void
    SetHiliData(const t_ptr<visu_DataSet>&) = 0;

  virtual t_ptr<visu_DataSet>
    GetHiliData() const = 0;

  virtual void
    ClearHiliData() = 0;

  virtual void
    GL_Hili() const = 0;

};

//! Collection of Actors.
typedef std::vector< t_ptr<visu_Actor> > TActorVec;

//! Name-Actor pair.
typedef std::pair< std::string, t_ptr<visu_Actor> > TStringActorPair;

//! Hash table for Actors by names.
typedef std::map< std::string, t_ptr<visu_Actor> > TStringActorMap;

}

#endif
