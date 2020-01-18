//-----------------------------------------------------------------------------
// Created on: July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

#ifndef core_IPlotter_HeaderFile
#define core_IPlotter_HeaderFile

// Core includes
#include <mobius/core_Ptr.h>

namespace mobius {

class geom_Curve;
class geom_Surface;
class core_UV;
class core_XYZ;

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Simplistic structure to hold color components.
struct core_Color
{
  double fRed;   //!< Red component in range [0, 1].
  double fGreen; //!< Green component in range [0, 1].
  double fBlue;  //!< Blue component in range [0, 1].

  core_Color() : fRed(0.), fGreen(0.), fBlue(0.) {} //!< Default ctor.

  //! Complete ctor.
  //! \param[in] _r red component to set.
  //! \param[in] _g green component to set.
  //! \param[in] _b blue component to set.
  core_Color(const double _r, const double _g, const double _b)
  : fRed(_r), fGreen(_g), fBlue(_b) {}
};

//-----------------------------------------------------------------------------

#define MobiusColor_Red       core_Color(1.0, 000, 000)
#define MobiusColor_Green     core_Color(000, 1.0, 000)
#define MobiusColor_Blue      core_Color(000, 000, 1.0)
#define MobiusColor_Yellow    core_Color(1.0, 1.0, 000)
#define MobiusColor_White     core_Color(1.0, 1.0, 1.0)
#define MobiusColor_Black     core_Color(000, 000, 000)
#define MobiusColor_Magenta   core_Color(1.0, 000, 1.0)

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Interface for Imperative Plotter concept. A particular algorithm may benefit
//! from immediate plotting of its geometric variables in a unified way
//! thanks to this abstract class.
class core_IPlotter : public core_OBJECT
{
// COMMON:
public:

  virtual void
    ERASE_ALL() {}

  virtual void
    ERASE(const std::string&) {}

// GEOMETRY:
public:

  //-------------------------------------------------------------------------//

  virtual void
    DRAW_POINT(const core_UV&,
               const core_Color&,
               const std::string&) {}

  virtual void
    DRAW_POINT(const core_XYZ&,
               const core_Color&,
               const std::string&) {}

  virtual void
    REDRAW_POINT(const std::string&,
                 const core_UV&,
                 const core_Color&) {}

  virtual void
    REDRAW_POINT(const std::string&,
                 const core_XYZ&,
                 const core_Color&) {}

  //-------------------------------------------------------------------------//

  virtual void
    DRAW_POINTS(const std::vector<core_XYZ>&,
                const core_Color&,
                const std::string&) {}

  virtual void
    REDRAW_POINTS(const std::string&,
                  const std::vector<core_XYZ>&,
                  const core_Color&) {}

  //-------------------------------------------------------------------------//

  virtual void
    DRAW_VECTOR_AT(const core_XYZ&,
                   const core_XYZ&,
                   const core_Color&,
                   const std::string&) {}

  virtual void
    REDRAW_VECTOR_AT(const std::string&,
                     const core_XYZ&,
                     const core_XYZ&,
                     const core_Color&) {}

  //-------------------------------------------------------------------------//

  virtual void
    DRAW_CURVE(const t_ptr<geom_Curve>&,
               const core_Color&,
               const std::string&) {}

  virtual void
    REDRAW_CURVE(const std::string&,
                 const t_ptr<geom_Curve>&,
                 const core_Color&) {}

  //-------------------------------------------------------------------------//

  virtual void
    DRAW_SURFACE(const t_ptr<geom_Surface>&,
                 const core_Color&,
                 const std::string&) {}

  virtual void
    DRAW_SURFACE(const t_ptr<geom_Surface>&,
                 const core_Color&,
                 const double, // opacity
                 const std::string&) {}

  virtual void
    DRAW_SURFACE(const t_ptr<geom_Surface>&,
                 const double, // U min
                 const double, // U max
                 const double, // V min
                 const double, // V max
                 const core_Color&,
                 const std::string&) {}

  virtual void
    DRAW_SURFACE(const t_ptr<geom_Surface>&,
                 const double, // U min
                 const double, // U max
                 const double, // V min
                 const double, // V max
                 const core_Color&,
                 const double, // opacity
                 const std::string&) {}

  virtual void
    REDRAW_SURFACE(const std::string&,
                   const t_ptr<geom_Surface>&,
                   const core_Color&) {}

  virtual void
    REDRAW_SURFACE(const std::string&,
                   const t_ptr<geom_Surface>&,
                   const core_Color&,
                   const double) {} // opacity

  virtual void
    REDRAW_SURFACE(const std::string&,
                   const t_ptr<geom_Surface>&,
                   const double, // U min
                   const double, // U max
                   const double, // V min
                   const double, // V max
                   const core_Color&) {}

  virtual void
    REDRAW_SURFACE(const std::string&,
                   const t_ptr<geom_Surface>&,
                   const double, // U min
                   const double, // U max
                   const double, // V min
                   const double, // V max
                   const core_Color&,
                   const double) {} // opacity

// TESSELLATION:
public:

  virtual void
    DRAW_LINK(const core_XYZ&,
              const core_XYZ&,
              const core_Color&,
              const std::string&) {}

  virtual void
    DRAW_LINK(const core_UV&,
              const core_UV&,
              const core_Color&,
              const std::string&) {}

  virtual void
    REDRAW_LINK(const std::string&,
                const core_XYZ&,
                const core_XYZ&,
                const core_Color&) {}

  virtual void
    REDRAW_LINK(const std::string&,
                const core_UV&,
                const core_UV&,
                const core_Color&) {}

};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Safe entry to imperative plotter.
class core_PlotterEntry
{
public:

  //! Default constructor.
  core_PlotterEntry() {}

  //! Dummy conversion constructor.
  core_PlotterEntry(std::nullptr_t) {}

  //! Constructor.
  //! \param iv [in] IV to wrap.
  core_PlotterEntry(const t_ptr<core_IPlotter>& iv) : m_iv(iv) {}

public:

  //! \return Journal instance.
  const t_ptr<core_IPlotter>& GetPlotter() const { return m_iv; }

public:

//---------------------------------------------------------------------------//

  void
    ERASE_ALL()
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->ERASE_ALL();
  }

//---------------------------------------------------------------------------//

  void
    ERASE(const std::string& name)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->ERASE(name);
  }

public:

//---------------------------------------------------------------------------//

  void
    DRAW_POINT(const core_UV&     coord,
               const core_Color&  color,
               const std::string& name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_POINT(coord, color, name);
  }

//---------------------------------------------------------------------------//

  void
    DRAW_POINT(const core_XYZ&    coord,
               const core_Color&  color,
               const std::string& name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_POINT(coord, color, name);
  }

//---------------------------------------------------------------------------//

  void
    REDRAW_POINT(const std::string& name,
                 const core_UV&     point,
                 const core_Color&  color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_POINT(name, point, color);
  }

//---------------------------------------------------------------------------//

  void
    REDRAW_POINT(const std::string& name,
                 const core_XYZ&    point,
                 const core_Color&  color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_POINT(name, point, color);
  }

//---------------------------------------------------------------------------//

  void
    DRAW_POINTS(const std::vector<core_XYZ>& coords,
                const core_Color&            color,
                const std::string&           name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_POINTS(coords, color, name);
  }

//---------------------------------------------------------------------------//

  void
    REDRAW_POINTS(const std::string&           name,
                  const std::vector<core_XYZ>& coords,
                  const core_Color&            color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_POINTS(name, coords, color);
  }

//---------------------------------------------------------------------------//

  void
    DRAW_VECTOR_AT(const core_XYZ&    P,
                   const core_XYZ&    V,
                   const core_Color&  color,
                   const std::string& name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_VECTOR_AT(P, V, color, name);
  }

//---------------------------------------------------------------------------//

  void
    REDRAW_VECTOR_AT(const std::string& name,
                     const core_XYZ&    P,
                     const core_XYZ&    V,
                     const core_Color&  color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_VECTOR_AT(name, P, V, color);
  }

//---------------------------------------------------------------------------//

  void
    DRAW_CURVE(const t_ptr<geom_Curve>& curve,
               const core_Color&        color,
               const std::string&       name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_CURVE(curve, color, name);
  }

//---------------------------------------------------------------------------//

  void
    REDRAW_CURVE(const std::string&       name,
                 const t_ptr<geom_Curve>& curve,
                 const core_Color&        color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_CURVE(name, curve, color);
  }

//---------------------------------------------------------------------------//

  void
    DRAW_SURFACE(const t_ptr<geom_Surface>& surface,
                 const core_Color&          color,
                 const std::string&         name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_SURFACE(surface, color, name);
  }

//---------------------------------------------------------------------------//

  void
    DRAW_SURFACE(const t_ptr<geom_Surface>& surface,
                 const core_Color&          color,
                 const double               opacity,
                 const std::string&         name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_SURFACE(surface, color, opacity, name);
  }

//---------------------------------------------------------------------------//

  void
    DRAW_SURFACE(const t_ptr<geom_Surface>& surface,
                 const double               uMin,
                 const double               uMax,
                 const double               vMin,
                 const double               vMax,
                 const core_Color&          color,
                 const std::string&         name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_SURFACE(surface, uMin, uMax, vMin, vMax, color, name);
  }

//---------------------------------------------------------------------------//

  void
    DRAW_SURFACE(const t_ptr<geom_Surface>& surface,
                 const double               uMin,
                 const double               uMax,
                 const double               vMin,
                 const double               vMax,
                 const core_Color&          color,
                 const double               opacity,
                 const std::string&         name = "")
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_SURFACE(surface, uMin, uMax, vMin, vMax, color, opacity, name);
  }

//---------------------------------------------------------------------------//

  void
    REDRAW_SURFACE(const std::string&         name,
                   const t_ptr<geom_Surface>& surface,
                   const core_Color&          color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_SURFACE(name, surface, color);
  }

//---------------------------------------------------------------------------//

  virtual void
    REDRAW_SURFACE(const std::string&         name,
                   const t_ptr<geom_Surface>& surface,
                   const core_Color&          color,
                   const double               opacity)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_SURFACE(name, surface, color, opacity);
  }

//---------------------------------------------------------------------------//

  virtual void
    REDRAW_SURFACE(const std::string&         name,
                   const t_ptr<geom_Surface>& surface,
                   const double               uMin,
                   const double               uMax,
                   const double               vMin,
                   const double               vMax,
                   const core_Color&          color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_SURFACE(name, surface, uMin, uMax, vMin, vMax, color);
  }

//---------------------------------------------------------------------------//

  virtual void
    REDRAW_SURFACE(const std::string&         name,
                   const t_ptr<geom_Surface>& surface,
                   const double               uMin,
                   const double               uMax,
                   const double               vMin,
                   const double               vMax,
                   const core_Color&          color,
                   const double               opacity)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_SURFACE(name, surface, uMin, uMax, vMin, vMax, color, opacity);
  }

//---------------------------------------------------------------------------//

  virtual void
    DRAW_LINK(const core_XYZ&    P1,
              const core_XYZ&    P2,
              const core_Color&  color,
              const std::string& name)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_LINK(P1, P2, color, name);
  }

//---------------------------------------------------------------------------//

  virtual void
    DRAW_LINK(const core_UV&     P1,
              const core_UV&     P2,
              const core_Color&  color,
              const std::string& name)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->DRAW_LINK(P1, P2, color, name);
  }

//---------------------------------------------------------------------------//

  virtual void
    REDRAW_LINK(const std::string& name,
                const core_XYZ&    P1,
                const core_XYZ&    P2,
                const core_Color&  color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_LINK(name, P1, P2, color);
  }

//---------------------------------------------------------------------------//

  virtual void
    REDRAW_LINK(const std::string& name,
                const core_UV&     P1,
                const core_UV&     P2,
                const core_Color&  color)
  {
    if ( m_iv.IsNull() ) return;
    //
    m_iv->REDRAW_LINK(name, P1, P2, color);
  }

private:

  t_ptr<core_IPlotter> m_iv; //!< IV instance.

};

}

#endif
