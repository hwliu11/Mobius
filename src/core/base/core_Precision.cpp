//-----------------------------------------------------------------------------
// Created on: 17 January 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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

// Own include
#include <mobius/core_Precision.h>

// Standard includes
#include <cfloat>

//! Precision in 3D space to cover rounding errors.
//! \return geometric precision to use in 3D.
double mobius::core_Precision::Resolution3D()
{
  return 1.0e-6;
}

//! \return squared 3D resolution.
double mobius::core_Precision::SquaredResolution3D()
{
  return Resolution3D() * Resolution3D();
}

//! Precision in 2D space to cover rounding errors.
//! \return geometric precision to use in 2D.
double mobius::core_Precision::Resolution2D()
{
  return 1.0e-8;
}

//! \return infinite value.
double mobius::core_Precision::Infinity()
{
  return 1.e+100;
}

//! Checks whether the passed value is infinite.
//! \param[in] val value to check.
//! \return true/false.
bool mobius::core_Precision::IsInfinite(const double val)
{
  return fabs(val) >= Infinity();
}

//! \return min representable floating-point value.
double mobius::core_Precision::RealSmall()
{
  return DBL_MIN;
}
