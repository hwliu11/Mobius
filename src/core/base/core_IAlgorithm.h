//-----------------------------------------------------------------------------
// Created on: July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2017-present, Sergey Slyadnev
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

#ifndef core_IAlgorithm_HeaderFile
#define core_IAlgorithm_HeaderFile

// Core includes
#include <mobius/core_IPlotter.h>
#include <mobius/core_IProgressNotifier.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Interface for algorithms.
class core_IAlgorithm : public core_OBJECT
{
public:

  core_ProgressEntry& GetProgress() const { return m_progress; }
  core_PlotterEntry&  GetPlotter()  const { return m_plotter; }

protected:

  mobiusCore_EXPORT
    core_IAlgorithm(core_ProgressEntry progress,
                    core_PlotterEntry  plotter);

protected:

  mutable core_ProgressEntry m_progress; //!< Progress Notifier.
  mutable core_PlotterEntry  m_plotter;  //!< Imperative Plotter.

protected:

  core_IAlgorithm() : core_OBJECT() {}

};

};

#endif
