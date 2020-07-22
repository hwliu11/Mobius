//-----------------------------------------------------------------------------
// Created on: 30 April 2014
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

#ifndef visu_ConsoleWindow_HeaderFile
#define visu_ConsoleWindow_HeaderFile

// Win API
#include <windows.h>

// visu includes
#include <mobius/visu_CommandQueue.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing command window.
class visu_ConsoleWindow
{
public:

  mobiusVisu_EXPORT
    visu_ConsoleWindow(const t_ptr<visu_CommandQueue>& Queue,
                         const t_ptr<visu_CommandRepo>& CmdRepo);

  mobiusVisu_EXPORT virtual
    ~visu_ConsoleWindow();

public:

  mobiusVisu_EXPORT virtual bool
    Create();

  mobiusVisu_EXPORT virtual void
    StartMessageLoop();

public:

  mobiusVisu_EXPORT static void
    DisplayMessage(const std::string& From,
                   const std::string& Message,
                   const bool newPrompt = true);

private:

  t_ptr<visu_CommandQueue> m_queue;   //!< Command queue.
  t_ptr<visu_CommandRepo>  m_cmdRepo; //!< Command repo.

};

}

#endif
