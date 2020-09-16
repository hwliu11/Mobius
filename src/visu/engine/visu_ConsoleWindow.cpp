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

// Own include
#include <mobius/visu_ConsoleWindow.h>

// visu includes
#include <mobius/visu_CommandRepo.h>
#include <mobius/visu_ExitCmd.h>

// STD includes
#include <strsafe.h>
#include <stdio.h>
#include <string>

//-----------------------------------------------------------------------------
// Some predefined commands
//-----------------------------------------------------------------------------

#define QR_CMD_PROMPT   "Mobius> "
#define QR_CMD_BUF_SIZE 255

//-----------------------------------------------------------------------------
// Construction and destruction
//-----------------------------------------------------------------------------

//! Constructor.
//! \param Queue [in] command queue.
//! \param CmdRepo [in] command repo.
mobius::visu_ConsoleWindow::visu_ConsoleWindow(const t_ptr<visu_CommandQueue>& Queue,
                                               const t_ptr<visu_CommandRepo>& CmdRepo)
: m_queue(Queue),
  m_cmdRepo(CmdRepo)
{
}

//! Destructor.
mobius::visu_ConsoleWindow::~visu_ConsoleWindow()
{
}

//! Creates new Console Window.
//! \return true in case of success, false -- otherwise.
bool mobius::visu_ConsoleWindow::Create()
{
  if ( AllocConsole() )
  {
    FILE *stream;
    SetConsoleTitleA("Mobius >>> Console");
    freopen_s(&stream, "CONIN$", "r", stdin);
    freopen_s(&stream, "CONOUT$", "wb", stdout);
    freopen_s(&stream, "CONOUT$", "wb", stderr);
    return true;
  }

  return false;
}

//! Starts message loop.
void mobius::visu_ConsoleWindow::StartMessageLoop()
{
  bool stopPrompt = false;
  do
  {
    // Get last command from queue until it is becomes the one we can handle
    t_ptr<visu_BaseCmd> LastCommand;
    bool canProceed = false;
    do
    {
      LastCommand = m_queue->Last();
      if ( LastCommand.IsNull() )
        canProceed = true;
      else
      {
        // Check command type: we can proceed only with console ones
        visu_ConsoleCmd* CmdPtr = dynamic_cast<visu_ConsoleCmd*>( LastCommand.Access() );
        if ( CmdPtr )
          canProceed = true;
      }
    }
    while ( !canProceed );

    // If there is something to proceed, let us do it
    if ( !LastCommand.IsNull() )
    {
      // Check if it is a standard 'exit' command
      visu_ExitCmd* ExitPtr = dynamic_cast<visu_ExitCmd*>( LastCommand.Access() );
      if ( ExitPtr )
        stopPrompt = true;

      // Remove command from queue and execute it
      m_queue->Pop();
      if ( !LastCommand->Execute() )
        DisplayMessage("Console", "Command failed!", false);

      // Has 'exit' command been pushed?
      if ( stopPrompt )
        continue;
    }

    // Get next command from user
    std::cout << QR_CMD_PROMPT;
    std::string inputStr;
    std::getline(std::cin, inputStr);

    // Split command name from arguments
    std::vector<std::string> chunks;
    core::str::split(inputStr, " ", chunks);

    // Proceed with command by name
    if ( !chunks.size() )
      DisplayMessage("Console", "Empty command (?!)", false);
    else
    {
      // Find command by name and push it to queue
      t_ptr<visu_BaseCmd> NewCommand = m_cmdRepo->FindCommand( chunks[0] );
      if ( NewCommand.IsNull() )
        DisplayMessage("Console", "Command does not exist", false);
      else
      {
        // Extract arguments
        if ( chunks.size() > 1 )
        {
          // Remove command name, so as only arguments will be kept
          chunks.erase( chunks.begin() );

          // Set arguments to command
          NewCommand->SetArguments(chunks);
        }
        else
          NewCommand->SetArguments( std::vector<std::string>() ); // Clean the arguments (if any)

        // Push command to the shared queue
        m_queue->Push(NewCommand);
      }
    }
  }
  while ( !stopPrompt );
}

//! Prints message from the given client (its user-friendly name should be
//! provided).
//! \param From [in] client name.
//! \param Message [in] message to display.
//! \param newPrompt [in] indicates whether to ask for a new prompt.
void mobius::visu_ConsoleWindow::DisplayMessage(const std::string& From,
                                                const std::string& Message,
                                                const bool newPrompt)
{
  std::cout << "`" << From.c_str() << "` says: \"" << Message.c_str() << "\"" << std::endl;
  if ( newPrompt )
    std::cout << QR_CMD_PROMPT;
}
