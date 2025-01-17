//-----------------------------------------------------------------------------
// Created on: 04 March 2015
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
#include <mobius/visu_ExplodeCmd.h>

// visu includes
#include <mobius/visu_CommandRepo.h>

// geom includes
#include <mobius/geom_SectionCloud.h>

//! Constructor.
//! \param cmd_repo [in] command repo.
mobius::visu_ExplodeCmd::visu_ExplodeCmd(const t_ptr<visu_CommandRepo>& cmd_repo)
: visu_ConsoleCmd(cmd_repo)
{}

//! Destructor.
mobius::visu_ExplodeCmd::~visu_ExplodeCmd()
{}

//! Executes command.
//! \return true in case of success, false -- otherwise.
bool mobius::visu_ExplodeCmd::Execute()
{
  std::cout << "Exploding object..." << std::endl;

  if ( this->Argc() != 1 )
  {
    std::cout << "Invalid number of arguments" << std::endl;
    std::cout << "Example: explode my_section_cloud" << std::endl;
    return false;
  }

  std::string obj_name = this->Arg(0);

  t_ptr<core_OBJECT> obj = this->CmdRepo()->Vars().FindVariable(obj_name);
  if ( obj.IsNull() )
  {
    std::cout << "There is no object with name " << obj_name.c_str() << std::endl;
    return false;
  }

  if ( !t_ptr<t_scloud>::DownCast(obj).IsNull() )
  {
    t_ptr<t_scloud> section_cloud = t_ptr<t_scloud>::DownCast(obj);

    // Extract sections and register them as variables
    for ( size_t s = 0; s < section_cloud->GetNumberOfSections(); ++s )
    {
      std::string sct_name = obj_name + "_" + core::str::to_string<int>( (int) (s + 1) );
      const t_ptr<t_sline>& sct = section_cloud->GetSectionByIndex(s);
      this->CmdRepo()->Vars().RegisterVariable( sct_name, sct.Access() );

      std::cout << "Section " << sct_name.c_str()
                << " (" << sct->Pts->GetNumberOfPoints() << " points) was registered" << std::endl;
    }
  }
  else if ( !t_ptr<t_sline>::DownCast(obj).IsNull() )
  {
    t_ptr<t_sline> section_line = t_ptr<t_sline>::DownCast(obj);

    // Extract sections and register them as variables
    for ( size_t s = 0; s < section_line->Slices.size(); ++s )
    {
      std::string sct_name = obj_name + "_" + core::str::to_string<int>( (int) (s + 1) );
      const t_ptr<t_sline>& sct = section_line->Slices[s];

      if ( sct.IsNull() )
        continue;

      this->CmdRepo()->Vars().RegisterVariable( sct_name, sct.Access() );

      std::cout << "Section " << sct_name.c_str()
                << " (" << ( sct->Pts.IsNull() ? 0 : sct->Pts->GetNumberOfPoints() ) << " points) was registered" << std::endl;
    }
  }
  else
  {
    std::cout << "Currently we can explode only section clouds, sorry" << std::endl;
    return false;
  }

  return true;
}
