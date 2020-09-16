//-----------------------------------------------------------------------------
// Created on: 14 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// visu includes
#include <mobius/visu_BezierOnRailsCmd.h>
#include <mobius/visu_CommandRepo.h>
#include <mobius/visu_ConsoleWindow.h>
#include <mobius/visu_GLVersionCmd.h>
#include <mobius/visu_LibDefinition.h>
#include <mobius/visu_SharedQueue.h>
#include <mobius/visu_ViewWindow.h>

// Console commands
#include <mobius/visu_DumpCmd.h>
#include <mobius/visu_ExitCmd.h>
#include <mobius/visu_ExplodeCmd.h>
#include <mobius/visu_HelloWorldCmd.h>
#include <mobius/visu_ListCmd.h>
#include <mobius/visu_PloadCmd.h>
#include <mobius/visu_SaveCmd.h>

// View commands
#include <mobius/visu_ActorsCmd.h>
#include <mobius/visu_BCurveCmd.h>
#include <mobius/visu_BSplBasisCmd.h>
#include <mobius/visu_ClearCmd.h>
#include <mobius/visu_CreateSectionCmd.h>
#include <mobius/visu_GeneralCloudCmd.h>
#include <mobius/visu_HedgehogCmd.h>
#include <mobius/visu_HedgehogUCmd.h>
#include <mobius/visu_HedgehogVCmd.h>
#include <mobius/visu_InsertKnotCmd.h>
#include <mobius/visu_Interpc01Cmd.h>
#include <mobius/visu_Interpc02Cmd.h>
#include <mobius/visu_Interpc03Cmd.h>
#include <mobius/visu_InterpSectionsCmd.h>
#include <mobius/visu_InterpSurfCmd.h>
#include <mobius/visu_KleinBottleCmd.h>
#include <mobius/visu_PlanarCloudCmd.h>
#include <mobius/visu_PolesCmd.h>
#include <mobius/visu_RefineKnotsCmd.h>
#include <mobius/visu_ReparamCmd.h>
#include <mobius/visu_SphereCmd.h>
#include <mobius/visu_SplitCurveCmd.h>
#include <mobius/visu_UnifyCmd.h>

// STD includes
#include <windows.h>

using namespace mobius;

//! Application utilities.
namespace AppUtils
{
  t_ptr<visu_CommandQueue> Queue;   //!< Command queue.
  t_ptr<visu_CommandRepo>  CmdRepo; //!< Command repository.
}

//-----------------------------------------------------------------------------
// Register commands
//-----------------------------------------------------------------------------

//! Registers console commands.
void RegisterConsoleCommands()
{
  AppUtils::CmdRepo->RegisterCommand( "exit",    new visu_ExitCmd       (AppUtils::CmdRepo) );
  AppUtils::CmdRepo->RegisterCommand( "hello",   new visu_HelloWorldCmd (AppUtils::CmdRepo) );
  AppUtils::CmdRepo->RegisterCommand( "list",    new visu_ListCmd       (AppUtils::CmdRepo) );
  AppUtils::CmdRepo->RegisterCommand( "dump",    new visu_DumpCmd       (AppUtils::CmdRepo) );
  AppUtils::CmdRepo->RegisterCommand( "pload",   new visu_PloadCmd      (AppUtils::CmdRepo) );
  AppUtils::CmdRepo->RegisterCommand( "save",    new visu_SaveCmd       (AppUtils::CmdRepo) );
  AppUtils::CmdRepo->RegisterCommand( "explode", new visu_ExplodeCmd    (AppUtils::CmdRepo) );
}

//! Registers visualization commands.
//! \param Picker [in] Picker instance.
void RegisterViewerCommands(const t_ptr<visu_Picker>& Picker)
{
  AppUtils::CmdRepo->RegisterCommand( "load-stl",                new visu_LoadStlCmd            (AppUtils::CmdRepo, Picker) );
  //
  AppUtils::CmdRepo->RegisterCommand( "actors scene",            new visu_ActorsCmd             (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "interp01",                new visu_Interpc01Cmd          (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "interp02",                new visu_Interpc02Cmd          (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "interp03",                new visu_Interpc03Cmd          (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "clear erase null killit", new visu_ClearCmd              (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "ptest",                   new visu_PlanarCloudCmd        (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "test",                    new visu_GeneralCloudCmd       (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "brails",                  new visu_BezierOnRailsCmd      (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "glver",                   new visu_GLVersionCmd          (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "sinterp",                 new visu_InterpSurfCmd         (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "sphere",                  new visu_SphereCmd             (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "klein",                   new visu_KleinBottleCmd        (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "c3d_bspl",                new visu_BCurveCmd             (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "hedgehog combs hh",       new visu_HedgehogCmd           (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "hedgehog_u combs_u hh_u", new visu_HedgehogUCmd          (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "hedgehog_v combs_v hh_v", new visu_HedgehogVCmd          (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "bspl",                    new visu_BSplBasisCmd          (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "poles",                   new visu_PolesCmd              (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "interp_sections",         new visu_InterpSectionsCmd     (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "insert_knot",             new visu_InsertKnotCmd         (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "refine_knots",            new visu_RefineKnotsCmd        (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "split_curve",             new visu_SplitCurveCmd         (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "create_section",          new visu_CreateSectionCmd      (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "reparam",                 new visu_ReparamCmd            (AppUtils::CmdRepo, Picker) );
  AppUtils::CmdRepo->RegisterCommand( "unify",                   new visu_UnifyCmd              (AppUtils::CmdRepo, Picker) );
}

//-----------------------------------------------------------------------------
// 3D Viewer
//-----------------------------------------------------------------------------

//! Working routine for visualization thread.
DWORD WINAPI Thread_OpenGL(LPVOID)
{
  // Create Viewer
  visu_ViewWindow ViewWindow(AppUtils::Queue, AppUtils::CmdRepo);
  if ( !ViewWindow.Create(32, 32, 512, 512) )
    return 1;

  // Register visualization commands
  RegisterViewerCommands( ViewWindow.Picker() );

  // Message loop
  ViewWindow.ResetScene();
  ViewWindow.StartMessageLoop();
  ViewWindow.Close();

  return 0;
}

//-----------------------------------------------------------------------------
// Console thread
//-----------------------------------------------------------------------------

//! Working routine for console thread.
DWORD WINAPI Thread_Console(LPVOID core_NotUsed(lpParam))
{
  // Create consol
  visu_ConsoleWindow ConsoleWindow(AppUtils::Queue, AppUtils::CmdRepo);
  if ( !ConsoleWindow.Create() )
    return 1;

  // Register console commands in global repository
  RegisterConsoleCommands();

  // Start message loop
  ConsoleWindow.StartMessageLoop();

  return 0;
}

//-----------------------------------------------------------------------------
// Entry point
//-----------------------------------------------------------------------------

//! Main().
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
  // Create command queue in the main thread
  if ( AppUtils::Queue.IsNull() )
    AppUtils::Queue = new visu_CommandQueue;

  // Create command repo in the main thread
  if ( AppUtils::CmdRepo.IsNull() )
    AppUtils::CmdRepo = new visu_CommandRepo;

  // Create thread for Viewer
  HANDLE hViewerThread = CreateThread(NULL, 0, Thread_OpenGL, NULL, 0, NULL);
  if ( !hViewerThread )
    ExitProcess(NULL);

  // Create thread for Console
  HANDLE hConsoleThread = CreateThread(NULL, 0, Thread_Console, NULL, 0, NULL);
  if ( !hConsoleThread )
    ExitProcess(NULL);

  // Aray to store thread handles
  HANDLE hThreads[] = {hViewerThread, hConsoleThread};

  // NOTICE: we pass FALSE here as we do not want to have the Viewer opened
  //         while the Console is closed and vice versa. Once such behaviour
  //         becomes acceptable, change the bWaitAll to TRUE, so this
  //         barrier will be passed only then ALL threads are signaling
  WaitForMultipleObjects(2, hThreads, FALSE, INFINITE);

  // Close all thread handles upon completion
  CloseHandle(hViewerThread);
  CloseHandle(hConsoleThread);
}
