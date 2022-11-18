project (mobiusVisu)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  visu.h
)
set (CPP_FILES
)

#------------------------------------------------------------------------------
# Actors
#------------------------------------------------------------------------------

set (actors_bspl_H_FILES
  actors/bspl/visu_ActorBSplBasis.h
  actors/bspl/visu_ActorBSplBasisDers.h
  actors/bspl/visu_ActorFunction.h
  actors/bspl/visu_ActorFunctionLevels.h
)
set (actors_bspl_CPP_FILES
  actors/bspl/visu_ActorBSplBasis.cpp
  actors/bspl/visu_ActorBSplBasisDers.cpp
  actors/bspl/visu_ActorFunction.cpp
  actors/bspl/visu_ActorFunctionLevels.cpp
)

set (actors_geom_H_FILES
  actors/geom/visu_ActorBezierOnRailsSurface.h
  actors/geom/visu_ActorBSplCurve.h
  actors/geom/visu_ActorBSplSurface.h
  actors/geom/visu_ActorInsensitiveSurface.h
  actors/geom/visu_ActorKleinBottle.h
  actors/geom/visu_ActorLine.h
  actors/geom/visu_ActorPolyLine.h
  actors/geom/visu_ActorPositionCloud.h
  actors/geom/visu_ActorSectionCloud.h
  actors/geom/visu_ActorSphereSurface.h
  actors/geom/visu_ActorSurface.h
  actors/geom/visu_ActorVectorField.h
  actors/geom/visu_DataPositionCloud.h
)
set (actors_geom_CPP_FILES
  actors/geom/visu_ActorBezierOnRailsSurface.cpp
  actors/geom/visu_ActorBSplCurve.cpp
  actors/geom/visu_ActorBSplSurface.cpp
  actors/geom/visu_ActorInsensitiveSurface.cpp
  actors/geom/visu_ActorKleinBottle.cpp
  actors/geom/visu_ActorLine.cpp
  actors/geom/visu_ActorPolyLine.cpp
  actors/geom/visu_ActorPositionCloud.cpp
  actors/geom/visu_ActorSectionCloud.cpp
  actors/geom/visu_ActorSphereSurface.cpp
  actors/geom/visu_ActorSurface.cpp
  actors/geom/visu_ActorVectorField.cpp
  actors/geom/visu_DataPositionCloud.cpp
)

set (actors_misc_H_FILES
  actors/misc/visu_ActorCartesianAxes.h
)
set (actors_misc_CPP_FILES
  actors/misc/visu_ActorCartesianAxes.cpp
)

set (actors_poly_H_FILES
  actors/poly/visu_ActorMesh.h
)
set (actors_poly_CPP_FILES
  actors/poly/visu_ActorMesh.cpp
)

#------------------------------------------------------------------------------
# Commands
#------------------------------------------------------------------------------

set (commands_console_H_FILES
  commands/console/visu_DumpCmd.h
  commands/console/visu_ExitCmd.h
  commands/console/visu_ExplodeCmd.h
  commands/console/visu_HelloWorldCmd.h
  commands/console/visu_ListCmd.h
  commands/console/visu_PloadCmd.h
  commands/console/visu_SaveCmd.h
)
set (commands_console_CPP_FILES
  commands/console/visu_DumpCmd.cpp
  commands/console/visu_ExitCmd.cpp
  commands/console/visu_ExplodeCmd.cpp
  commands/console/visu_HelloWorldCmd.cpp
  commands/console/visu_ListCmd.cpp
  commands/console/visu_PloadCmd.cpp
  commands/console/visu_SaveCmd.cpp
)

set (commands_view_H_FILES
  commands/view/visu_ActorsCmd.h
  commands/view/visu_BCurveCmd.h
  commands/view/visu_BezierOnRailsCmd.h
  commands/view/visu_BSplBasisCmd.h
  commands/view/visu_ClearCmd.h
  commands/view/visu_CreateSectionCmd.h
  commands/view/visu_GeneralCloudCmd.h
  commands/view/visu_GLVersionCmd.h
  commands/view/visu_HedgehogCmd.h
  commands/view/visu_HedgehogUCmd.h
  commands/view/visu_HedgehogVCmd.h
  commands/view/visu_InsertKnotCmd.h
  commands/view/visu_Interpc01Cmd.h
  commands/view/visu_Interpc02Cmd.h
  commands/view/visu_Interpc03Cmd.h
  commands/view/visu_InterpSectionsCmd.h
  commands/view/visu_InterpSurfCmd.h
  commands/view/visu_KleinBottleCmd.h
  commands/view/visu_LoadSTLCmd.h
  commands/view/visu_PlanarCloudCmd.h
  commands/view/visu_PolesCmd.h
  commands/view/visu_RefineKnotsCmd.h
  commands/view/visu_ReparamCmd.h
  commands/view/visu_SphereCmd.h
  commands/view/visu_SplitCurveCmd.h
  commands/view/visu_UnifyCmd.h
)
set (commands_view_CPP_FILES
  commands/view/visu_BSplBasisCmd.cpp
  commands/view/visu_CreateSectionCmd.cpp
  commands/view/visu_InterpSectionsCmd.cpp
  commands/view/visu_InterpSurfCmd.cpp
  commands/view/visu_LoadSTLCmd.cpp
)

#------------------------------------------------------------------------------
# Engine
#------------------------------------------------------------------------------

set (engine_H_FILES
  engine/visu_Actor.h
  engine/visu_ActorInsensitive.h
  engine/visu_BaseCmd.h
  engine/visu_Camera.h
  engine/visu_ColorSelector.h
  engine/visu_CommandQueue.h
  engine/visu_CommandRepo.h
  engine/visu_ConsoleCmd.h
  engine/visu_ConsoleWindow.h
  engine/visu_DataSet.h
  engine/visu_LibDefinition.h
  engine/visu_Picker.h
  engine/visu_Scene.h
  engine/visu_SharedQueue.h
  engine/visu_UniqueName.h
  engine/visu_Utils.h
  engine/visu_VarRepo.h
  engine/visu_ViewCmd.h
  engine/visu_ViewWindow.h
)
set (engine_CPP_FILES
  engine/visu_Actor.cpp
  engine/visu_ActorInsensitive.cpp
  engine/visu_BaseCmd.cpp
  engine/visu_Camera.cpp
  engine/visu_ColorSelector.cpp
  engine/visu_CommandRepo.cpp
  engine/visu_ConsoleCmd.cpp
  engine/visu_ConsoleWindow.cpp
  engine/visu_DataSet.cpp
  engine/visu_Picker.cpp
  engine/visu_Scene.cpp
  engine/visu_SharedQueue.cpp
  engine/visu_UniqueName.cpp
  engine/visu_Utils.cpp
  engine/visu_VarRepo.cpp
  engine/visu_ViewCmd.cpp
  engine/visu_ViewWindow.cpp
)

set (engine_GL_H_FILES
  engine/GL/ARB_Multisample.h
  engine/GL/glext.h
  engine/GL/wglext.h
)
set (engine_GL_CPP_FILES
  engine/GL/ARB_Multisample.cpp
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES visu)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

#------------------------------------------------------------------------------

# Create include variable
set (visu_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (visu_include_dir ${visu_include_dir_loc} PARENT_SCOPE)

foreach (FILE ${H_FILES})
  source_group ("Header Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${CPP_FILES})
  source_group ("Source Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${actors_bspl_H_FILES})
  source_group ("Header Files\\Actors\\BSpl" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${actors_bspl_CPP_FILES})
  source_group ("Source Files\\Actors\\BSpl" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${actors_geom_H_FILES})
  source_group ("Header Files\\Actors\\Geom" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${actors_geom_CPP_FILES})
  source_group ("Source Files\\Actors\\Geom" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${actors_misc_H_FILES})
  source_group ("Header Files\\Actors\\Misc" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${actors_misc_CPP_FILES})
  source_group ("Source Files\\Actors\\Misc" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${actors_poly_H_FILES})
  source_group ("Header Files\\Actors\\Poly" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${actors_poly_CPP_FILES})
  source_group ("Source Files\\Actors\\Poly" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${commands_console_H_FILES})
  source_group ("Header Files\\Commands\\Console" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${commands_console_CPP_FILES})
  source_group ("Source Files\\Commands\\Console" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${commands_view_H_FILES})
  source_group ("Header Files\\Commands\\View" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${commands_view_CPP_FILES})
  source_group ("Source Files\\Commands\\View" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${engine_H_FILES})
  source_group ("Header Files\\Engine" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${engine_CPP_FILES})
  source_group ("Source Files\\Engine" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${engine_GL_H_FILES})
  source_group ("Header Files\\Engine\\GL" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${engine_GL_CPP_FILES})
  source_group ("Source Files\\Engine\\GL" FILES "${FILE}")
endforeach (FILE)

include_directories(${visu_include_dir_loc} ${core_include_dir} ${bspl_include_dir} ${geom_include_dir} ${3RDPARTY_EIGEN_DIR})

add_library (mobiusVisu SHARED
  ${H_FILES}                  ${CPP_FILES}
  ${actors_bspl_H_FILES}      ${actors_bspl_CPP_FILES}
  ${actors_geom_H_FILES}      ${actors_geom_CPP_FILES}
  ${actors_misc_H_FILES}      ${actors_misc_CPP_FILES}
  ${actors_poly_H_FILES}      ${actors_poly_CPP_FILES}
  ${commands_console_H_FILES} ${commands_console_CPP_FILES}
  ${commands_view_H_FILES}    ${commands_view_CPP_FILES}
  ${engine_H_FILES}           ${engine_CPP_FILES}
  ${engine_GL_H_FILES}        ${engine_GL_CPP_FILES})

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusVisu mobiusCore mobiusBSpl mobiusGeom mobiusPoly)

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusVisu
         CONFIGURATIONS Release
         RUNTIME DESTINATION bin COMPONENT Runtime
         ARCHIVE DESTINATION lib COMPONENT Library
         LIBRARY DESTINATION lib COMPONENT Library)

install (TARGETS mobiusVisu
         CONFIGURATIONS RelWithDebInfo
         RUNTIME DESTINATION bini COMPONENT Runtime
         ARCHIVE DESTINATION libi COMPONENT Library
         LIBRARY DESTINATION libi COMPONENT Library)

install (TARGETS mobiusVisu
         CONFIGURATIONS Debug
         RUNTIME DESTINATION bind COMPONENT Runtime
         ARCHIVE DESTINATION libd COMPONENT Library
         LIBRARY DESTINATION libd COMPONENT Library)

install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusVisu.pdb DESTINATION bind CONFIGURATIONS Debug)

install (FILES ${H_FILES}                  DESTINATION include/mobius)
install (FILES ${actors_bspl_H_FILES}      DESTINATION include/mobius)
install (FILES ${actors_geom_H_FILES}      DESTINATION include/mobius)
install (FILES ${actors_misc_H_FILES}      DESTINATION include/mobius)
install (FILES ${actors_poly_H_FILES}      DESTINATION include/mobius)
install (FILES ${commands_console_H_FILES} DESTINATION include/mobius)
install (FILES ${commands_view_H_FILES}    DESTINATION include/mobius)
install (FILES ${engine_H_FILES}           DESTINATION include/mobius)
install (FILES ${engine_GL_H_FILES}        DESTINATION include/mobius)
