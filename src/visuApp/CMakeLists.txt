project (mobiusVisuApp)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
)

set (CPP_FILES
  visuApp_Main.cpp
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES visuApp)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

#------------------------------------------------------------------------------

# Create include variable
set (visuApp_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (visuApp_include_dir ${visuApp_include_dir_loc} PARENT_SCOPE)

foreach (FILE ${H_FILES})
  source_group ("Header Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${CPP_FILES})
  source_group ("Source Files" FILES "${FILE}")
endforeach (FILE)

include_directories(${visuApp_include_dir_loc} ${core_include_dir} ${bspl_include_dir} ${geom_include_dir} ${poly_include_dir})

add_executable (mobiusVisuApp
  ${H_FILES} ${CPP_FILES}
)

# Set WinMain() as an entry point as we need a non-console app.
set_target_properties(mobiusVisuApp PROPERTIES WIN32_EXECUTABLE TRUE)

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

configure_file(${CMAKE_SOURCE_DIR}/cmake/templates/exePROTOTYPE.vcxproj.user.in
               ${mobiusVisuApp_BINARY_DIR}/mobiusVisuApp.vcxproj.user @ONLY)

configure_file(${CMAKE_SOURCE_DIR}/cmake/templates/setenv.bat
               ${mobiusVisuApp_BINARY_DIR}/setenv.bat @ONLY)

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusVisuApp mobiusVisu mobiusCore mobiusPoly mobiusBSpl mobiusGeom)
