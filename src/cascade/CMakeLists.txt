project (mobiusCascade)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  cascade.h
  cascade_BSplineCurve.h
  cascade_BSplineSurface.h
  cascade_MultResolver.h
  cascade_Triangulation.h
)
set (CPP_FILES
  cascade.cpp
  cascade_BSplineCurve.cpp
  cascade_BSplineSurface.cpp
  cascade_MultResolver.cpp
  cascade_Triangulation.cpp
)

#------------------------------------------------------------------------------
# OpenCascade
#------------------------------------------------------------------------------

set (OCCT_LIB_FILES
  TKernel
  TKMath
  TKBRep
  TKTopAlgo
  TKG2d
  TKG3d
  TKGeomBase
  TKGeomAlgo
  TKMesh
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES cascade)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

#------------------------------------------------------------------------------

# Create include variable
set (cascade_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (cascade_include_dir ${cascade_include_dir_loc} PARENT_SCOPE)

foreach (FILE ${H_FILES})
  source_group ("Header Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${CPP_FILES})
  source_group ("Source Files" FILES "${FILE}")
endforeach (FILE)

include_directories(SYSTEM ${cascade_include_dir_loc} ${core_include_dir} ${bspl_include_dir} ${geom_include_dir} ${3RDPARTY_OCCT_INCLUDE_DIR})

if (USE_ADOLC)
  include_directories(${3RDPARTY_adolc_INCLUDE_DIR})
endif()

add_library ( mobiusCascade SHARED ${H_FILES} ${CPP_FILES} )

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusCascade mobiusCore mobiusPoly mobiusBSpl mobiusGeom)

foreach (LIB_FILE ${OCCT_LIB_FILES})
  if (WIN32)
    set (LIB_FILENAME "${LIB_FILE}${CMAKE_STATIC_LIBRARY_SUFFIX}")
  else()
    set (LIB_FILENAME "lib${LIB_FILE}${CMAKE_SHARED_LIBRARY_SUFFIX}")
  endif()

  if (3RDPARTY_OCCT_LIBRARY_DIR_DEBUG AND EXISTS "${3RDPARTY_OCCT_LIBRARY_DIR_DEBUG}/${LIB_FILENAME}")
    target_link_libraries (mobiusCascade debug ${3RDPARTY_OCCT_LIBRARY_DIR_DEBUG}/${LIB_FILENAME})
    target_link_libraries (mobiusCascade optimized ${3RDPARTY_OCCT_LIBRARY_DIR}/${LIB_FILENAME})
  else()
    target_link_libraries (mobiusCascade ${3RDPARTY_OCCT_LIBRARY_DIR}/${LIB_FILENAME})
  endif()
endforeach()

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusCascade
         CONFIGURATIONS Release
         RUNTIME DESTINATION bin COMPONENT Runtime
         ARCHIVE DESTINATION lib COMPONENT Library
         LIBRARY DESTINATION lib COMPONENT Library)

install (TARGETS mobiusCascade
         CONFIGURATIONS RelWithDebInfo
         RUNTIME DESTINATION bini COMPONENT Runtime
         ARCHIVE DESTINATION libi COMPONENT Library
         LIBRARY DESTINATION libi COMPONENT Library)

install (TARGETS mobiusCascade
         CONFIGURATIONS Debug
         RUNTIME DESTINATION bind COMPONENT Runtime
         ARCHIVE DESTINATION libd COMPONENT Library
         LIBRARY DESTINATION libd COMPONENT Library)

install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusCascade.pdb DESTINATION bind CONFIGURATIONS Debug)
install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bini/mobiusCascade.pdb DESTINATION bini CONFIGURATIONS RelWithDebInfo)

install (FILES ${H_FILES} DESTINATION include/mobius)
