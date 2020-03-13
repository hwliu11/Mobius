project (mobiusPoly)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  poly.h
  poly_Edge.h
  poly_Handles.h
  poly_Mesh.h
  poly_Quad.h
  poly_ReadOBJ.h
  poly_ReadPLY.h
  poly_ReadSTL.h
  poly_Triangle.h
  poly_Vertex.h
)
set (CPP_FILES
  poly_Edge.cpp
  poly_Mesh.cpp
  poly_Quad.cpp
  poly_ReadOBJ.cpp
  poly_ReadPLY.cpp
  poly_ReadSTL.cpp
  poly_Triangle.cpp
  poly_Vertex.cpp
)

#------------------------------------------------------------------------------
# Fields
#------------------------------------------------------------------------------

set (fields_H_FILES
  fields/poly_BooleanFunc.h
  fields/poly_DistanceField.h
  fields/poly_DistanceFunc.h
  fields/poly_GridTessellator.h
  fields/poly_MarchingCubes.h
  fields/poly_RealFunc.h
  fields/poly_SVO.h
  fields/poly_Tessellator.h
  fields/poly_UnionFunc.h
)
set (fields_CPP_FILES
  fields/poly_DistanceField.cpp
  fields/poly_DistanceFunc.cpp
  fields/poly_GridTessellator.cpp
  fields/poly_MarchingCubes.cpp
  fields/poly_SVO.cpp
  fields/poly_Tessellator.cpp
  fields/poly_UnionFunc.cpp
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES poly)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

#------------------------------------------------------------------------------

# Create include variable
set (poly_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (poly_include_dir ${poly_include_dir_loc} PARENT_SCOPE)

foreach (FILE ${H_FILES})
  source_group ("Header Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${CPP_FILES})
  source_group ("Source Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${fields_H_FILES})
  source_group ("Header Files\\Fields" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${fields_CPP_FILES})
  source_group ("Source Files\\Fields" FILES "${FILE}")
endforeach (FILE)

include_directories(${poly_include_dir_loc} ${core_include_dir} ${3RDPARTY_EIGEN_DIR})

if (USE_ADOLC)
  include_directories(${3RDPARTY_adolc_INCLUDE_DIR})
endif()

add_library (mobiusPoly SHARED
  ${H_FILES}        ${CPP_FILES}
  ${fields_H_FILES} ${fields_CPP_FILES}
)

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusPoly mobiusCore)

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusPoly
         CONFIGURATIONS Release
         RUNTIME DESTINATION bin COMPONENT Runtime
         ARCHIVE DESTINATION lib COMPONENT Library
         LIBRARY DESTINATION lib COMPONENT Library)

install (TARGETS mobiusPoly
         CONFIGURATIONS RelWithDebInfo
         RUNTIME DESTINATION bini COMPONENT Runtime
         ARCHIVE DESTINATION libi COMPONENT Library
         LIBRARY DESTINATION libi COMPONENT Library)

install (TARGETS mobiusPoly
         CONFIGURATIONS Debug
         RUNTIME DESTINATION bind COMPONENT Runtime
         ARCHIVE DESTINATION libd COMPONENT Library
         LIBRARY DESTINATION libd COMPONENT Library)

install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusPoly.pdb DESTINATION bind CONFIGURATIONS Debug)
install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bini/mobiusPoly.pdb DESTINATION bini CONFIGURATIONS RelWithDebInfo)

install (FILES ${H_FILES}        DESTINATION include/mobius)
install (FILES ${fields_H_FILES} DESTINATION include/mobius)
