project (mobiusTestEngine)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  test.h
  test_CaseIDs.h
)

set (CPP_FILES
  test_Main.cpp
)

#------------------------------------------------------------------------------
# Basic splines
#------------------------------------------------------------------------------

set (bspl_H_FILES
  bspl/test_EffectiveN.h
  bspl/test_EffectiveNDers.h
  bspl/test_FindSpan.h
  bspl/test_KnotMultiset.h
  bspl/test_KnotsAverage.h
  bspl/test_N.h
  bspl/test_ParamsCentripetal.h
  bspl/test_ParamsChordLength.h
  bspl/test_ParamsUniform.h
  bspl/test_UnifyKnots.h
)

set (bspl_CPP_FILES
  bspl/test_EffectiveN.cpp
  bspl/test_EffectiveNDers.cpp
  bspl/test_FindSpan.cpp
  bspl/test_KnotMultiset.cpp
  bspl/test_KnotsAverage.cpp
  bspl/test_N.cpp
  bspl/test_ParamsCentripetal.cpp
  bspl/test_ParamsChordLength.cpp
  bspl/test_ParamsUniform.cpp
  bspl/test_UnifyKnots.cpp
)

#------------------------------------------------------------------------------
# Core
#------------------------------------------------------------------------------

set (core_H_FILES
  core/test_Quaternion.h
)

set (core_CPP_FILES
  core/test_Quaternion.cpp
)

#------------------------------------------------------------------------------
# Geom
#------------------------------------------------------------------------------

set (geom_H_FILES
  geom/test_InterpolateCurve3D.h
  geom/test_Line3D.h
  geom/test_PointOnLine.h
)

set (geom_CPP_FILES
  geom/test_InterpolateCurve3D.cpp
  geom/test_Line3D.cpp
  geom/test_PointOnLine.cpp
)

#------------------------------------------------------------------------------

# Create include variable
set (test_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (test_include_dir ${test_include_dir_loc} PARENT_SCOPE)

foreach (FILE ${H_FILES})
  source_group ("Header Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${CPP_FILES})
  source_group ("Source Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${reportgen_H_FILES})
  source_group ("Header Files\\Reporting" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${reportgen_CPP_FILES})
  source_group ("Source Files\\Reporting" FILES "${FILE}")
endforeach (FILE)

include_directories(${test_include_dir_loc} ${core_include_dir})
add_library (mobiusTest SHARED
  ${H_FILES}           ${CPP_FILES}
  ${reportgen_H_FILES} ${reportgen_CPP_FILES})

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusTest mobiusCore)

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusTest
         RUNTIME DESTINATION bin COMPONENT Runtime
         CONFIGURATIONS Release
         ARCHIVE DESTINATION lib COMPONENT Library
         CONFIGURATIONS Release)

install (TARGETS mobiusTest
         RUNTIME DESTINATION bind COMPONENT Runtime
         CONFIGURATIONS Debug
         ARCHIVE DESTINATION libd COMPONENT Library
         CONFIGURATIONS Debug)

install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusTest.pdb DESTINATION bind CONFIGURATIONS Debug)

install (FILES ${H_FILES}           DESTINATION include/mobius)
install (FILES ${reportgen_H_FILES} DESTINATION include/mobius)
