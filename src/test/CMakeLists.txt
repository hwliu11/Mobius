project (mobiusTest)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  test.h
  test_CaseIDs.h
  test_CommonFacilities.h
  test_ProgressNotifier.h
)

set (CPP_FILES
  test_CommonFacilities.cpp
  test_Main.cpp
  test_ProgressNotifier.cpp
)

#------------------------------------------------------------------------------
# Basic splines
#------------------------------------------------------------------------------

set (bspl_H_FILES
  bspl/test_Decompose.h
  bspl/test_EffectiveN.h
  bspl/test_EffectiveNDers.h
  bspl/test_FindSpan.h
  bspl/test_InsKnot.h
  bspl/test_KnotMultiset.h
  bspl/test_KnotsAverage.h
  bspl/test_KnotsUniform.h
  bspl/test_N.h
  bspl/test_ParamsCentripetal.h
  bspl/test_ParamsChordLength.h
  bspl/test_ParamsUniform.h
  bspl/test_UnifyKnots.h
)

set (bspl_CPP_FILES
  bspl/test_Decompose.cpp
  bspl/test_EffectiveN.cpp
  bspl/test_EffectiveNDers.cpp
  bspl/test_FindSpan.cpp
  bspl/test_InsKnot.cpp
  bspl/test_KnotMultiset.cpp
  bspl/test_KnotsAverage.cpp
  bspl/test_KnotsUniform.cpp
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
  core/test_Integral.h
  core/test_Quaternion.h
  core/test_XYZ.h
)

set (core_CPP_FILES
  core/test_Integral.cpp
  core/test_Quaternion.cpp
  core/test_XYZ.cpp
)

#------------------------------------------------------------------------------
# Geom
#------------------------------------------------------------------------------

set (geom_H_FILES
  geom/test_ApproxSurf.h
  geom/test_BSplineCurve.h
  geom/test_BSplineSurface.h
  geom/test_FairCurve.h
  geom/test_FairSurf.h
  geom/test_InterpolateCurve.h
  geom/test_Line3D.h
  geom/test_MakeBicubicBSurf.h
  geom/test_PlaneSurface.h
  geom/test_PointOnLine.h
  geom/test_PositionCloud.h
  geom/test_SkinSurface.h
)

set (geom_CPP_FILES
  geom/test_ApproxSurf.cpp
  geom/test_BSplineCurve.cpp
  geom/test_BSplineSurface.cpp
  geom/test_FairCurve.cpp
  geom/test_FairSurf.cpp
  geom/test_InterpolateCurve.cpp
  geom/test_Line3D.cpp
  geom/test_MakeBicubicBSurf.cpp
  geom/test_PlaneSurface.cpp
  geom/test_PointOnLine.cpp
  geom/test_PositionCloud.cpp
  geom/test_SkinSurface.cpp
)

#------------------------------------------------------------------------------
# Poly
#------------------------------------------------------------------------------

set (poly_H_FILES
  poly/test_Mesh.h
  poly/test_SVO.h
)

set (poly_CPP_FILES
  poly/test_Mesh.cpp
  poly/test_SVO.cpp
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES test)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

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

foreach (FILE ${bspl_H_FILES})
  source_group ("Header Files\\BSpl" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${bspl_CPP_FILES})
  source_group ("Source Files\\BSpl" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${core_H_FILES})
  source_group ("Header Files\\Core" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${core_CPP_FILES})
  source_group ("Source Files\\Core" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${geom_H_FILES})
  source_group ("Header Files\\Geom" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${geom_CPP_FILES})
  source_group ("Source Files\\Geom" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${poly_H_FILES})
  source_group ("Header Files\\Poly" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${poly_CPP_FILES})
  source_group ("Source Files\\Poly" FILES "${FILE}")
endforeach (FILE)

include_directories(${test_include_dir_loc} ${core_include_dir} ${bspl_include_dir} ${geom_include_dir} ${poly_include_dir} ${testEngine_include_dir})

if (USE_ADOLC)
  include_directories(${3RDPARTY_adolc_INCLUDE_DIR})
endif()

add_executable (mobiusTest
  ${H_FILES}      ${CPP_FILES}
  ${bspl_H_FILES} ${bspl_CPP_FILES}
  ${core_H_FILES} ${core_CPP_FILES}
  ${geom_H_FILES} ${geom_CPP_FILES}
  ${poly_H_FILES} ${poly_CPP_FILES}
)

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

configure_file(${CMAKE_SOURCE_DIR}/cmake/templates/exePROTOTYPE.vcxproj.user.in
               ${mobiusTest_BINARY_DIR}/mobiusTest.vcxproj.user @ONLY)

configure_file(${CMAKE_SOURCE_DIR}/cmake/templates/setenv.bat
               ${mobiusTest_BINARY_DIR}/setenv.bat @ONLY)

# Post-build event: run tests.
add_custom_command(
        TARGET mobiusTest
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Running unit tests."
        COMMAND call ${mobiusTest_BINARY_DIR}/setenv.bat && $<TARGET_FILE:mobiusTest>
    )

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusTest mobiusCore mobiusPoly mobiusBSpl mobiusGeom mobiusTestEngine)
