project (mobiusBSpl)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  bspl.h
  bspl_excMaxDegreeViolation.h
)
set (CPP_FILES
  bspl.cpp
)

#------------------------------------------------------------------------------
# Basis
#------------------------------------------------------------------------------

set (basis_H_FILES
  basis/bspl_Decompose.h
  basis/bspl_EffectiveN.h
  basis/bspl_EffectiveNDers.h
  basis/bspl_FindSpan.h
  basis/bspl_InsKnot.h
  basis/bspl_KnotMultiset.h
  basis/bspl_MultResolver.h
  basis/bspl_N.h
  basis/bspl_NDiscrete.h
  basis/bspl_ParamDirection.h
  basis/bspl_RefineKnots.h
  basis/bspl_UnifyKnots.h
)
set (basis_CPP_FILES 
  basis/bspl_Decompose.cpp
  basis/bspl_EffectiveN.cpp
  basis/bspl_EffectiveNDers.cpp
  basis/bspl_FindSpan.cpp
  basis/bspl_InsKnot.cpp
  basis/bspl_KnotMultiset.cpp
  basis/bspl_MultResolver.cpp
  basis/bspl_N.cpp
  basis/bspl_NDiscrete.cpp
  basis/bspl_RefineKnots.cpp
  basis/bspl_UnifyKnots.cpp
)

#------------------------------------------------------------------------------
# Laws
#------------------------------------------------------------------------------

set (laws_H_FILES
  laws/bspl_ConstLaw.h
  laws/bspl_HermiteLaw.h
  laws/bspl_ScalarLaw.h
)
set (laws_CPP_FILES 
  laws/bspl_ConstLaw.cpp
  laws/bspl_HermiteLaw.cpp
)

#------------------------------------------------------------------------------
# Parameterization
#------------------------------------------------------------------------------

set (parameterization_H_FILES 
  parameterization/bspl_KnotsSelection.h
  parameterization/bspl_KnotsAverage.h
  parameterization/bspl_KnotsUniform.h
  parameterization/bspl_ParamsSelection.h
  parameterization/bspl_ParamsCentripetal.h
  parameterization/bspl_ParamsChordLength.h
  parameterization/bspl_ParamsUniform.h
)
set (parameterization_CPP_FILES 
  parameterization/bspl_KnotsAverage.cpp
  parameterization/bspl_ParamsCentripetal.cpp
  parameterization/bspl_ParamsChordLength.cpp
  parameterization/bspl_ParamsUniform.cpp
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES bspl)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

#------------------------------------------------------------------------------

# Create include variable
set (bspl_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (bspl_include_dir ${bspl_include_dir_loc} PARENT_SCOPE)

foreach (FILE ${H_FILES})
  source_group ("Header Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${CPP_FILES})
  source_group ("Source Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${basis_H_FILES})
  source_group ("Header Files\\Basis" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${basis_CPP_FILES})
  source_group ("Source Files\\Basis" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${laws_H_FILES})
  source_group ("Header Files\\Laws" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${laws_CPP_FILES})
  source_group ("Source Files\\Laws" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${parameterization_H_FILES})
  source_group ("Header Files\\Parameterization" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${parameterization_CPP_FILES})
  source_group ("Source Files\\Parameterization" FILES "${FILE}")
endforeach (FILE)

include_directories(${bspl_include_dir_loc} ${core_include_dir})

if (USE_ADOLC)
  include_directories(${3RDPARTY_adolc_INCLUDE_DIR})
endif()

add_library (mobiusBSpl SHARED
  ${H_FILES}                  ${CPP_FILES} 
  ${basis_H_FILES}            ${basis_CPP_FILES}
  ${laws_H_FILES}             ${laws_CPP_FILES}
  ${parameterization_H_FILES} ${parameterization_CPP_FILES})

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusBSpl mobiusCore)

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusBSpl
         CONFIGURATIONS Release
         RUNTIME DESTINATION bin COMPONENT Runtime
         ARCHIVE DESTINATION lib COMPONENT Library
         LIBRARY DESTINATION lib COMPONENT Library)

install (TARGETS mobiusBSpl
         CONFIGURATIONS RelWithDebInfo
         RUNTIME DESTINATION bini COMPONENT Runtime
         ARCHIVE DESTINATION libi COMPONENT Library
         LIBRARY DESTINATION libi COMPONENT Library)

install (TARGETS mobiusBSpl
         CONFIGURATIONS Debug
         RUNTIME DESTINATION bind COMPONENT Runtime
         ARCHIVE DESTINATION libd COMPONENT Library
         LIBRARY DESTINATION libd COMPONENT Library)

install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusBSpl.pdb DESTINATION bind CONFIGURATIONS Debug)
install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bini/mobiusBSpl.pdb DESTINATION bini CONFIGURATIONS RelWithDebInfo)

install (FILES ${H_FILES}                  DESTINATION include/mobius)
install (FILES ${basis_H_FILES}            DESTINATION include/mobius)
install (FILES ${laws_H_FILES}             DESTINATION include/mobius)
install (FILES ${parameterization_H_FILES} DESTINATION include/mobius)
