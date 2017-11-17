project (mobiusCore)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  core.h
)

#------------------------------------------------------------------------------
# Base
#------------------------------------------------------------------------------

set (base_H_FILES
  base/core_OBJECT.h
  base/core_Precision.h
  base/core_Ptr.h
  base/core_Smoothness.h
)
set (base_CPP_FILES 
  base/core_OBJECT.cpp
  base/core_Precision.cpp
  base/core_Ptr.cpp
)

#------------------------------------------------------------------------------
# BLAS
#------------------------------------------------------------------------------

set (blas_H_FILES
  blas/core_SolveLinearSystem.h
)
set (blas_CPP_FILES 
  blas/core_SolveLinearSystem.cpp
)

#------------------------------------------------------------------------------
# Primitives
#------------------------------------------------------------------------------

set (primitives_H_FILES 
  primitives/core_IsoTransform.h
  primitives/core_IsoTransformChain.h
  primitives/core_Quaternion.h
  primitives/core_UV.h
  primitives/core_XYZ.h
)
set (primitives_CPP_FILES 
  primitives/core_IsoTransform.cpp
  primitives/core_IsoTransformChain.cpp
  primitives/core_Quaternion.cpp
  primitives/core_UV.cpp
  primitives/core_XYZ.cpp
)

#------------------------------------------------------------------------------
# Utilities
#------------------------------------------------------------------------------

set (utils_H_FILES 
  utils/core_FileDumper.h
  utils/core_HeapAlloc.h
  utils/core_Journal.h
  utils/core_JournalTicket.h
  utils/core_StringBuffer.h
)
set (utils_CPP_FILES 
  utils/core_FileDumper.cpp
  utils/core_HeapAlloc.cpp
  utils/core_Journal.cpp
  utils/core_JournalTicket.cpp
  utils/core_StringBuffer.cpp
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES core)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

#------------------------------------------------------------------------------

# Create include variable
set (core_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (core_include_dir ${core_include_dir_loc} PARENT_SCOPE)

foreach (FILE ${H_FILES})
  source_group ("Header Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${CPP_FILES})
  source_group ("Source Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${base_H_FILES})
  source_group ("Header Files\\Base" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${base_CPP_FILES})
  source_group ("Source Files\\Base" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${blas_H_FILES})
  source_group ("Header Files\\BLAS" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${blas_CPP_FILES})
  source_group ("Source Files\\BLAS" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${primitives_H_FILES})
  source_group ("Header Files\\Primitives" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${primitives_CPP_FILES})
  source_group ("Source Files\\Primitives" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${utils_H_FILES})
  source_group ("Header Files\\Utilities" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${utils_CPP_FILES})
  source_group ("Source Files\\Utilities" FILES "${FILE}")
endforeach (FILE)

message (STATUS "EIGEN dir (3RDPARTY_EIGEN_DIR): ${3RDPARTY_EIGEN_DIR}")

include_directories(${core_include_dir_loc} ${3RDPARTY_EIGEN_DIR})
add_library (mobiusCore SHARED
  ${H_FILES}            ${CPP_FILES} 
  ${base_H_FILES}       ${base_CPP_FILES}
  ${blas_H_FILES}       ${blas_CPP_FILES}
  ${primitives_H_FILES} ${primitives_CPP_FILES}
  ${utils_H_FILES}      ${utils_CPP_FILES})

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusCore
         RUNTIME DESTINATION bin COMPONENT Runtime
         CONFIGURATIONS Release
         ARCHIVE DESTINATION lib COMPONENT Library
         CONFIGURATIONS Release)

install (TARGETS mobiusCore
         RUNTIME DESTINATION bind COMPONENT Runtime
         CONFIGURATIONS Debug
         ARCHIVE DESTINATION libd COMPONENT Library
         CONFIGURATIONS Debug)

install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusCore.pdb DESTINATION bind CONFIGURATIONS Debug)

install (FILES ${H_FILES}            DESTINATION include/mobius)
install (FILES ${base_H_FILES}       DESTINATION include/mobius)
install (FILES ${blas_H_FILES}       DESTINATION include/mobius)
install (FILES ${primitives_H_FILES} DESTINATION include/mobius)
install (FILES ${utils_H_FILES}      DESTINATION include/mobius)
