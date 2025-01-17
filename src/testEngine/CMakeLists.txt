project (mobiusTestEngine)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  outcome.h
  testEngine.h
  testEngine_Launcher.h
  testEngine_Macro.h
  testEngine_TestCase.h
)

set (CPP_FILES
  testEngine_Launcher.cpp
  testEngine_TestCase.cpp
)

#------------------------------------------------------------------------------
# Report generator
#------------------------------------------------------------------------------

set (reportgen_H_FILES
  reportgen/testEngine_DescriptionProc.h
  reportgen/testEngine_ReportRenderer.h
  reportgen/testEngine_ReportStyle.h
  reportgen/testEngine_ReportStyleFactory.h
  reportgen/testEngine_ReportTag.h
  reportgen/testEngine_ReportTagFactory.h
)

set (reportgen_CPP_FILES
  reportgen/testEngine_DescriptionProc.cpp
  reportgen/testEngine_ReportRenderer.cpp
  reportgen/testEngine_ReportStyle.cpp
  reportgen/testEngine_ReportStyleFactory.cpp
  reportgen/testEngine_ReportTag.cpp
  reportgen/testEngine_ReportTagFactory.cpp
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES testEngine)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

#------------------------------------------------------------------------------

# Create include variable
set (testEngine_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (testEngine_include_dir ${testEngine_include_dir_loc} PARENT_SCOPE)

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

include_directories(${testEngine_include_dir_loc} ${core_include_dir})

if (USE_ADOLC)
  include_directories(${3RDPARTY_adolc_INCLUDE_DIR})
endif()

add_library (mobiusTestEngine SHARED
  ${H_FILES}           ${CPP_FILES}
  ${reportgen_H_FILES} ${reportgen_CPP_FILES})

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusTestEngine mobiusCore)

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusTestEngine
         CONFIGURATIONS Release
         RUNTIME DESTINATION bin COMPONENT Runtime
         ARCHIVE DESTINATION lib COMPONENT Library
         LIBRARY DESTINATION lib COMPONENT Library)

install (TARGETS mobiusTestEngine
         CONFIGURATIONS RelWithDebInfo
         RUNTIME DESTINATION bini COMPONENT Runtime
         ARCHIVE DESTINATION libi COMPONENT Library
         LIBRARY DESTINATION libi COMPONENT Library)

install (TARGETS mobiusTestEngine
         CONFIGURATIONS Debug
         RUNTIME DESTINATION bind COMPONENT Runtime
         ARCHIVE DESTINATION libd COMPONENT Library
         LIBRARY DESTINATION libd COMPONENT Library)

if (MSVC)
  install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusTestEngine.pdb DESTINATION bind CONFIGURATIONS Debug)
endif()

install (FILES ${H_FILES}           DESTINATION include/mobius)
install (FILES ${reportgen_H_FILES} DESTINATION include/mobius)
