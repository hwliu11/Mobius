cmake_minimum_required (VERSION 3.0.0 FATAL_ERROR)

# Project name
project (Mobius-OX CXX)

# Install dir
set (INSTALL_DIR "" CACHE PATH "Where to install" )
set (CMAKE_INSTALL_PREFIX "${INSTALL_DIR}" CACHE INTERNAL "" FORCE )

# Directories for embedded unit tests.
set (MOBIUS_TEST_DUMPING "${CMAKE_BINARY_DIR}" CACHE PATH "Dumping directory for tests.")
set (MOBIUS_TEST_DATA    "${CMAKE_SOURCE_DIR}/data" CACHE PATH "Directory which contains test data.")
set (MOBIUS_TEST_DESCR   "${CMAKE_SOURCE_DIR}/src/test" CACHE PATH "Directory which contains descriptions of tests.")

# Build configurations
set (CMAKE_CONFIGURATION_TYPES Release Debug RelWithDebInfo CACHE INTERNAL "" FORCE)

#-------------------------------------------------------------------------------
# Find 3rd-parties
#-------------------------------------------------------------------------------

include (${CMAKE_SOURCE_DIR}/cmake/mobius_3rdparty_macros.cmake)

MOBIUS_RETRIEVE_VERSION(${CMAKE_SOURCE_DIR}/src/core/core.h VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

# Directory containing all 3-rd parties
set (3RDPARTY_DIR "" CACHE PATH "The root directory for all 3-rd parties")

if (NOT DEFINED ${3RDPARTY_DIR} AND ${3RDPARTY_DIR} STREQUAL "")
  message (FATAL_ERROR "... Root 3rdparty directory was not found.\nPlease, set 3RDPARTY_DIR variable.")
endif()

# OpenCascade is optional
set (USE_OPENCASCADE OFF CACHE BOOL "OpenCascade is optional. If turned on, converters to OpenCascade are available.")

# ADOLC is optional
set (USE_ADOLC OFF CACHE BOOL "ADOL-C for algorithmic differentiation.")

# Whether to generate documentation
set (GENERATE_DOC OFF CACHE BOOL "Generate documentation from sources (with Doxygen).")

# Eigen
include (${CMAKE_SOURCE_DIR}/cmake/mobius_Eigen.cmake)

# OCCT
include (${CMAKE_SOURCE_DIR}/cmake/mobius_OCCT.cmake)

# Doxygen for documentation
include (${CMAKE_SOURCE_DIR}/cmake/mobius_Doxygen.cmake)

# Mathjax for documentation
include (${CMAKE_SOURCE_DIR}/cmake/mobius_Mathjax.cmake)

# Graphviz for documentation
include (${CMAKE_SOURCE_DIR}/cmake/mobius_Graphviz.cmake)

# ADOL-C for experimenting with algorithmic differentiation
include (${CMAKE_SOURCE_DIR}/cmake/mobius_adolc.cmake)

set (3RDPARTY_DLL_PATH "${3RDPARTY_freetype_DLL_DIR};${3RDPARTY_freeimage_DLL_DIR};")
set (3RDPARTY_DLL_DEB_PATH "${3RDPARTY_DLL_PATH}")

if (3RDPARTY_OCCT_DLL_DIR_DEBUG AND EXISTS "${3RDPARTY_OCCT_DLL_DIR_DEBUG}")
  set (3RDPARTY_DLL_DEB_PATH "${3RDPARTY_DLL_DEB_PATH};${3RDPARTY_OCCT_DLL_DIR_DEBUG}")
endif()

set (3RDPARTY_DLL_PATH "${3RDPARTY_DLL_PATH};${3RDPARTY_OCCT_DLL_DIR};")

#-------------------------------------------------------------------------------
string (REGEX REPLACE ";" " " 3RDPARTY_NOT_INCLUDED "${3RDPARTY_NOT_INCLUDED}")

# check all 3rdparty paths
if (3RDPARTY_NOT_INCLUDED)
  message (FATAL_ERROR "NOT FOUND: ${3RDPARTY_NOT_INCLUDED}")
endif()

#-------------------------------------------------------------------------------
# Compiler
#-------------------------------------------------------------------------------

# Configure warnings level
if (MSVC)
  add_definitions (/W4)
  add_definitions (-D_CRT_SECURE_NO_WARNINGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /EHa")
elseif (CMAKE_COMPILER_IS_GNUCXX)
  add_definitions (-Wall -pedantic -Wno-unknown-pragmas)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
else()
  message ("Unknown compiler")
endif()

# OpenMP support
find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#-------------------------------------------------------------------------------
# Output
#-------------------------------------------------------------------------------

# build directories
set (OS_WITH_BIT "${PLATFORM}${COMPILER_BITNESS}")

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/lib")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/lib")

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/libi")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/bini")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/libi")

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/libd")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/bind")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/libd")

if (WIN32)
  set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/bin")
  set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/bini")
  set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_BINARY_DIR}/${OS_WITH_BIT}/${COMPILER}/bind")
endif()

#-------------------------------------------------------------------------------
# Sub-projects
#-------------------------------------------------------------------------------

add_subdirectory(${CMAKE_SOURCE_DIR}/src/core)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/poly)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/bspl)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/geom)
#add_subdirectory(${CMAKE_SOURCE_DIR}/src/api)

if (USE_OPENCASCADE)
  add_subdirectory(${CMAKE_SOURCE_DIR}/src/cascade)
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/src/testEngine)

if (WIN32)
  add_subdirectory(${CMAKE_SOURCE_DIR}/src/test)
  add_subdirectory(${CMAKE_SOURCE_DIR}/src/visu)
  add_subdirectory(${CMAKE_SOURCE_DIR}/src/visuApp)
endif()

#-------------------------------------------------------------------------------
# Documentation
#-------------------------------------------------------------------------------

if (GENERATE_DOC AND NOT "${DOXYGEN_EXE}" STREQUAL "" AND EXISTS "${DOXYGEN_EXE}")
  message(STATUS "... Collect dependencies for docapi")
  file(GLOB_RECURSE dep_sources "*.h" "*.cpp")
  set (DOXYGEN_STATIC "${CMAKE_SOURCE_DIR}/doc/static")

  if (EXISTS "${MATHJAX_EXE}")
    set (DOXYGEN_USE_MATHJAX "YES")
    set (DOXYGEN_MATHJAX "${MATHJAX_DIR}")
  else()
    set (DOXYGEN_USE_MATHJAX "NO")
  endif()

  set (USE_GRAPHVIZ ON CACHE BOOL "Using of Graphviz for diagrams in documentation")
  if (EXISTS "${GRAPHVIZ_EXE}" AND USE_GRAPHVIZ)
    set (DOXYGEN_USE_GRAPHVIZ "YES")
  else()
    set (DOXYGEN_USE_GRAPHVIZ "NO")
  endif()

  add_subdirectory(${CMAKE_SOURCE_DIR}/doc/docapi)
  add_subdirectory(${CMAKE_SOURCE_DIR}/doc/docalgo)

  add_dependencies(docalgo docapi)
endif()

#-------------------------------------------------------------------------------
# Setup compiler
#-------------------------------------------------------------------------------

if (NOT "${INNOSETUP_EXE}" STREQUAL "" AND EXISTS "${INNOSETUP_EXE}")
  add_subdirectory(${CMAKE_SOURCE_DIR}/setup)
endif()

#-------------------------------------------------------------------------------
# Folders
#-------------------------------------------------------------------------------

set_property(TARGET mobiusCore    PROPERTY FOLDER Kernel)
set_property(TARGET mobiusPoly    PROPERTY FOLDER Kernel)
set_property(TARGET mobiusBSpl    PROPERTY FOLDER Kernel)
set_property(TARGET mobiusGeom    PROPERTY FOLDER Kernel)

if (USE_OPENCASCADE)
  set_property(TARGET mobiusCascade PROPERTY FOLDER Services)
endif()

set_property(TARGET mobiusTestEngine PROPERTY FOLDER Tests)

if (WIN32)
  set_property(TARGET mobiusTest    PROPERTY FOLDER Tests)
  set_property(TARGET mobiusVisu    PROPERTY FOLDER Visualization)
  set_property(TARGET mobiusVisuApp PROPERTY FOLDER Visualization)
endif()

if (GENERATE_DOC AND NOT "${DOXYGEN_EXE}" STREQUAL "" AND EXISTS "${DOXYGEN_EXE}")
  # Path to the logo.
  set (DOXYGEN_LOGO "${CMAKE_SOURCE_DIR}/doc/imgs/doxy_logo.png")

  # Version of the library.
  string(TIMESTAMP VERSION_WITH_TIMESTAMP "%d%m%y")
  set (VERSION_WITH_TIMESTAMP "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}.${VERSION_WITH_TIMESTAMP}")

  set_property(TARGET docalgo PROPERTY FOLDER Documentation)
  set_property(TARGET docapi  PROPERTY FOLDER Documentation)
endif()

set_property (GLOBAL PROPERTY USE_FOLDERS ON)

#-------------------------------------------------------------------------------

message (STATUS "MOBIUS_MODULES: ${MOBIUS_MODULES}")

string (TIMESTAMP CURRENT_TIME "%H:%M:%S")
message (STATUS "\nInfo: \(${CURRENT_TIME}\) Start collecting all header files into ${CMAKE_BINARY_DIR}/inc ...")

# collect all headers to <binary dir>/inc folder
COLLECT_AND_INSTALL_MOBIUS_HEADER_FILES ("${CMAKE_BINARY_DIR}" "${MOBIUS_MODULES}")

string (TIMESTAMP CURRENT_TIME "%H:%M:%S")
message (STATUS "Info: \(${CURRENT_TIME}\) End collecting")
