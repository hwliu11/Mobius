# Useful macro

#-------------------------------------------------------------------------------
# Name:    MOBIUS_MAKE_PLATFORM_SHORT_NAME
# Purpose: initializes PLATFORM variable with a relevant value
#-------------------------------------------------------------------------------
macro (MOBIUS_MAKE_PLATFORM_SHORT_NAME)
  if (MSVC)
    set (PLATFORM win)
  else()
    set (PLATFORM lin)
  endif()
endmacro()

#-------------------------------------------------------------------------------
# Name:    MOBIUS_MAKE_COMPILER_SHORT_NAME
# Purpose: initializes COMPILER variable with a relevant value
#-------------------------------------------------------------------------------
macro (MOBIUS_MAKE_COMPILER_SHORT_NAME)
  if (MSVC)
    if (MSVC70)
      set (COMPILER vc7)
    elseif (MSVC80)
      set (COMPILER vc8)
    elseif (MSVC90)
      set (COMPILER vc9)
    elseif (MSVC10)
      set (COMPILER vc10)
    elseif (MSVC11)
      set (COMPILER vc11)
    elseif (MSVC12)
      set (COMPILER vc12)
    elseif (MSVC14)
      set (COMPILER vc14)
    endif()
  elseif (DEFINED CMAKE_COMPILER_IS_GNUCC)
    set (COMPILER gcc)
  elseif (DEFINED CMAKE_COMPILER_IS_GNUCXX)
    set (COMPILER gxx)
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set (COMPILER clang)
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    set (COMPILER icc)
  else()
    set (COMPILER ${CMAKE_GENERATOR})
    string (REGEX REPLACE " " "" COMPILER ${COMPILER})
  endif()
endmacro()

#-------------------------------------------------------------------------------
# Name:    MOBIUS_MAKE_COMPILER_BITNESS
# Purpose: initializes COMPILER_BITNESS variable with a relevant value
#-------------------------------------------------------------------------------
macro (MOBIUS_MAKE_COMPILER_BITNESS)
  math (EXPR COMPILER_BITNESS "32 + 32*(${CMAKE_SIZEOF_VOID_P}/8)")
endmacro()

# Name:    COLLECT_AND_INSTALL_MOBIUS_HEADER_FILES
# Purpose: copies the links to include files to binary dir
#-------------------------------------------------------------------------------
macro (COLLECT_AND_INSTALL_MOBIUS_HEADER_FILES ROOT_TARGET_MOBIUS_DIR MOBIUS_MODULES)

  set (ROOT_MOBIUS_DIR ${CMAKE_SOURCE_DIR})
  set (MOBIUS_HEADER_FILES)

  # Set template for a link header from header.in
  set (TEMPLATE_HEADER_PATH "${CMAKE_SOURCE_DIR}/cmake/templates/header.in")

  # For each module, find recursively all header files with their full filenames.
  # These full filename will be used to create header links.
  foreach (MOBIUS_MODULE ${MOBIUS_MODULES})
    string(TIMESTAMP CURRENT_TIME "%H:%M:%S")
    message (STATUS "\(${CURRENT_TIME}\) Next module: ${MOBIUS_MODULE}")

    file (GLOB_RECURSE ALL_FILES "${CMAKE_SOURCE_DIR}/src/${MOBIUS_MODULE}/*.h")
    foreach (ONE_FILE ${ALL_FILES})
      message (STATUS "... Next file: ${ONE_FILE}")
      list (APPEND MOBIUS_HEADER_FILES ${ONE_FILE})
    endforeach()

    # Do the same for hxx (if I knew how to define one pattern, this code duplication would not be here :-(
    file (GLOB_RECURSE ALL_FILES "${CMAKE_SOURCE_DIR}/src/${MOBIUS_MODULE}/*.hxx")
    foreach (ONE_FILE ${ALL_FILES})
      message (STATUS "... Next file: ${ONE_FILE}")
      list (APPEND MOBIUS_HEADER_FILES ${ONE_FILE})
    endforeach()
  endforeach()

  # Create header links
  string(TIMESTAMP CURRENT_TIME "%H:%M:%S")
  message (STATUS "Info: \(${CURRENT_TIME}\) Create header-links in inc folder...")
  #
  foreach (MOBIUS_HEADER_FILE ${MOBIUS_HEADER_FILES})
    get_filename_component (HEADER_FILE_NAME ${MOBIUS_HEADER_FILE} NAME)
    set (MOBIUS_HEADER_FILE_CONTENT "#include \"${MOBIUS_HEADER_FILE}\"")
    configure_file ("${TEMPLATE_HEADER_PATH}" "${ROOT_TARGET_MOBIUS_DIR}/inc/mobius/${HEADER_FILE_NAME}" @ONLY)
  endforeach()

endmacro()

macro (CADPRO_UNSET VARNAME)
  if (DEFINED ${VARNAME})
    unset (${VARNAME} CACHE)
  endif()
endmacro()
