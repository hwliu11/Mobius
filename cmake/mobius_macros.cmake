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

macro (MOBIUS_UNSET VARNAME)
  if (DEFINED ${VARNAME})
    unset (${VARNAME} CACHE)
  endif()
endmacro()

#-------------------------------------------------------------------------------
# Name:    MOBIUS_CONFIGURE_WARNING_LEVEL
# Purpose: Configure warnings level
#-------------------------------------------------------------------------------
macro (MOBIUS_CONFIGURE_WARNING_LEVEL)
  if (MSVC)
    # define all warnings print.
    add_definitions (/Wall)

    # Specify floating-point behavior.
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")

    # enumerator '' in switch of enum '' is not explicitly handled by a case label
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4061\"")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4062\"")
    # 'type cast': unsafe conversion from 'PROC' to 'PFNWGLCHOOSEPIXELFORMATARBPROC'
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4191\"")
    # 'argument': conversion from 'const int' to 'const unsigned __int64', signed/unsigned mismatch
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4365\"")
    # unreferenced inline function has been removed
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4514\"")
    # constructor is not implicitly called
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4582\"")
    # destructor is not implicitly called
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4583\"")
    # copy constructor was implicitly defined as deleted
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4625\"")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4626\"")
    # '_WIN32_WINNT_WIN10_RS5' is not defined as a preprocessor macro, replacing with '0' for '#if/#elif'
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4668\"")
    # '' function not inlined
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4710\"")
    # function '' selected for automatic inline expansion
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4711\"")
    # '4' bytes padding added after data member
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4820\"")
    # 'getenv':      This function or variable may be unsafe. Consider using _dupenv_s instead.
    # '_vsprintf_l': This function or variable may be unsafe. Consider using _vsprintf_s_l instead.
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"4996\"")


    # move constructor was implicitly defined as deleted
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"5026\"")
    # move assignment operator was implicitly defined as deleted
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"5027\"")
    # 'CreateThread': pointer or reference to potentially throwing function passed to 'extern "C"' function under -EHc.
    # Undefined behavior may occur if this function throws an exception.
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"5039\"")
    # Compiler will insert Spectre mitigation for memory load if /Qspectre switch specified
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"5045\"")
    # a non-static data member with a volatile qualified type no longer implies
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"5220\"")
    # '`anonymous-namespace'::__glDisableLighting': unreferenced function with internal linkage has been removed
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"5245\"")


    # Arithmetic overflow: Using operator '+' on a 4 byte value and then casting the result to a 8 byte value. Cast the value to the wider type before calling operator '+' to avoid overflow
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"26451\"")
    # The enum type '' is unscoped. Prefer 'enum class' over 'enum' (Enum.3)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd\"26812\"")

  elseif (CMAKE_COMPILER_IS_GNUCXX)
    add_definitions (-Wall -Wno-unknown-pragmas)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
    add_definitions(-DUSE_GCC)
  else()
    message ("Unknown compiler")
  endif()
endmacro()
