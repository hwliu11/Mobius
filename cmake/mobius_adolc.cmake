if (NOT USE_ADOLC)
  MOBIUS_UNSET_3RDPARTY("adolc")
  return()
endif()

MOBIUS_THIRDPARTY_PRODUCT("adolc" "" "adolc.h" "adolc")

add_definitions (-DUSE_ADOLC)

# DLL is useless
MOBIUS_UNSET ("3RDPARTY_adolc_DLL_DIR")

if (3RDPARTY_adolc_INCLUDE_DIR STREQUAL "")
  message (STATUS "... adolc Include dir is not conventional")
  list (REMOVE_ITEM 3RDPARTY_NOT_INCLUDED 3RDPARTY_adolc_INCLUDE_DIR)
  set (3RDPARTY_adolc_INCLUDE_DIR ${3RDPARTY_adolc_DIR}/include CACHE FILEPATH "Non-conventional inc dir" FORCE)
endif()

message (STATUS "... adolc Include dirs: ${3RDPARTY_adolc_INCLUDE_DIR}")
message (STATUS "... adolc Library dirs: ${3RDPARTY_adolc_LIBRARY_DIR}")

add_definitions (-DDTK_SHARED)

string (REPLACE lib libd 3RDPARTY_adolc_LIBRARY_DIR_DEBUG ${3RDPARTY_adolc_LIBRARY_DIR})
if (3RDPARTY_adolc_LIBRARY_DIR_DEBUG AND EXISTS "${3RDPARTY_adolc_LIBRARY_DIR_DEBUG}")
  if (WIN32)
    if (NOT EXISTS "${3RDPARTY_adolc_LIBRARY_DIR_DEBUG}/adolc.lib")
      set (3RDPARTY_adolc_LIBRARY_DIR_DEBUG "" CACHE INTERNAL FORCE)
    endif()
  else()
    if (NOT EXISTS "${3RDPARTY_adolc_LIBRARY_DIR_DEBUG}/adolc.so")
      set (3RDPARTY_adolc_LIBRARY_DIR_DEBUG "" CACHE INTERNAL FORCE)
    endif()
  endif()
endif()

message (STATUS "... adolc Debug Library dirs: ${3RDPARTY_adolc_LIBRARY_DIR_DEBUG}")
