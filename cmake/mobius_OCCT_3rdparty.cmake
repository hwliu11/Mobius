MOBIUS_THIRDPARTY_PRODUCT("tcl" "" "tcl.h" "tcl86")

if (3RDPARTY_tcl_DIR AND NOT 3RDPARTY_tk_DIR)
  if (EXISTS ${3RDPARTY_tcl_INCLUDE_DIR}/tk.h)
    set (3RDPARTY_tk_DIR "${3RDPARTY_tcl_DIR}" CACHE PATH "The directory containing tk" FORCE)
  endif()
endif()
MOBIUS_THIRDPARTY_PRODUCT("tk" "" "tk.h" "tk86")

MOBIUS_THIRDPARTY_PRODUCT("freeimage" "" "FreeImage.h" "FreeImage")
if (3RDPARTY_freeimage_DIR AND NOT 3RDPARTY_freeimageplus_DIR)
  set (3RDPARTY_freeimageplus_DIR "${3RDPARTY_freeimage_DIR}" CACHE PATH "The directory containing freeimageplus" FORCE)
endif()
MOBIUS_THIRDPARTY_PRODUCT("freeimageplus" "" "FreeImagePlus.h" "FreeImagePlus")

MOBIUS_THIRDPARTY_PRODUCT("freetype" "" "ft2build.h" "freetype")
MOBIUS_THIRDPARTY_PRODUCT("tbb" "tbb" "tbb.h" "tbb")
