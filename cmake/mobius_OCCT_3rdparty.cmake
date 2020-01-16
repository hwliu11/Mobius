if (NOT USE_OPENCASCADE)
  MOBIUS_UNSET_3RDPARTY("freeimage")
  MOBIUS_UNSET_3RDPARTY("freeimageplus")
  MOBIUS_UNSET_3RDPARTY("freetype")
  MOBIUS_UNSET_3RDPARTY("tbb")
  return()
endif()

MOBIUS_THIRDPARTY_PRODUCT("freeimage" "" "FreeImage.h" "FreeImage")
if (3RDPARTY_freeimage_DIR AND NOT 3RDPARTY_freeimageplus_DIR)
  set (3RDPARTY_freeimageplus_DIR "${3RDPARTY_freeimage_DIR}" CACHE PATH "The directory containing freeimageplus" FORCE)
endif()
MOBIUS_THIRDPARTY_PRODUCT("freeimageplus" "" "FreeImagePlus.h" "FreeImagePlus")

MOBIUS_THIRDPARTY_PRODUCT("freetype" "" "ft2build.h" "freetype")
MOBIUS_THIRDPARTY_PRODUCT("tbb" "tbb" "tbb.h" "tbb")
