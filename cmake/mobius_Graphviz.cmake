if (NOT GENERATE_DOC)
  MOBIUS_UNSET(GRAPHVIZ_EXE)
  return()
endif()

message (STATUS "Processing Graphviz 3-rd party")

MOBIUS_FIND_PRODUCT_DIR ("${3RDPARTY_DIR}" "graphviz" GRAPHVIZ_DIR)

set (GRAPHVIZ_DIR "${3RDPARTY_DIR}/${GRAPHVIZ_DIR}")
message (STATUS "... Graphviz dir: ${GRAPHVIZ_DIR}")
set (GRAPHVIZ_EXE "${GRAPHVIZ_DIR}/bin/dot${CMAKE_EXECUTABLE_SUFFIX}")
if (EXISTS "${GRAPHVIZ_EXE}")
  message (STATUS "... Graphviz executable found: ${GRAPHVIZ_EXE}")
  set (GRAPHVIZ_EXE "${GRAPHVIZ_EXE}" CACHE PATH "Graphviz - pretty graphs support for API documentation" FORCE)
else()
  set (GRAPHVIZ_EXE "" CACHE PATH "Graphviz - pretty graphs support for API documentation" FORCE)
endif()


