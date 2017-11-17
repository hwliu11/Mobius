message (STATUS "Processing Eigen 3-rd party")

MOBIUS_FIND_PRODUCT_DIR ("${3RDPARTY_DIR}" "eigen" EIGEN_DIR)

set (3RDPARTY_EIGEN_DIR "${3RDPARTY_DIR}/${EIGEN_DIR}" CACHE PATH "The directory containing Eigen")

message (STATUS "... Eigen dir: ${3RDPARTY_EIGEN_DIR}")
