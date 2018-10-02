cmake_minimum_required(VERSION 2.8.3)

project(manifoldReconstructor)

#----------------------- CMAKE MODULES ------------------------

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)

# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------- INCLUDES ----------------------------
include_directories(  ${EIGEN3_INCLUDE_DIR}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS}
                      ${CGAL_INCLUDE_DIRS}
                      external/glm/glm/
                      external/
                      include/core/
                      include/core/utilities/
                      include/example/
                      )
# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ MESSAGES ----------------------------

# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE LIB_SOURCE src/core/*.cpp)
file(GLOB_RECURSE MAIN_SOURCE src/example/OpenMvgParser.cpp
                            src/example/ORBIncrementalParser.cpp
                            src/example/ORBParser.cpp
                            src/example/points_filtering.cpp
)

add_executable(sfmReconstructor src/example/Main_sfm.cpp ${MAIN_SOURCE} ${LIB_SOURCE})
target_link_libraries(sfmReconstructor ${OpenCV_LIBS}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}  )



# ------------------------ INSTALLATION ------------------------


# --------------------------------------------------------------


