cmake_minimum_required(VERSION 3.1.0)
project(artslam_core)

# set the default path for built executables to the "bin" directory of the project
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set the default path for built libraries to the "lib" directory of the project
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# set the default compilation flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -DBOOST_LOG_DYN_LINK")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# set the cmake build type
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS program_options log log_setup)
include_directories(${Boost_INCLUDE_DIR})
message("${PROJECT_NAME} - Boost version ${Boost_VERSION}")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("${PROJECT_NAME} - OpenCV version ${OpenCV_VERSION}")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
message("${PROJECT_NAME} - PCL version ${PCL_VERSION}")

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
        INCLUDE_DIRS src
        LIBRARIES
        artslam_core_io_library
        artslam_core_types_library
        artslam_core_utils_library
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/src
)

link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(${PROJECT_SOURCE_DIR}/src)