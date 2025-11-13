cmake_minimum_required(VERSION 3.14)
project(bytetrack)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -MMD -Wall -Wextra -Winit-self")

# Check C++20 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
if(COMPILER_SUPPORTS_CXX20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
  add_definitions(-DCOMPILEDWITHC20)
  message(STATUS "Using flag -std=c++20.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++20 support. Please use a different C++ compiler.")
endif()

set(EIGEN_VERSION 3.4)
find_package(Eigen3 ${EIGEN_VERSION} QUIET)
if(NOT EIGEN3_FOUND)
  set(BUILD_TESTING OFF CACHE INTERNAL "")
  include(FetchContent)
  FetchContent_Declare(eigen
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         ${EIGEN_VERSION}
    GIT_SHALLOW     ON)
  FetchContent_MakeAvailable(eigen)
  unset(BUILD_TESTING CACHE)
endif()

add_library(${PROJECT_NAME} STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Detection.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/KalmanFilter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lapjv.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Rect.cpp
  )
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Build the tests if the 'BUILD_BYTETRACK_TEST' variable is set to 'ON'
set(BUILD_BYTETRACK_TEST OFF CACHE BOOL "The flag whether to build the tests or not")
if(BUILD_BYTETRACK_TEST)
  enable_testing()

  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      GIT_REPOSITORY  https://github.com/google/googletest.git
      GIT_TAG         v1.12.0
      GIT_SHALLOW     ON)
    FetchContent_MakeAvailable(googletest)
  endif()

  find_package(Boost REQUIRED)

  add_executable(${PROJECT_NAME}_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_BYTETracker.cpp
  )

  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
    Boost::boost
    GTest::gtest_main
    )

  gtest_discover_tests(${PROJECT_NAME}_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/YOLOX_ncnn_palace)
endif()
