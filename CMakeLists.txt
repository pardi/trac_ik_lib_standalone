cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()


find_package(Boost REQUIRED COMPONENTS date_time)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)
pkg_check_modules(Eigen REQUIRED eigen3)


include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
)

add_library(trac_ik
  src/kdl_tl.cpp
  src/nlopt_ik.cpp
  src/trac_ik.cpp)

target_link_libraries(trac_ik
  ${pkg_nlopt_LIBRARIES}
  ${Boost_LIBRARIES})

