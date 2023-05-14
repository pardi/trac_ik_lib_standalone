cmake_minimum_required(VERSION 3.10)
project(example)

# if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
#     set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# endif()
set(CMAKE_CXX_FLAGS "-std=c++20 ${CMAKE_CXX_FLAGS}")

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel.")

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)
pkg_check_modules(Eigen REQUIRED eigen3)


add_executable(${PROJECT_NAME} MACOSX_BUNDLE panda_robot.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
)

# Target link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE trac_ik urdfdom_model orocos-kdl kdl_parser
${pkg_nlopt_LIBRARIES} ${Boost_LIBRARIES})



# find_package(Boost REQUIRED COMPONENTS date_time)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(pkg_nlopt REQUIRED nlopt)
# pkg_check_modules(Eigen REQUIRED eigen3)

# add_library(${LIB_NAME} STATIC
#   src/kdl_tl.cpp
#   src/nlopt_ik.cpp
#   src/trac_ik.cpp)

# target_include_directories(${LIB_NAME} PUBLIC
#   include
#   ${Eigen_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
#   ${pkg_nlopt_INCLUDE_DIRS}
# )

# target_link_libraries(${LIB_NAME} PRIVATE
#   ${pkg_nlopt_LIBRARIES}
#   ${Boost_LIBRARIES})

#   # Install target
# install(TARGETS ${LIB_NAME}
# LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
# ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

# install(DIRECTORY include/ DESTINATION include)