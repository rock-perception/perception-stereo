# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
include(Rock)
rock_init(dense_stereo 0.1)

pkg_check_modules(base_TYPES REQUIRED "base-types")
INCLUDE_DIRECTORIES(${base_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${base_TYPES_LIBRARY_DIRS})

rock_standard_layout()


