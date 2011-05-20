# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
include(Rock)
rock_init(dense_stereo 0.1)

rock_find_pkgconfig(base_TYPES REQUIRED "base-types")
rock_export_includedir(${PROJECT_SOURCE_DIR}/src dense_stereo)
rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit)

rock_standard_layout()


