# include macro
include(FindPkgConfig)

# resolve dependency
pkg_check_modules(OPENCV REQUIRED "opencv")

# include directories
include_directories("${OPENCV_INCLUDE_DIRS}")

# opencv link directory
link_directories(${OPENCV_LIBRARY_DIRS})

# use sse3 instruction set
set(CMAKE_CXX_FLAGS "-msse3")

rock_library(stereo
    SOURCES configuration.cpp densestereo.cpp sparse_stereo.cpp
    DEPS_PKGCONFIG opencv frame_helper libelas envire
    HEADERS configuration.h densestereo.h dense_stereo_types.h sparse_stereo.hpp sparse_stereo_types.h)

add_subdirectory( ransac )
