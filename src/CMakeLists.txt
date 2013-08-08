# include macro
include(FindPkgConfig)

# The CV 2.3 pkg-config file does not include the opencv_gpu library. Look for
# it manually
rock_find_pkgconfig(OPENCV REQUIRED opencv)
find_library(OPENCV_GPU_LIB NAMES opencv_gpu HINTS ${OPENCV_LIBRARY_DIRS})

if (NOT OPENCV_GPU_LIB)
    message(FATAL_ERROR "cannot find the opencv_gpu library in ${OPENCV_LIBRARY_DIRS}. This is required")
endif()

# use sse3 instruction set
set(CMAKE_CXX_FLAGS "-msse3")

rock_library(stereo
    SOURCES densestereo.cpp sparse_stereo.cpp psurf.cpp homography.cpp dense_stereo_types.cpp configuration.cpp
    DEPS_PKGCONFIG opencv frame_helper libelas envire
    HEADERS densestereo.h dense_stereo_types.h sparse_stereo.hpp sparse_stereo_types.h homography.h store_vector.hpp)

target_link_libraries(stereo ${OPENCV_GPU_LIB})
