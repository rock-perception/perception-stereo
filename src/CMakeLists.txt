# include macro
include(FindPkgConfig)

rock_find_pkgconfig(OPENCV REQUIRED opencv)
if (${OPENCV_VERSION} VERSION_LESS 2.4)
    # The CV 2.3 pkg-config file does not include the opencv_gpu library. Look for
    # it manually
    find_library(OPENCV_GPU_LIB NAMES opencv_gpu HINTS ${OPENCV_LIBRARY_DIRS})
    if (NOT OPENCV_GPU_LIB)
        message(FATAL_ERROR "cannot find the opencv_gpu library in ${OPENCV_LIBRARY_DIRS}. This is required")
    endif()

    list(APPEND EXTRA_SOURCES sparse_stereo.cpp)
    list(APPEND EXTRA_HEADERS sparse_stereo.hpp)
else()
    set(OPENCV_GPU_LIB)
    # Check if the non-free stuff is there
    find_file(OPENCV_HAS_NONFREE
        opencv2/nonfree/features2d.hpp
        PATHS ${OPENCV_INCLUDE_DIRS})

    add_definitions(-DPSURF_NEEDS_LEGACY)
    add_definitions(-DOPENCV_HAS_GPUMAT_IN_CORE)
    if (OPENCV_HAS_NONFREE)
        add_definitions(-DOPENCV_HAS_NONFREE)
        list(APPEND EXTRA_SOURCES sparse_stereo.cpp)
        list(APPEND EXTRA_HEADERS sparse_stereo.hpp)
    endif()
endif()


# use sse3 instruction set
set(CMAKE_CXX_FLAGS "-msse3")

rock_library(stereo
    SOURCES densestereo.cpp homography.cpp dense_stereo_types.cpp configuration.cpp psurf.cpp ${EXTRA_SOURCES}
    DEPS_PKGCONFIG opencv frame_helper libelas envire
    HEADERS densestereo.h dense_stereo_types.h sparse_stereo_types.h homography.h store_vector.hpp psurf.h ${EXTRA_HEADERS})

target_link_libraries(stereo ${OPENCV_GPU_LIB})
