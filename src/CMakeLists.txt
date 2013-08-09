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
else()
    # Check if the non-free stuff is there
    find_file(OPENCV_HAS_NONFREE
        opencv2/nonfree/features2d.hpp
        PATHS ${OPENCV_INCLUDE_DIRS})
    # .. and the gpu
    find_file(OPENCV_HAS_GPU
        opencv2/gpu/gpu.hpp
        PATHS ${OPENCV_INCLUDE_DIRS})

    set(OPENCV_HAS_GPUMAT_IN_CORE TRUE)
    set(PSURF_NEEDS_LEGACY TRUE)
    if (OPENCV_HAS_NONFREE)
        add_definitions(-DOPENCV_HAS_SURF -DOPENCV_HAS_SIFT -DOPENCV_HAS_NONFREE)
    endif()
    if (OPENCV_HAS_GPU)
        add_definitions(-DOPENCV_HAS_SURF_GPU)
    endif()
endif()

configure_file(config.h.in stereo/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# use sse3 instruction set
set(CMAKE_CXX_FLAGS "-msse3")

rock_library(stereo
    SOURCES densestereo.cpp homography.cpp dense_stereo_types.cpp configuration.cpp psurf.cpp sparse_stereo.cpp
    DEPS_PKGCONFIG opencv frame_helper libelas envire
    HEADERS densestereo.h dense_stereo_types.h sparse_stereo_types.h
    homography.h store_vector.hpp psurf.h sparse_stereo.hpp)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/stereo/config.h
    DESTINATION include/stereo)

target_link_libraries(stereo ${OPENCV_GPU_LIB})
