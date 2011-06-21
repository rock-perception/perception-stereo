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

#set(CMAKE_BUILD_TYPE Release)

rock_library(dense_stereo
    SOURCES calibrationparameters.cpp configuration.cpp densestereo.cpp
	imageprocessing.cpp
    DEPS_PKGCONFIG opencv libelas envire
    HEADERS calibrationparameters.h configuration.h densestereo.h 
	imageprocessing.h dense_stereo_types.h)

#rock_executable(dense_stereo_bin main.cpp
#    DEPS dense_stereo
#    DEPS_PKGCONFIG opencv libelas)

