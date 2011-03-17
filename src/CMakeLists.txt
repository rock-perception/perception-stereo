# include macro
include(FindPkgConfig)

# resolve dependency
pkg_check_modules(OPENCV REQUIRED "opencv")

# include directories
include_directories("${OPENCV_INCLUDE_DIRS}")

# opencv link directory
link_directories(${OPENCV_LIBRARY_DIRS})

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

rock_library(dense_stereo
    SOURCES calibrationparameters.cpp triangle.cpp
	densestereo.cpp elas.cpp descriptor.cpp
	imageprocessing.cpp matrix.cpp
    DEPS_PKGCONFIG opencv
    HEADERS calibrationparameters.h image.h timer.h
	densestereo.h elas.h imageprocessing.h triangle.h
	descriptor.h filter.h matrix.h)

#rock_executable(dense_stereo_bin main.cpp
#    DEPS dense_stereo
#    DEPS_PKGCONFIG opencv)

