rock_testsuite(test_stereo
    unit.cpp
    DEPS stereo)

rock_testsuite(undistort
    undistort.cpp
    DEPS stereo)

rock_testsuite(dense_stereo
    dense_stereo.cpp
    DEPS stereo)

if( envire_FOUND AND vizkit_FOUND )
find_package(Boost REQUIRED system)
rock_testsuite(test_stereo-viz
    viz.cpp
    DEPS stereo stereo-viz
    DEPS_PLAIN Boost_SYSTEM
    DEPS_PKGCONFIG envire-viz )
endif()
