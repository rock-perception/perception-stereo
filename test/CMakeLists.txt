rock_testsuite(test_stereo
    unit.cpp
    DEPS stereo)

if( envire_FOUND AND vizkit_FOUND )
rock_testsuite(test_stereo-viz
    viz.cpp
    DEPS stereo stereo-viz
    DEPS_PKGCONFIG envire-viz )
endif()
