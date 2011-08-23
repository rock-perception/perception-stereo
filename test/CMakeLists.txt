if( envire_FOUND AND vizkit_FOUND )
rock_testsuite(test_stereo-viz
    unit.cpp
    DEPS stereo stereo-viz
    DEPS_PKGCONFIG envire-viz )
endif()
