if( envire_FOUND AND vizkit_FOUND )
rock_testsuite(test_dense_stereo-viz
    unit.cpp
    DEPS dense_stereo dense_stereo-viz
    DEPS_PKGCONFIG envire-viz )
endif()
