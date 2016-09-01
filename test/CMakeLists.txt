rock_testsuite(test_stereo
    unit.cpp
    DEPS stereo)

rock_testsuite(undistort
    undistort.cpp
    DEPS stereo)

rock_testsuite(dense_stereo
    dense_stereo.cpp
    DEPS stereo)

