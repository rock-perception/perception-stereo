rock_library(dense_stereo
    SOURCES dummy.cpp)

rock_executable(dense_stereo_bin main.cpp
    DEPS dense_stereo)

