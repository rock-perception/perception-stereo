rock_find_pkgconfig(envire envire)
if (envire_FOUND)
    rock_vizkit_plugin(dense_stereo-viz
	DistanceImageVisualization.cpp
	DEPS dense_stereo
	DEPS_PKGCONFIG envire envire-viz
	HEADERS DistanceImageVisualization.hpp)
endif()
