rock_find_pkgconfig(envire envire)
if (envire_FOUND)
    rock_vizkit_plugin(stereo-viz
	MOC DistanceImageVisualization.cpp
	DEPS stereo
	DEPS_PKGCONFIG envire envire-viz
	HEADERS DistanceImageVisualization.hpp)
endif()
