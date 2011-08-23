#ifndef stereo_DistanceImageVisualization_H
#define stereo_DistanceImageVisualization_H

#include <vizkit/VizPlugin.hpp>
#include <vizkit/EnvireVisualization.hpp>
#include <base/samples/distance_image.h>

#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>

namespace envire
{
    class DistanceGridToPointcloud;
    class DistanceGrid;
    class Pointcloud;
}

namespace vizkit
{
    class DistanceImageVisualization
        : public EnvireVisualization
	, public VizPluginAddType<base::samples::DistanceImage>
        , boost::noncopyable
    {
    public:
        DistanceImageVisualization();
        ~DistanceImageVisualization();

    protected:
        virtual void updateDataIntern(base::samples::DistanceImage const& plan);

    private:
	boost::scoped_ptr<envire::Environment> m_env;

	envire::DistanceGridToPointcloud *m_diop;
	envire::DistanceGrid *m_grid;
	envire::Pointcloud *m_pc;
    };
}
#endif
