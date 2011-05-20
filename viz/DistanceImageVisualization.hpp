#ifndef dense_stereo_DistanceImageVisualization_H
#define dense_stereo_DistanceImageVisualization_H

#include <vizkit/VizPlugin.hpp>
#include <vizkit/EnvireVisualization.hpp>
#include <dense_stereo/dense_stereo_types.h>

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
	, public VizPluginAddType<dense_stereo::distance_image>
        , boost::noncopyable
    {
    public:
        DistanceImageVisualization();
        ~DistanceImageVisualization();

    protected:
        virtual void updateDataIntern(dense_stereo::distance_image const& plan);

    private:
	boost::scoped_ptr<envire::Environment> m_env;

	envire::DistanceGridToPointcloud *m_diop;
	envire::DistanceGrid *m_grid;
	envire::Pointcloud *m_pc;
    };
}
#endif
