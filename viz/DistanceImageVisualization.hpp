#ifndef stereo_DistanceImageVisualization_H
#define stereo_DistanceImageVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <vizkit3d/EnvireVisualization.hpp>
#include <base/samples/distance_image.h>

#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>

namespace envire
{
    class DistanceGridToPointcloud;
    class DistanceGrid;
    class Pointcloud;
}

namespace vizkit3d
{
    class DistanceImageVisualization
        : public envire::EnvireVisualization
	, public VizPluginAddType<base::samples::DistanceImage>
        , boost::noncopyable
    {
	Q_OBJECT

    public:
        DistanceImageVisualization();
        ~DistanceImageVisualization();
	
	Q_INVOKABLE void updateData(const base::samples::DistanceImage& data) 
	{ Vizkit3DPlugin<envire::Environment*>::updateData(data); }

	Q_INVOKABLE void updateDistanceImage(const base::samples::DistanceImage& data) 
	{ Vizkit3DPlugin<envire::Environment*>::updateData(data); }

    protected:
        virtual void updateDataIntern(base::samples::DistanceImage const& data);

    private:
	boost::scoped_ptr<envire::Environment> m_env;

	envire::DistanceGridToPointcloud *m_diop;
	envire::DistanceGrid *m_grid;
	envire::Pointcloud *m_pc;
    };
}
#endif
