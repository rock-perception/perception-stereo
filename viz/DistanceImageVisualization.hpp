#ifndef dense_stereo_DistanceImageVisualization_H
#define dense_stereo_DistanceImageVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/VizPlugin.hpp>
#include <osg/Geode>

namespace vizkit
{
    class DistanceImageVisualization
        : public vizkit::VizPlugin<dense_stereo::distance_image>
        , boost::noncopyable
    {
    public:
        DistanceImageVisualization();
        ~DistanceImageVisualization();

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(dense_stereo::distance_image const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
