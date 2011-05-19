#include "DistanceImageVisualization.hpp"

using namespace vizkit;

struct DistanceImageVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    dense_stereo::distance_image data;
};


DistanceImageVisualization::DistanceImageVisualization()
    : p(new Data)
{
}

DistanceImageVisualization::~DistanceImageVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> DistanceImageVisualization::createMainNode()
{
    // Geode is a common node used for vizkit plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void DistanceImageVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void DistanceImageVisualization::updateDataIntern(dense_stereo::distance_image const& value)
{
    p->data = value;
}

