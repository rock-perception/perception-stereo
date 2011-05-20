#include "DistanceImageVisualization.hpp"

#include <envire/operators/DistanceGridToPointcloud.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/Grids.hpp>

using namespace vizkit;
using namespace envire;

DistanceImageVisualization::DistanceImageVisualization()
    : m_env( new Environment() ),
    m_grid( NULL )
{
    // set the environment
    FrameNode *c_fm = new FrameNode();
    m_env->getRootNode()->addChild( c_fm );

    // create the target pointcloud 
    m_pc = new Pointcloud();
    m_env->setFrameNode( m_pc, c_fm );
    
    // create the operator
    m_diop = new DistanceGridToPointcloud();

    // set output, leave input open, since this will be the distance map that
    // we haven't initialized yet, because we don't know the size yet
    m_env->addOutput( m_diop, m_pc );

    // set the environment for the plugin
    updateData( m_env.get() );
}

DistanceImageVisualization::~DistanceImageVisualization()
{
}

void DistanceImageVisualization::updateDataIntern(dense_stereo::distance_image const& value)
{
    const size_t 
	width = value.width, 
	height = value.height;

    if( !m_grid )
    {
	// create new grid and attach it 
	m_grid = new DistanceGrid( 
		value.width, value.height, 
		value.scale_x, value.scale_y, 
		value.center_x, value.center_y );

	m_env->addInput( m_diop, m_grid );
	m_env->setFrameNode( m_grid, m_pc->getFrameNode() );
    }

    DistanceGrid::ArrayType& distance = 
	m_grid->getGridData( DistanceGrid::DISTANCE );

    // copy the content not very performant but should do for now.
    for( size_t x = 0; x<width; x++ )
    {
	for( size_t y = 0; y<height; y++ )
	{
	    distance[y][x] = value.data[y*width+x];
	}
    }

    m_diop->updateAll();
}

