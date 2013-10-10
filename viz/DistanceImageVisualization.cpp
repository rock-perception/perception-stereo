#include "DistanceImageVisualization.hpp"

#include <envire/operators/DistanceGridToPointcloud.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/Grids.hpp>

using namespace vizkit3d;
using namespace envire;

DistanceImageVisualization::DistanceImageVisualization()
    : m_env( new Environment() ),
    m_grid( NULL )
{
    // set up the ruby adapters
    VizPluginRubyAdapter( DistanceImageVisualization, base::samples::DistanceImage, DistanceImage );

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
    EnvireVisualization::updateData( m_env.get() );
}

DistanceImageVisualization::~DistanceImageVisualization()
{
}

void DistanceImageVisualization::updateDataIntern(base::samples::DistanceImage const& value)
{
    if( !m_grid )
    {
	// create new grid with the size based on the distance image
	m_grid = new envire::DistanceGrid( value );

	// grid was created and needs to be attached
	m_env->addInput( m_diop, m_grid );
	m_env->setFrameNode( m_grid, m_pc->getFrameNode() );
    }
    m_grid->copyFromDistanceImage( value );
    m_diop->updateAll();
}

VizkitQtPlugin( DistanceImageVisualization );

