#include "icpSolver.h"
#include "ransac.h"

#include <iostream>

using namespace groupsac;
using namespace groupsac::estimators;

int main( int argc, char* argv[] )
{
    std::cout << "RANSAC test" << std::endl;

    icpSolver solver;
    icpSolver::T dataPoints;
    Eigen::Affine3d model = Eigen::Translation3d( 0, 0, 0.5 ) * 
	Eigen::AngleAxisd( M_PI/8.0, Eigen::Vector3d::UnitX() );

    const double outlierProb = 0.1;

    for( int i = 0; i < 50; i++ )
    {
	Eigen::Vector3d v1 = Eigen::Vector3d::Random() * 10.0;
	Eigen::Vector3d v2 = model * v1;

	if( rand() % 100 < outlierProb * 100 )
	{
	    v2 += Eigen::Vector3d::Random() * 10.0;
	}

	dataPoints.push_back( std::make_pair( v1, v2 ) );
    }

    std::vector<int> inliers;
    std::vector<Eigen::Affine3d> models;

    ransac::Ransac_Handler ransac_fun_Handler;
    ransac::Ransac_RobustEstimator
	(
	 dataPoints, // the input data
	 estimators::icpSolver::extractor, // How select sampled point from indices
	 dataPoints.size(),  // the number of putatives data
	 solver,  // compute the underlying model given a sample set
	 estimators::icpSolver::defaultEvaluator,  // the function to evaluate a given model
	 //Ransac Object that contain function:
	 // CandidatesSelector, Sampler and TerminationFunction
	 ransac_fun_Handler, // the basic ransac object
	 1000,  // the maximum rounds for RANSAC routine
	 inliers, // inliers to the final solution
	 models, // models array that fit input data
	 0.95, // the confidence want to achieve at the end
	 0.1
	); 

}
