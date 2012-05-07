#include "homography.h"
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace stereo;

void calcHomography( const Eigen::Vector3f& center, const Eigen::Vector3f& normal, Eigen::Matrix3f& homography )
{
    // get rotation which rotates the camera center point onto the normal vector
    Eigen::Vector3f r = -normal.cross( -Eigen::Vector3f::UnitZ() );
    Eigen::Matrix3f R = Eigen::AngleAxisf( asin( r.norm() ), r.normalized() ).toRotationMatrix();
    
    // distance of the plane to camera 
    float d = center.dot( Eigen::Vector3f::UnitZ() );

    // get the translation which rotates the camera around the feature center point
    Eigen::Vector3f t = (-R*center + center) / d;

    // get the homography matrix
    Eigen::Matrix3f H = R - (t*(-Eigen::Vector3f::UnitZ()).transpose());

    homography = H;
}


bool Homography::estimateFromDistanceImage( const base::samples::DistanceImage &dImage, size_t center_x, size_t center_y, double radius )
{
    // get the center point  
    Eigen::Vector3f center;
    if( !dImage.getScenePoint( center_x, center_y, center ) )
    {
	// TODO we could just take the point from the plane here
	// so that we wouldn't reject as many possible candidates
	return false;
    }

    // ratio between smallest and second smalles eigenvector
    double eigen_ratio = 1.0;
    double view_angle = M_PI/2.0;

    // calculate the mean and covariance of 3d points 
    Eigen::Vector3f mu(Eigen::Vector3f::Zero());
    Eigen::Matrix3f sigma(Eigen::Matrix3f::Zero());
    size_t sample_count = 0;

    // initialize the intrinsic matrix
    intrinsic = dImage.getIntrinsic<float>();

    // initialize current best homography as identity
    homography = Eigen::Matrix3f::Identity();

    // the way the sampling for the normal works now, 
    // is to sample a number of circles around the center
    // each circle is projected by the current best guess
    // for the homography
    const size_t max_iter = 4;
    for( size_t iter = 0; iter < max_iter; iter++ )
    {
	// precalculate the homography in pixel-coords
	Eigen::Matrix3f G = intrinsic * homography * intrinsic.inverse();

	// sample a circle around the center and fill 
	// mean and covariance
	const size_t max_sample_count = 6 + 2 * iter;
	for( size_t i = 0; i < max_sample_count ; i++ )
	{
	    float r = radius * (iter + 1.0) / max_iter;
	    float x = center_x + r * sin( i * M_PI * 2.0 / max_sample_count );
	    float y = center_y + r * cos( i * M_PI * 2.0 / max_sample_count );

	    // project x and y according to current best homography
	    Eigen::Vector3f hp = G * Eigen::Vector3f( x, y, 1.0 );
	    hp /= hp.z();

	    Eigen::Vector3f p;
	    if( dImage.getScenePoint( hp.x(), hp.y(), p ) )
	    {
		cv::circle( dbgImg, cv::Point( hp.x(), hp.y() ), 0, cv::Scalar(0,0,255) );

		sigma += p * p.transpose();
		mu += p;
		sample_count++;
	    }
	}

	if( sample_count >= 4 )
	{
	    // calculate mean and covariance
	    Eigen::Vector3f mu_i = mu / sample_count;
	    Eigen::Matrix3f sigma_i = sigma / sample_count - mu_i * mu_i.transpose();

	    // solve the eigenvalues of the covariance matrix
	    // and get the shortest eigenvector
	    Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver(sigma_i);
	    Eigen::Matrix3f::Index idx; 
	    eigenSolver.eigenvalues().real().minCoeff( &idx );
	    Eigen::Vector3f normal = eigenSolver.eigenvectors().real().col( idx );
	    normal.normalize();

	    // look at the ratio between smallest and other eigenvalues
	    size_t idx1 = (idx != 0? 0 : 1), idx2 = (idx != 2? 2 : 1);
	    eigen_ratio = std::max( 
		    eigenSolver.eigenvalues().real()[idx] / eigenSolver.eigenvalues().real()[ idx1 ],
		    eigenSolver.eigenvalues().real()[idx] / eigenSolver.eigenvalues().real()[ idx2 ] );

	    // flip the normal if it's not within +-90 deg of the view vector
	    view_angle = acos( normal.dot( center.normalized() ) );
	    if( view_angle > (M_PI/2.0) )
	    {
		normal *= -1.0f;
		view_angle = acos( normal.dot( center.normalized() ) );
	    }

	    //normal = Eigen::AngleAxisf( M_PI / 2.0, Eigen::Vector3f::UnitX()) * -Eigen::Vector3f::UnitZ();

	    calcHomography( center, normal, homography );
	}
    }

    // std::cout << "eigen_ratio: " << eigen_ratio << " view_angle: " << view_angle * 180/M_PI << std::endl;

    if( eigen_ratio > 0.2 )
	return false;

    if( view_angle > (80 / 180.0 * M_PI) )
	return false;

    return true;
}

void Homography::reproject( const cv::Mat& source, cv::Mat& target, size_t center_x, size_t center_y, double radius )
{
    // the window size is 2 * radius
    // create a new cv::Mat of that size and reproject that patch from the origin 
    // center point using the homography
    size_t width = radius * 2, height = radius * 2;
    target.create( width, height, source.type() );

    // create a intrinsic matrix for the resulting patch based on the current intinsic matrix
    Eigen::Matrix3f patch_intrinsic( intrinsic );
    patch_intrinsic.topRightCorner<2,1>() += Eigen::Vector2f( -(center_x - radius), -(center_y - radius) );

    // create opencv transform object
    Eigen::Matrix3f h( intrinsic * homography * patch_intrinsic.inverse());
    /*
    Eigen::Vector3f p = h * Eigen::Vector3f( center_x, center_y, 1.0 );
    std::cout << p.transpose() << std::endl;
    p /= p.z();
    std::cout << p.transpose() << std::endl;
    std::cout << std::endl;
    */

    cv::Mat trans;
    cv::eigen2cv( h, trans );

    // perform the transform
    cv::warpPerspective( source, target, trans, target.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP );
}

