#ifndef __STEREO_HOMGRAPHY_H__
#define __STEREO_HOMGRAPHY_H__

#include <base/samples/distance_image.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues> 

#include "opencv2/features2d/features2d.hpp"

namespace stereo
{
    struct Homography
    {
	/** 
	 * estimate homography from distance image 
	 * this functions samples the region around center_x, center_y
	 * in the distance image and creates a homography transform
	 * for that patch.
	 */
	bool estimateFromDistanceImage( 
		const base::samples::DistanceImage &dImage, 
		size_t center_x, size_t center_y, double radius );

	/** 
	 * use the homography transform to reproject a patch with the
	 * given radius to the target.
	 */
	void reproject( const cv::Mat& source, cv::Mat& target, size_t center_x, size_t center_y, double radius );

	const Eigen::Matrix3f& getTransform() { return homography; }

	Eigen::Matrix3f intrinsic;
	Eigen::Matrix3f homography;
	cv::Mat dbgImg;
    };
}

#endif
