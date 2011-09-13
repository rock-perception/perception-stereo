#ifndef __SPARSE_STEREO_TYPES_HPP__
#define __SPARSE_STEREO_TYPES_HPP__ 

#include <base/eigen.h>
#include <vector>
#include <base/time.h>
#include <envire/maps/Featurecloud.hpp>

namespace stereo
{
enum DETECTOR
{
    DETECTOR_SURF = 1,
    DETECTOR_GOOD = 2,
    DETECTOR_SURFGPU = 3,
    DETECTOR_STAR = 4,
    DETECTOR_MSER = 5,
    DETECTOR_SIFT = 6,
    DETECTOR_FAST = 7,
};

enum FILTER
{
    NONE,
    HOMOGRAPHY,
    FUNDAMENTAL,
    INTELLIGENT,
    STEREO,
};

struct DetectorConfiguration
{
    DetectorConfiguration()
	: SURFparam(170),
	goodParam(0.1),
	mserParam(3),
	starParam(9),
	fastParam(12)
    {}

    int SURFparam;
    float goodParam;
    float mserParam;
    float starParam;
    float fastParam;
};

struct FeatureConfiguration
{
    FeatureConfiguration() 
	: debugImage( true ),
	  targetNumFeatures( 100 ),
	  maxStereoYDeviation( 5 ),
	  knn( 1 ),
	  distanceFactor( 2.0 ),
	  adaptiveDetectorParam( false ),
	  detectorType( DETECTOR_SURF ),
	  filterType( STEREO )
    {}

    bool debugImage;

    int targetNumFeatures;
    int maxStereoYDeviation;

    int knn;
    int distanceFactor;

    bool adaptiveDetectorParam;
    DetectorConfiguration detectorConfig;

    DETECTOR detectorType;
    FILTER filterType;
};

struct StereoFeatureArray
{
    base::Time time;

    typedef float Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Eigen::DontAlign> Descriptor;

    int descriptorSize;
    envire::DESCRIPTOR descriptorType;

    std::vector<base::Vector3d> points;
    std::vector<envire::KeyPoint> keypoints;
    std::vector<Scalar> descriptors;

    StereoFeatureArray() : descriptorSize(0) {}

    void push_back( const base::Vector3d& point, const envire::KeyPoint& keypoint, const Descriptor& descriptor ) 
    {
	points.push_back( point );
	keypoints.push_back( keypoint );

	if( descriptorSize == 0 )
	    descriptorSize = descriptor.size();

	assert( descriptorSize == descriptor.size() );

	// try to have some efficiency in copying the descriptor data
	descriptors.resize( descriptors.size() + descriptorSize );
	memcpy( &descriptors[0] + descriptors.size() - descriptorSize, descriptor.data(), descriptorSize ); 
    }

    Eigen::Map<Descriptor> getDescriptor( size_t index )
    { 
	return Eigen::Map<Descriptor>( &descriptors[index*descriptorSize], descriptorSize ); 
    }

    Eigen::Map<const Descriptor> getDescriptor( size_t index ) const
    { 
	return Eigen::Map<const Descriptor>( &descriptors[index*descriptorSize], descriptorSize ); 
    }

    size_t size() const 
    { 
	return points.size(); 
    }

    void clear() 
    { 
	descriptorSize = 0;
	points.clear(); 
	descriptors.clear(); 
	keypoints.clear(); 
    }

    void copyTo( envire::Featurecloud& fc ) const
    {
	fc.clear();

	std::copy( points.begin(), points.end(), std::back_inserter( fc.vertices ) );
	fc.keypoints = keypoints;
	fc.descriptors = descriptors;
	fc.descriptorType = descriptorType;
	fc.descriptorSize = descriptorSize;
    }
};
}

#endif
