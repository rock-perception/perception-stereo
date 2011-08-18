#ifndef __SPARSE_STEREO_TYPES_HPP__
#define __SPARSE_STEREO_TYPES_HPP__ 

#include <base/eigen.h>
#include <vector>
#include <base/time.h>

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

enum DESCRIPTOR
{
    DESCRIPTOR_SURF = 1,
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
	  adaptiveDetectorParam( false ),
	  detectorType( DETECTOR_SURF ),
	  filterType( STEREO )
    {}

    bool debugImage;

    int targetNumFeatures;
    int maxStereoYDeviation;

    bool adaptiveDetectorParam;
    DetectorConfiguration detectorConfig;

    DETECTOR detectorType;
    FILTER filterType;
};

struct KeyPoint
{
    typedef float Scalar;

    Scalar size;
    Scalar angle;
    Scalar response;
};

struct StereoFeatureArray
{
    base::Time time;

    typedef float Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Eigen::DontAlign> Descriptor;

    int descriptorSize;
    DESCRIPTOR descriptorType;

    std::vector<base::Vector3d> points;
    std::vector<KeyPoint> keypoints;
    std::vector<Scalar> descriptors;

    StereoFeatureArray() : descriptorSize(0) {}

    void push_back( const base::Vector3d& point, const KeyPoint& keypoint, const Descriptor& descriptor ) 
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

    size_t size() { return points.size(); }

    void clear() 
    { 
	descriptorSize = 0;
	points.clear(); 
	descriptors.clear(); 
	keypoints.clear(); 
    }
};
}

#endif
