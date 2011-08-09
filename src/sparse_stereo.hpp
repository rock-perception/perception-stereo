#ifndef __VISUAL_ODOMETRY_STEREO_HPP__
#define __VISUAL_ODOMETRY_STEREO_HPP__

#include <frame_helper/CalibrationCv.h>
#include <base/time.h>
#include <base/eigen.h>
#include <cv.h>

namespace stereo
{
enum DETECTOR
{
    SURF,
    GOOD,
    SURFGPU,
    STAR,
    MSER,
    SIFT,
    FAST
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
	  maxStereoYDeviation( 20 ),
	  adaptiveDetectorParam( false ),
	  detectorType( SURF ),
	  filterType( NONE )
    {}

    bool debugImage;

    int targetNumFeatures;
    int maxStereoYDeviation;

    bool adaptiveDetectorParam;
    DetectorConfiguration detectorConfig;

    DETECTOR detectorType;
    FILTER filterType;
};

struct FeatureInfo
{
    base::Time detectorTime;
    base::Time descriptorTime;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
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
    typedef float Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Eigen::DontAlign> Descriptor;

    int descriptorSize;
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

class StereoFeatures
{
public:
    StereoFeatures();
    void setCalibration( const frame_helper::StereoCalibration &calib );
    void setConfiguration( const FeatureConfiguration &config );

    void processFramePair( const cv::Mat &left_image, const cv::Mat &right_image );

    void findFeatures( const cv::Mat &left_image, const cv::Mat &right_image );
    bool getPutativeStereoCorrespondences();
    bool refineFeatureCorrespondences();
    void calculateDepthInformationBetweenCorrespondences();

    const cv::Mat& getDebugImage() { return debugImage; }
    const StereoFeatureArray& getStereoFeatures() { return stereoFeatures; }

protected:
    void initDetector( size_t lastNumFeatures );
    void findFeatures( const cv::Mat &image, FeatureInfo& info );
    void crossCheckMatching( const cv::Mat& descriptors1, const cv::Mat& descriptors2, 
	    std::vector<cv::DMatch>& filteredMatches12, int knn = 1 );

    frame_helper::StereoCalibrationCv calib;
    FeatureConfiguration config;
    DetectorConfiguration detectorParams;

    FeatureInfo leftFeatures, rightFeatures;
    FeatureInfo leftPutativeMatches, rightPutativeMatches;
    FeatureInfo leftMatches, rightMatches;

    StereoFeatureArray stereoFeatures;

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;

    cv::Mat debugImage;
    int debugRightOffset;
};

}

#endif
