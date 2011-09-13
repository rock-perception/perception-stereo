#ifndef __VISUAL_ODOMETRY_STEREO_HPP__
#define __VISUAL_ODOMETRY_STEREO_HPP__

#include <stereo/sparse_stereo_types.h>
#include <frame_helper/CalibrationCv.h>
#include <base/time.h>
#include <base/eigen.h>
#include <cv.h>

namespace stereo
{

struct FeatureInfo
{
    base::Time detectorTime;
    base::Time descriptorTime;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
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
    StereoFeatureArray& getStereoFeatures() { return stereoFeatures; }

protected:
    void initDetector( size_t lastNumFeatures );
    void findFeatures( const cv::Mat &image, FeatureInfo& info );
    void crossCheckMatching( const cv::Mat& descriptors1, const cv::Mat& descriptors2, 
	    std::vector<cv::DMatch>& filteredMatches12, int knn = 1, float distanceFactor = 2.0 );

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
