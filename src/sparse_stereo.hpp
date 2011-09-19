#ifndef __VISUAL_ODOMETRY_STEREO_HPP__
#define __VISUAL_ODOMETRY_STEREO_HPP__

#include <stereo/sparse_stereo_types.h>
#include <frame_helper/CalibrationCv.h>
#include <base/time.h>
#include <base/eigen.h>
#include <cv.h>

namespace stereo
{

class StereoFeatures
{
public:
    StereoFeatures();

    /** Set the calibration parameters for the stereo camera setup.
     */
    void setCalibration( const frame_helper::StereoCalibration &calib );

    /** Set the configuration for the stereo processing chain.
     * This will also reset any previous configuration including detector type.
     */
    void setConfiguration( const FeatureConfiguration &config );

    /** Perform the processing on a stereo frame pair.
     * This will perform feature detection, description, matching and filtering 
     * based on the configuration given in the configuration class.
     */
    void processFramePair( const cv::Mat &left_image, const cv::Mat &right_image );

    /** Get the result of the last stereo image processing step.
     */
    StereoFeatureArray& getStereoFeatures() { return stereoFeatures; }

    /** calculate the relation between two stereo pairs
     */
    void calculateInterFrameCorrespondences( const envire::Featurecloud* fc1, const envire::Featurecloud* fc2, int filterMethod );
    void calculateInterFrameCorrespondences( const StereoFeatureArray& frame1, const StereoFeatureArray& frame2, int filterMethod );
    void calculateInterFrameCorrespondences( const cv::Mat& feat1, const std::vector<envire::KeyPoint> keyp1, const cv::Mat& feat2, const std::vector<envire::KeyPoint> keyp2, int filterMethod );

    /** Get the correspondences of the last interframe calculation 
     * @return - a vector of an std::pair, where first is an index to frame1 and
     *	    second an index to frame2 as given by the
     *	    calculateDepthInformationBetweenCorrespondences()
     */
    std::vector<std::pair<long,long> > getInterFrameCorrespondences() { return correspondences; }

    /** get the debug image for a stereo pair, if debugImage has been 
     * activated in the configuration.
     *
     * @todo generate the image in this function instead through the pipeline.
     *   to allow generation based on result types. 
     */
    const cv::Mat& getDebugImage() { return debugImage; }

    /** get the debug image for interframe correspondences. 
     */
    cv::Mat getInterFrameDebugImage( const cv::Mat& debug1, const StereoFeatureArray& frame1, const cv::Mat& debug2, const StereoFeatureArray& frame2 );

public:
    void findFeatures( const cv::Mat &left_image, const cv::Mat &right_image );
    bool getPutativeStereoCorrespondences();
    bool refineFeatureCorrespondences();
    void calculateDepthInformationBetweenCorrespondences();

    void crossCheckMatching( const cv::Mat& descriptors1, const cv::Mat& descriptors2, 
	    std::vector<cv::DMatch>& filteredMatches12, int knn = 1, float distanceFactor = 2.0 );

protected:
    struct FeatureInfo
    {
	base::Time detectorTime;
	base::Time descriptorTime;

	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
    };

    void initDetector( size_t lastNumFeatures );
    void findFeatures( const cv::Mat &image, FeatureInfo& info );

    frame_helper::StereoCalibrationCv calib;
    FeatureConfiguration config;
    DetectorConfiguration detectorParams;

    FeatureInfo leftFeatures, rightFeatures;
    FeatureInfo leftPutativeMatches, rightPutativeMatches;
    FeatureInfo leftMatches, rightMatches;

    StereoFeatureArray stereoFeatures;
    std::vector<std::pair<long,long> > correspondences;

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;

    cv::Mat debugImage;
    int debugRightOffset;
};

}

#endif
