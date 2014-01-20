#ifndef __VISUAL_ODOMETRY_STEREO_HPP__
#define __VISUAL_ODOMETRY_STEREO_HPP__

#include <stereo/config.h>
#include <stereo/sparse_stereo_types.h>
#include <frame_helper/CalibrationCv.h>
#include <base/time.h>
#include <base/eigen.h>
#include <opencv2/opencv.hpp>
#include <base/samples/distance_image.h>

#ifdef OPENCV_HAS_GPUMAT_IN_CORE
#include <opencv2/core/gpumat.hpp>
#else
#include <opencv2/gpu/gpu.hpp>
#endif

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

    /** Set the calibration parameters for the stereo camera setup.
     */
    void setCalibration( const frame_helper::StereoCalibration &calib );

    /** Set the configuration for the stereo processing chain.
     * This will also reset any previous configuration including detector type.
     */
    void setConfiguration( const FeatureConfiguration &config);

    /** Set the configuration for the feature detector 
     */
    void setDetectorConfiguration( const DetectorConfiguration &detector_config );

    /** optionally set the distance images before each call to process frame 
     * pair, in order to perform a perspective undistort of the features
     * before running the descriptor.
     * @param left - left distance image
     * @param right - right distance image
     */
    void setDistanceImages( 
	    const base::samples::DistanceImage *left, 
	    const base::samples::DistanceImage *right );

    /** Perform the processing on a stereo frame pair.
     * This will perform feature detection, description, matching and filtering 
     * based on the configuration given in the configuration class.
     * If the optional last parameter is given, it will not work in class internal 
     * storage, but in the storage provided.
     */
    void processFramePair( const cv::Mat &left_image, const cv::Mat &right_image, StereoFeatureArray *stereo_features = NULL );
     
    /** Get the result of the last stereo image processing step.
     */
    StereoFeatureArray& getStereoFeatures() { return stereoFeatures; }

    /** Get the result of the last feature extraction, left
     */
    FeatureInfo& getFeatureInfoLeft() { return leftFeatures; }

    /** Get the result of the last feature extraction, left
     */
    FeatureInfo& getFeatureInfoRight() { return rightFeatures; }

    /** calculate the relation between two stereo pairs
     */
    void calculateInterFrameCorrespondences( const envire::Featurecloud* fc1, const envire::Featurecloud* fc2, int filterMethod );
    void calculateInterFrameCorrespondences( const StereoFeatureArray& frame1, const StereoFeatureArray& frame2, int filterMethod );
    void calculateInterFrameCorrespondences( 
	    const cv::Mat& feat1, const std::vector<envire::KeyPoint>& keyp1, const std::vector<Eigen::Vector3d>& points1,
	    const cv::Mat& feat2, const std::vector<envire::KeyPoint>& keyp2, const std::vector<Eigen::Vector3d>& points2, 
	    int filterMethod );

    /** Get the correspondences of the last interframe calculation 
     * @return - a vector of an std::pair, where first is an index to frame1 and
     *	    second an index to frame2 as given by the
     *	    calculateDepthInformationBetweenCorrespondences()
     */
    std::vector<std::pair<long,long> > getInterFrameCorrespondences() { return correspondences; }

    /** Get the transform calculated at the last interframe correspondence calculation.
     * Only valid if the filter type was set to ISOMETRY and the number of correspondences > 3.
     */
    base::Affine3d getInterFrameCorrespondenceTransform() { return correspondenceTransform; }

    /** get the debug image for a stereo pair, if debugImage has been 
     * activated in the configuration.
     *
     * @todo generate the image in this function instead through the pipeline.
     *   to allow generation based on result types. 
     */
    const cv::Mat& getDebugImage() { return debugImage; }

    /** get the debug image for interframe correspondences. 
     */
    cv::Mat getInterFrameDebugImage( const cv::Mat& debug1, const StereoFeatureArray& frame1, const cv::Mat& debug2, const StereoFeatureArray& frame2 , std::vector<std::pair<long,long> > *correspondence = NULL);

public:
    // use threading parameter: 0 for no threads, 1 for 2 threads (one per image), 2 for 8 threads (4 per image).
    void findFeatures( const cv::Mat &left_image, const cv::Mat &right_image, int use_threading = 1, int crop_left = 0, int crop_right = 0); 
    bool getPutativeStereoCorrespondences();
    bool refineFeatureCorrespondences();
    void calculateDepthInformationBetweenCorrespondences(StereoFeatureArray *stereo_features = NULL);

    void crossCheckMatching( const cv::Mat& descriptors1, const cv::Mat& descriptors2, 
	    std::vector<cv::DMatch>& filteredMatches12, int knn = 1, float distanceFactor = 2.0 );

    cv::Mat getHomography() { return homography;}

protected:
    void initDetector( size_t lastNumFeatures );
    void findFeatures2( const cv::Mat &image, FeatureInfo& info, bool left_frame = true, int crop_left = 0, int crop_right = 0 );
    void findFeatures_threading( const cv::Mat &image, FeatureInfo& info, bool left_frame = true, int crop_left = 0, int crop_right = 0);

    void crossCheckMatching( std::vector<std::vector<cv::DMatch> > matches12, std::vector<std::vector<cv::DMatch> > matches21, std::vector<cv::DMatch>& filteredMatches12, int knn = 1, float distanceFactor = 2.0);

    frame_helper::StereoCalibrationCv calib;
    FeatureConfiguration config;
    DetectorConfiguration detectorParams;

    FeatureInfo leftFeatures, rightFeatures;
    FeatureInfo leftPutativeMatches, rightPutativeMatches;
    FeatureInfo leftMatches, rightMatches;

    StereoFeatureArray stereoFeatures;
    std::vector<std::pair<long,long> > correspondences;
    base::Affine3d correspondenceTransform;

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
 
    cv::Mat homography;

    cv::Mat debugImage;
    int debugRightOffset;
    const base::samples::DistanceImage *dist_left, *dist_right;
    bool use_gpu_detector;
    cv::gpu::GpuMat descriptors_gpu_left;
    cv::gpu::GpuMat descriptors_gpu_right;
};

}

#endif
