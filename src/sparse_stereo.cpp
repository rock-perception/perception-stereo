#include "sparse_stereo.hpp"
#include <Eigen/Core>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <envire/ransac.hpp>
#include "psurf.h"

using namespace stereo;
using namespace std;

cv::Point eigen2cv( const Eigen::Vector2d& point )
{
    return cv::Point( point.x(), point.y() );
}

Eigen::Vector2d cv2eigen( const cv::Point& point )
{
    return Eigen::Vector2d( point.x, point.y );
}


StereoFeatures::StereoFeatures()
    : dist_left( NULL ), dist_right( NULL )
{
    descriptorMatcher = cv::DescriptorMatcher::create("FlannBased");
    initDetector( config.targetNumFeatures );
    setConfiguration( FeatureConfiguration() );
    use_gpu_detector = false;
}

void StereoFeatures::setCalibration( const frame_helper::StereoCalibration &calib )
{
    this->calib.setCalibration( calib );
}

void StereoFeatures::setDetectorConfiguration( const DetectorConfiguration &detector_config )
{
   detectorParams = detector_config; 
}

void StereoFeatures::setConfiguration( const FeatureConfiguration &config )
{
    this->config = config;
    initDetector( config.targetNumFeatures );

    if( config.descriptorType == envire::DESCRIPTOR_SURF )
	descriptorExtractor = new cv::SurfDescriptorExtractor(4, 3, false);
    else if( config.descriptorType == envire::DESCRIPTOR_PSURF )
	descriptorExtractor = new cv::PSurfDescriptorExtractor(4, 3, false);
    else
	throw std::runtime_error( "Unknown descriptorType" );
}

void StereoFeatures::setDistanceImages( 
	const base::samples::DistanceImage *left, 
	const base::samples::DistanceImage *right )
{
    dist_left = left;
    dist_right = right;
}

void StereoFeatures::initDetector( size_t lastNumFeatures )
{
    DETECTOR detectorType = config.detectorType;
    size_t localLastNumFeatures = lastNumFeatures;
    size_t targetNumFeatures = config.targetNumFeatures;
    switch(detectorType)
    {
	case DETECTOR_SURFGPU:
	    {
		// TODO subclass opencv detector interface for surfgpu
		// return surfgpu(leftImage, rightImage, lastNumFeatures, targetNumFeatures);
                config.detectorType = DETECTOR_SURF;
                std::cout << "StereoFeatures::initDetector: Warning: DETECTOR_SURFGPU was selected, which is currently not implemented. Automatically switching to DETECTOR_SURF." << std::endl;
                initDetector(lastNumFeatures);
	    }
	    break;
	case DETECTOR_SURF:
	    {
		int &SURFparam = detectorParams.SURFparam;
		// adaptively adjust the parameters for the SURF extractor in order to get around TARGET_NUM_FEATURES features
		//            double surfParamDiff = (localLastNumFeatures - targetNumFeatures);
		double surfParamDiff = (double)localLastNumFeatures / (double)targetNumFeatures;

		SURFparam = (int)((double)SURFparam * sqrt(surfParamDiff));

		// to prevent the value from running haywire, cap it
		if(SURFparam < 3)
		    SURFparam = 3;
		if(SURFparam > 40000)
		    SURFparam = 40000;

                if(SURFparam == 40000 || SURFparam == 3)
                  std::cout << "Warning: it seems the Detector cannot adapt its parameter well enough and encountered a safety cap. Check input images." << std::endl;

		// double hessianThreshold = 400., int octaves = 3, int octaveLayers = 4
		detector = new cv::SurfFeatureDetector( SURFparam, 4, 3 );
	    }
	    break;
	case DETECTOR_GOOD:
	    {   
		float &goodParam = detectorParams.goodParam;
		double goodParamDiff = (double)localLastNumFeatures / (double)targetNumFeatures;
		goodParam = goodParam * goodParamDiff;
		goodParam += (localLastNumFeatures - targetNumFeatures) / 10000.0;

		if(goodParam < 0.01)
		    goodParam = 0.01;
		// int maxNumFeatures, double qualityLevel, double minDistance, int blockSize, bool useHarrisDetector, double k
		detector = new cv::GoodFeaturesToTrackDetector( targetNumFeatures + 20, goodParam, 15.0, 15, false, 0.04 );
	    }
	    break;
	case DETECTOR_SIFT:
	    {
		// double threshold, double edgeThreshold, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, 
		// int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE
		detector = new cv::SiftFeatureDetector();
	    }
	    break;
	case DETECTOR_MSER:
	    {
		float &mserParam = detectorParams.mserParam;
		mserParam += (localLastNumFeatures - targetNumFeatures) / 100.0;
		// int delta, int minArea, int maxArea, float maxVariation, float minDiversity, int maxEvolution, double areaThreshold, double minMargin, int edgeBlurSize 
		detector = new cv::MserFeatureDetector(mserParam * 10.0, 5, 500, 1.0, 0.5, 1, 1.0, 0.0, 1);
	    }
	    break;
	case DETECTOR_STAR:
	    {
		float &starParam = detectorParams.starParam;
		starParam += (localLastNumFeatures - targetNumFeatures) / 100.0;
		// int maxSize=16, int responseThreshold=30, int lineThresholdProjected = 10, int lineThresholdBinarized=8, int suppressNonmaxSize=5
		detector = new cv::StarFeatureDetector(16, starParam, 6, 8, 5);
	    }
	    break;
	case DETECTOR_FAST:
	    {
		float &fastParam = detectorParams.fastParam;
		fastParam += (localLastNumFeatures - targetNumFeatures) / 100.0;
		// int threshold = 1
		detector = new cv::FastFeatureDetector(fastParam);
	    }
	    break;
        case DETECTOR_SURF_CV_GPU:
            {
		int &SURFparam = detectorParams.SURFparam;
		// adaptively adjust the parameters for the SURF extractor in order to get around TARGET_NUM_FEATURES features
		//            double surfParamDiff = (localLastNumFeatures - targetNumFeatures);
		double surfParamDiff = (double)localLastNumFeatures / (double)targetNumFeatures;

		SURFparam = (int)((double)SURFparam * sqrt(surfParamDiff));

		// to prevent the value from running haywire, cap it
		if(SURFparam < 3)
		    SURFparam = 3;
		if(SURFparam > 115500)
		    SURFparam = 115500;

                use_gpu_detector = true;
//std::cout << "SurfParam: " << detectorParams.SURFparam << " LastNumFeatures: " << localLastNumFeatures << " targetNumFeatures: " << targetNumFeatures<< std::endl;
            }
            break;
	default: 
	    throw std::runtime_error("Selected feature detector is not implemented.");
    }
}

void StereoFeatures::findFeatures( const cv::Mat &image, FeatureInfo& info, bool left_frame )
{
    clock_t start, finish;

    if(!use_gpu_detector)
    {
        start = clock();
        detector->detect( image, info.keypoints);
        finish = clock();
        info.detectorTime = base::Time::fromSeconds( (finish - start) / (CLOCKS_PER_SEC * 1.0) );
        start = clock();
        descriptorExtractor->compute( image, info.keypoints, info.descriptors );
        finish = clock();
        info.descriptorTime = base::Time::fromSeconds( (finish - start) / (CLOCKS_PER_SEC * 1.0) );
    }
    else
    {
        // the structure for the GPU detector is a bit different, so handle it separately
        try
        {
            cv::gpu::SURF_GPU surf(detectorParams.SURFparam, 4, 3, true);
            // size of a sincle descriptor. 128 if exteded == true, 64 if extended == false
            int desc_size = 128;
            // in descriptors_gpu we carry a pointer to the respective descriptors_gpu_[left:right] structure on the GPU. This is needed later for descriptor matching on the GPU.
            cv::gpu::GpuMat *descriptors_gpu = &descriptors_gpu_left;
            cv::gpu::GpuMat keypoints_gpu;
            cv::gpu::GpuMat gpu_image(image);
            if(!left_frame)
                descriptors_gpu = &descriptors_gpu_right;
            // calculate the surf detector/descriptor pair
            surf(gpu_image, cv::gpu::GpuMat(), keypoints_gpu, *descriptors_gpu);
            // download keypoints
            surf.downloadKeypoints(keypoints_gpu, info.keypoints);
            // download descriptors to temporary variables
            std::vector<float> descriptor;
            surf.downloadDescriptors(*descriptors_gpu, descriptor);
            // copy the descriptors to the matrix space in the global structure
            info.descriptors = cv::Mat(0, desc_size, CV_32F, 1);
            for(int i = 0; i < (int)info.keypoints.size(); ++i)
            {
                cv::Mat row(1, desc_size, CV_32F, &(descriptor[i * desc_size]), 1);
                info.descriptors.push_back(row);
            }
        }
        catch(...)
        {
            std::cout << "FindFeatures (Warn): detectorType == DETECTOR_SURF_CV_GPU was selected, but opencv was not build with CUDA support Switching to CPU-SURF (detectorType == DETECTOR_SURF). Please Re-Build opencv with CUDA enabled to use DETECTOR_SURF_CV_GPU." << std::endl;
            use_gpu_detector = false;
            findFeatures( image, info );
        }
    }
}

void StereoFeatures::processFramePair( const cv::Mat &left_image, const cv::Mat &right_image, StereoFeatureArray *stereo_features )
{
    stereoFeatures.clear();

    findFeatures( left_image, right_image );
    if(!getPutativeStereoCorrespondences())
    {
      std::cout << "stereo::getPutativeStereoCorrespondences: returned false." << std::endl;
      return;
    }
    refineFeatureCorrespondences();
    calculateDepthInformationBetweenCorrespondences(stereo_features);
}

void StereoFeatures::findFeatures( const cv::Mat &leftImage, const cv::Mat &rightImage )
{
    // initialize the calibration structure
    // if the image size has changed
    cv::Size imageSize = leftImage.size();
    if( calib.getImageSize() != imageSize )
    {
	calib.setImageSize( imageSize );
	calib.initCv();
    }

    // this is the right time to set the distance images
    // in the extractor if they are available, then run
    // the findFeatures method
    cv::PSurfDescriptorExtractor *psurf = 
	dynamic_cast<cv::PSurfDescriptorExtractor*>( &(*descriptorExtractor) ); 
    if( dist_left && psurf )
	psurf->setDistanceImage( dist_left );
    findFeatures( leftImage, leftFeatures, true );

    if( dist_right && psurf ) 
	psurf->setDistanceImage( dist_right );
    findFeatures( rightImage, rightFeatures, false );

    if( config.adaptiveDetectorParam )
    {
	size_t lastNumFeatures = 
	    std::min( leftFeatures.keypoints.size(), rightFeatures.keypoints.size() );

	initDetector( lastNumFeatures );
    }

    if( config.debugImage )
    {
	debugRightOffset = leftImage.size().width;
	cv::Size debugSize = 
	    cv::Size(debugRightOffset + rightImage.size().width , leftImage.size().height);

	debugImage.create( debugSize, CV_8UC3 );

	// copy the source images into a single big image
	cv::Mat leftRoi( debugImage, cv::Rect( 0, 0, leftImage.size().width, leftImage.size().height ) );
	cv::cvtColor( 
		leftImage,
		leftRoi,
		CV_GRAY2BGR );

	cv::Mat rightRoi( debugImage, cv::Rect( debugRightOffset, 0, rightImage.size().width, rightImage.size().height ) );  
	cv::cvtColor( 
		rightImage,
		rightRoi,
		CV_GRAY2BGR );
    }
}

/** check if a match for knn > 1 is robust, by making sure, the distance to the
 * next match is further away than the first by a specific factor.
 */
bool robustMatch( std::vector<cv::DMatch>& matches, float distanceFactor = 2.0 )
{
    if( matches.size() >= 2 )
    {
	return matches[0].distance * distanceFactor < matches[1].distance;
    }
    return false;
}

/** 
 * perform cross checking to make sure that the nearest neighbour relationship
 * goes both ways. if a knn of greater than 1 is given, a distanceFactor of
 * greater 1.0 will make the matches more robust, by checking against the next
 * nn.
 */
void StereoFeatures::crossCheckMatching( const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& filteredMatches12, int knn, float distanceFactor )
{
  std::vector<std::vector<cv::DMatch> > matches12, matches21;
  descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
  descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
  crossCheckMatching(matches12, matches21, filteredMatches12, knn, distanceFactor);
}

void StereoFeatures::crossCheckMatching( std::vector<std::vector<cv::DMatch> > matches12, std::vector<std::vector<cv::DMatch> > matches21, std::vector<cv::DMatch>& filteredMatches12, int knn, float distanceFactor)
{
    filteredMatches12.clear();
    for( size_t m = 0; m < matches12.size(); m++ )
    {
	if( knn > 1 && !robustMatch( matches12[m], distanceFactor ) )
	    continue;

	if( !matches12[m].empty() )
	{
	    cv::DMatch &forward = matches12[m][0];

	    if( knn > 1 && !robustMatch( matches21[forward.trainIdx], distanceFactor ) )
		continue;

	    if( !matches21[forward.trainIdx].empty() )
	    {
		cv::DMatch backward = matches21[forward.trainIdx][0];

                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
		}
	    }
	}
    }
}

bool StereoFeatures::getPutativeStereoCorrespondences()
{
    std::vector<cv::DMatch> stereoCorrespondences;
    if(leftFeatures.descriptors.rows < 5 || rightFeatures.descriptors.rows < 5)
    {
	std::cerr << "GetPutativeStereoCorrespondences_Descriptor: At least 5 features are needed left and right, currently " <<
	    leftFeatures.descriptors.rows << " left and " <<
	    rightFeatures.descriptors.rows << " right detected!" <<
	    std::endl;
        return false;
    }

    // check if the matching should be done on the GPU
    if(use_gpu_detector)
    {
            // do stereo feature matching on the GPU.
            cv::gpu::BruteForceMatcher_GPU< cv::L2<float> > gpu_matcher;
            cv::gpu::GpuMat trainIdx, distance, all_dist;
            // do matching on the GPU
            gpu_matcher.knnMatch(descriptors_gpu_left, descriptors_gpu_right, trainIdx, distance, all_dist, 1);
            // download the matches
            std::vector<std::vector<cv::DMatch> > matches12;
            cv::gpu::BruteForceMatcher_GPU< cv::L2<float> >::knnMatchDownload(trainIdx, distance, matches12);
            cv::gpu::GpuMat trainIdx2, distance2, all_dist2;
            gpu_matcher.knnMatch(descriptors_gpu_right, descriptors_gpu_left, trainIdx2, distance2, all_dist2, 1);
            // download the matches
            std::vector<std::vector<cv::DMatch> >matches21;
            cv::gpu::BruteForceMatcher_GPU< cv::L2<float> >::knnMatchDownload(trainIdx2, distance2, matches21);
            // use the gpu generated data for a cross check match
            crossCheckMatching(matches12, matches21, stereoCorrespondences, config.knn, config.distanceFactor);
    }
    else 
    {
        // do good cross check matching
        crossCheckMatching( leftFeatures.descriptors, rightFeatures.descriptors, stereoCorrespondences, config.knn, config.distanceFactor);
    }

    // resize the descriptor matrices
    leftPutativeMatches.descriptors.create( stereoCorrespondences.size(),
	    leftFeatures.descriptors.cols, leftFeatures.descriptors.type());
    rightPutativeMatches.descriptors.create( stereoCorrespondences.size(),
	    rightFeatures.descriptors.cols, rightFeatures.descriptors.type());

    // purge the two old putative matches arrays
    leftPutativeMatches.keypoints.clear();
    rightPutativeMatches.keypoints.clear();

    // extract all matched descriptors and points into the putative matces vector
    for(size_t i = 0; i < stereoCorrespondences.size(); i++ )
    {
        int q_i = stereoCorrespondences.at(i).queryIdx; 
        leftPutativeMatches.keypoints.push_back(leftFeatures.keypoints[q_i]); 
        cv::Mat curRow = leftPutativeMatches.descriptors.row(i);
        leftFeatures.descriptors.row(q_i).copyTo(curRow);

        int t_i = stereoCorrespondences.at(i).trainIdx; 
        rightPutativeMatches.keypoints.push_back(rightFeatures.keypoints[t_i]); 
        curRow = rightPutativeMatches.descriptors.row(i);
        rightFeatures.descriptors.row(t_i).copyTo(curRow);
    }

    return true;
}

bool StereoFeatures::refineFeatureCorrespondences()
{
    // extract the 2d points from the keypoint lists
    vector<cv::Point2f> points1, points2;
    cv::KeyPoint::convert(leftPutativeMatches.keypoints, points1);
    cv::KeyPoint::convert(rightPutativeMatches.keypoints, points2);

    vector<uchar> matchesMask;
    // the number of correctly matched points will be contained in this integer
    int numberOfGood = 0;

    // check if the constraints by the filter have been met
    bool runDefault = false;

    // select the type of filtering
    switch( config.filterType )
    {
	case FILTER_HOMOGRAPHY:
        {
            // check if there are enough points for homography extraction
            if(leftPutativeMatches.keypoints.size() < 4 || rightPutativeMatches.keypoints.size() < 4)	
            {
                cout << "RefineFeatureCorrespondences(HOMOGRAPHY): " 
		     << "At least 4 features are needed left and right, currently " 
		     << leftPutativeMatches.keypoints.size() << "left and " 
		     << rightPutativeMatches.keypoints.size()
		     << " right detected!" << endl;
		runDefault = true;
                break;
            }
            // use the two point lists to find the homography
            cv::Mat H12 = findHomography( cv::Mat(points1), cv::Mat(points2), CV_RANSAC, 1.0 );
        
            // create the mask: transform the left points using the homography, and compare the result with the right points. If that is equal (or very near) it is an inlier.
            cv::Mat transformed_left_points;
            // create the mask list, which contains the inliers
            matchesMask = vector<uchar>( leftPutativeMatches.keypoints.size(), 0 );
    
            perspectiveTransform(cv::Mat(points1), transformed_left_points, H12);
            for(size_t i = 0; i < leftPutativeMatches.keypoints.size(); i++ )
            {
                if(norm(rightPutativeMatches.keypoints[i].pt - transformed_left_points.at<cv::Point2f>(i,0)) < 4 ) // inlier
                {
                    matchesMask[i] = 1;
                    numberOfGood++;
                }
            }
            break;
        }
        case FILTER_FUNDAMENTAL:
        {
            // check if there are enough points for fundamental matrix extraction
            if(leftPutativeMatches.keypoints.size() < 8 || rightPutativeMatches.keypoints.size() < 8)	
            {
                cout << "RefineFeatureCorrespondences(FUNDAMENTAL): At least 8 features are needed left and right, currently " 
		    << leftPutativeMatches.keypoints.size() << " left and " 
		    << rightPutativeMatches.keypoints.size() << " right detected!" << endl;
		runDefault = true;
                break;
            }
            // use the two point lists to find the fundamental matrix
            cv::Mat fund = findFundamentalMat( cv::Mat(points1), cv::Mat(points2), matchesMask, CV_FM_RANSAC, 1.0, 0.99 );

            // determine how many good features were detected
            for(size_t i = 0; i < matchesMask.size(); i++)
                if(matchesMask[i] == 1)
                    numberOfGood++;
            break;
        }
        case FILTER_STEREO:
            // just check, if the epipolar geometry is maintained for each match
            matchesMask = vector<uchar>( leftPutativeMatches.keypoints.size(), 0 );
            for(size_t i = 0; i < matchesMask.size(); i++)
            {
		const double ydev = fabs( leftPutativeMatches.keypoints[i].pt.y - rightPutativeMatches.keypoints[i].pt.y );
		const double disparity = leftPutativeMatches.keypoints[i].pt.x - rightPutativeMatches.keypoints[i].pt.x;
                if( ydev < config.maxStereoYDeviation && disparity > 0 )
                {
                    matchesMask[i] = 1;
                    numberOfGood++;
                }
            }
            break;
        case FILTER_NONE:
	    runDefault = true;
            break;
        default:
            cout << "RefineFeatureCorrespondences: (Warn) unrecognized filter method selected, no filtering applied!" << endl;
	    runDefault = true;
    }

    if( runDefault )
    {
        // no filter selected, make all matches positive.
        numberOfGood = leftPutativeMatches.keypoints.size();
        matchesMask = vector<uchar>( leftPutativeMatches.keypoints.size(), 1 );
    }

    assert( matchesMask.size() == leftPutativeMatches.keypoints.size() );

    // resize the descriptor matrices
    leftMatches.descriptors.create(numberOfGood, leftPutativeMatches.descriptors.cols, leftPutativeMatches.descriptors.type());
    rightMatches.descriptors.create(numberOfGood, rightPutativeMatches.descriptors.cols, rightPutativeMatches.descriptors.type());

    // purge the two old matches arrays
    leftMatches.keypoints.clear();
    rightMatches.keypoints.clear();

    int counter = 0;
    // extract all matched putative descriptors and points into the matces vector
    for(size_t i = 0; i < leftPutativeMatches.keypoints.size(); i++ )
    {
        if(matchesMask[i] == 1)
        {
            leftMatches.keypoints.push_back(leftPutativeMatches.keypoints[i]); 
            cv::Mat curRow = leftMatches.descriptors.row(counter);
            leftPutativeMatches.descriptors.row(i).copyTo(curRow);
            rightMatches.keypoints.push_back(rightPutativeMatches.keypoints[i]); 
            curRow = rightMatches.descriptors.row(counter);
            rightPutativeMatches.descriptors.row(i).copyTo(curRow);
            counter++;
        }
    }

    if( config.debugImage )
    {
	// draw left inter-frame correspondences
	cv::Scalar color = cv::Scalar(0, 255, 0);
	int width = 1;
	for(size_t i = 0; i < leftMatches.keypoints.size() && i < rightMatches.keypoints.size(); i++ )
	{
	    cv::Point center1, center2;
	    center1 = leftMatches.keypoints[i].pt;
	    center2 = rightMatches.keypoints[i].pt;
	    center2.x += debugRightOffset;
	    cv::line( debugImage, center1, center2, color, width);

	    int lradius = cvRound(leftMatches.keypoints[i].size*1.2/9.*2);
	    cv::circle( debugImage, center1, lradius + 2, cvScalar(0, 0, 255), 1, 8, 0 );

	    int rradius = cvRound(rightMatches.keypoints[i].size*1.2/9.*2);
	    cv::circle( debugImage, center2, rradius + 2, cvScalar(0, 0, 255), 1, 8, 0 );
	}
    }

//    std::cout << "Number of refined stereo matches: " << counter << std::endl;
    return !runDefault;
}

void StereoFeatures::calculateDepthInformationBetweenCorrespondences(StereoFeatureArray *stereo_features)
{
    // create a pointer to the stereo-feature memory which should be used for this function
    StereoFeatureArray *stereo_feature_pointer = &stereoFeatures;

    if(stereo_features)
      stereo_feature_pointer = stereo_features;

    stereo_feature_pointer->clear();

    // currently we always use the surf descriptor (might change)
    stereo_feature_pointer->descriptorType = envire::DESCRIPTOR_SURF;

    // get Q Projection Matrix as Eigen
    Eigen::Matrix4d Q;
    cv2eigen( calib.Q, Q );
    
    stereo_feature_pointer->mean_z_value = 0;

    // loop through all features available
    for(size_t i = 0; i < leftMatches.keypoints.size(); i++)
    {
        //ok, we found a match. put all the necessary data into the new data structure
        Eigen::Vector4d v;
        // build the 3d point
        v[0] = leftMatches.keypoints[i].pt.x;
        v[1] = leftMatches.keypoints[i].pt.y;
        v[2] = rightMatches.keypoints[i].pt.x - leftMatches.keypoints[i].pt.x; // disparity
	v[3] = 1.0;

	// perform projection to 3d space
	// and change to meters instead of mm
	Eigen::Vector4d vh = Q * v;
	vh *= .001/vh[3];

	// TODO for the time being take only left keypoints. However, it might
	// be better to take the keypoint with the strongest response
	envire::KeyPoint kp;
	// calculate the keypointSize from the calibration matrix's fx parameter 
	//  correct for unit and scale by distance (z.value)
	const double keypointSize = leftMatches.keypoints[i].size 
	    / calib.camLeft.camMatrix.at<double>(0,0) * vh[2];
	kp.size = keypointSize;
	kp.angle = leftMatches.keypoints[i].angle;
	kp.response = leftMatches.keypoints[i].response;
	kp.point = cv2eigen( leftMatches.keypoints[i].pt );

	stereo_feature_pointer->push_back( 
		vh.head<3>(), kp, 
		Eigen::Map<StereoFeatureArray::Descriptor>( 
		    leftMatches.descriptors.ptr<float>(i), leftMatches.descriptors.cols ) );
        // keep a running average of the mean z position
        stereo_feature_pointer->mean_z_value += v[2];
    }
    if(leftMatches.keypoints.size() > 0)
      stereo_feature_pointer->mean_z_value /= (double)(leftMatches.keypoints.size());
std::cout << "****************************************** mean_z: " << stereo_feature_pointer->mean_z_value  / -100.0 << "m" << std::endl;
}

void StereoFeatures::calculateInterFrameCorrespondences( const envire::Featurecloud* fc1, const envire::Featurecloud* fc2, int filterMethod )
{
    // get features from array
    const cv::Mat feat1 = cv::Mat( fc1->size(), fc1->descriptorSize, cv::DataType<float>::type, const_cast<float*>(&fc1->descriptors[0])); 
    const cv::Mat feat2 = cv::Mat( fc2->size(), fc2->descriptorSize, cv::DataType<float>::type, const_cast<float*>(&fc2->descriptors[0])); 

    calculateInterFrameCorrespondences( feat1, fc1->keypoints, fc1->vertices, feat2, fc2->keypoints, fc2->vertices, filterMethod );
}

void StereoFeatures::calculateInterFrameCorrespondences( const StereoFeatureArray& frame1, const StereoFeatureArray& frame2, int filterMethod )
{
    // get features as cv::Mat from arrays
    // need to const cast here, as opencv doesn't provide a way to supply a const void *
    const cv::Mat feat1 = 
	cv::Mat( frame1.size(), frame1.descriptorSize, cv::DataType<float>::type, const_cast<float*>(&frame1.descriptors[0]) ); 
    const cv::Mat feat2 = 
	cv::Mat( frame2.size(), frame2.descriptorSize, cv::DataType<float>::type, const_cast<float*>(&frame2.descriptors[0]) ); 

    std::vector<Eigen::Vector3d> p1, p2;
    std::copy( frame1.points.begin(), frame1.points.end(), std::back_inserter( p1 ) );
    std::copy( frame2.points.begin(), frame2.points.end(), std::back_inserter( p2 ) );

    calculateInterFrameCorrespondences( feat1, frame1.keypoints, p1, feat2, frame2.keypoints, p2, filterMethod );
}

void StereoFeatures::calculateInterFrameCorrespondences( 
	const cv::Mat& feat1, const std::vector<envire::KeyPoint>& keyp1, const std::vector<Eigen::Vector3d>& points1,
	const cv::Mat& feat2, const std::vector<envire::KeyPoint>& keyp2, const std::vector<Eigen::Vector3d>& points2, 
	int filterMethod )
{
    int numberOfGood = 0;
    std::vector<cv::DMatch> leftCorrespondences;
    std::vector<uchar> matches_mask;

    const int minFeatures = 5;
    if( feat1.rows < minFeatures || feat2.rows < minFeatures )
    {
	// we cannot do matching, so simply add all points and be done. This is
	// done by setting filterMethod = FILTER_NONE and leaving
	// leftCorrespondences empty.
        cout << "CalculateInterFrameCorrespondences: (Error) At least 5 features are needed in both frames, currently "
	    << feat1.rows << " last and " << feat2.rows << " current detected!" << endl;
        filterMethod = FILTER_NONE;
    }
    else
    {
        // do cross check matching and pre filtering of features
	crossCheckMatching( feat1, feat2, 
		leftCorrespondences, config.knn, config.distanceFactor );

	// match the features by size
	// TODO do properly
	std::vector<cv::DMatch> leftCorrespondences2 = leftCorrespondences;
	leftCorrespondences.clear();
	for( size_t i = 0; i < leftCorrespondences2.size(); i ++ )
	{
	    double size_diff = fabs( keyp1[ leftCorrespondences2[i].queryIdx ].size - 
		    keyp2[ leftCorrespondences2[i].trainIdx ].size ); 

	    if( size_diff < 0.5 )
		leftCorrespondences.push_back( leftCorrespondences2[i] );
	}
    }

    // do filtering on the point lists. select one of the filters:
    switch(filterMethod)
    {
	case FILTER_ISOMETRY:
	{
	    // find an isometry transformation between the 3d points 
	    // using ransac
	    Eigen::Affine3d best_model;
	    std::vector<size_t> best_inliers;
	    const double DIST_THRESHOLD = config.isometryFilterThreshold;

	    std::vector<float> e1, e2;
	    std::vector<Eigen::Vector3d> x, p;
	    for( size_t i = 0; i < leftCorrespondences.size(); i++ )
	    {
		const Eigen::Vector3d &v1( points1[leftCorrespondences[i].queryIdx]  );
		const Eigen::Vector3d &v2( points2[leftCorrespondences[i].trainIdx]  );
		//const double max_dist = 15.0;
		//if( v1.norm() < max_dist && v2.norm() < max_dist )
		{
		    x.push_back( v1 );
		    p.push_back( v2 );
		}

		{
		    const float dist_factor = 1/70.0;
		    e1.push_back( v1.norm() * dist_factor );
		    e2.push_back( v2.norm() * dist_factor );
		}
	    }

	    if( x.size() >= 3 )
	    {
		envire::ransac::FitTransformUncertain fit( x, p, e1, e2, DIST_THRESHOLD );
		envire::ransac::ransacSingleModel( fit, 3, DIST_THRESHOLD, best_model, best_inliers, config.isometryFilterMaxSteps );

		correspondenceTransform = best_model;
	    }

	    matches_mask = vector<uchar>( leftCorrespondences.size(), 0 );
	    for( size_t i=0; i<best_inliers.size(); i++ )
	    {
		matches_mask[best_inliers[i]] = 1;
		numberOfGood++;
	    }
	}
	    break;
        case FILTER_HOMOGRAPHY:
        case FILTER_FUNDAMENTAL:
        {
            // extract the 2d points from the keypoint lists
            vector<cv::Point2f> points1, points2;
            for(size_t i = 0; i < leftCorrespondences.size(); i++ )
            {
		const base::Vector2d &v1( keyp1[ leftCorrespondences.at(i).queryIdx ].point );
		points1.push_back( eigen2cv( v1 ) );

		const base::Vector2d &v2( keyp2[ leftCorrespondences.at(i).trainIdx ].point );
		points2.push_back( eigen2cv( v2 ) );
            }

            if(filterMethod == FILTER_HOMOGRAPHY)
            {
                // check if there are enough points for homography extraction
		const size_t minPoints = 4;
                if( points1.size() < minPoints || points2.size() < minPoints )	
                {
                    cout << "CalculateInterFrameCorrespondences(FILTER_HOMOGRAPHY): At least 4 features are needed in this and the last frame, currently " 
			<< points1.size() << " in this frame and " << points2.size() << " in the last frame detected!" << endl;
                    filterMethod = FILTER_NONE;
                    break;
                }
                // use the two point lists to find the homography
		homography = findHomography( cv::Mat(points1), cv::Mat(points2), CV_RANSAC, 1.0 );
                matches_mask = vector<uchar>( points1.size(), 0 );
                // create the mask: transform the current frame points using the homography, and compare the result with the last frame points. If that is equal (or very near) it is an inlier.
		cv::Mat transformed_current_points;
                // create the mask list, which contains the inliers
                perspectiveTransform( cv::Mat(points1), transformed_current_points, homography );
                for(int i = points1.size() -1; i >= 0; i-- )
                {
		    const float inlierRadius = 4.0;
                    if( norm(points2[i] - transformed_current_points.at<cv::Point2f>(i,0)) < inlierRadius ) // inlier 
                    {
                        matches_mask[i] = 1;
                        numberOfGood++;
                    }
                }
            }// end of filterMethod == HOMOGRAPHY
            else
            {// filterMethod == FUNDAMENTAL
                // check if there are enough points for fundamental matrix calculation
                if(points1.size() < 8 || points2.size() < 8)	
                {
                    cout << "CalculateInterFrameCorrespondences(FILTER_FUNDAMENTAL): At least 8 features are needed in this and the last frame, currently " 
			<< points1.size() << " in this frame and " << points2.size() << " in the last frame detected!" << endl;
                    filterMethod = FILTER_NONE;
                    break;
                }
                matches_mask.clear();
                // use the two point lists to find the fundamental matrix
		const float outlierDistance = 3.0;
		const float accuracy = 0.99;
		cv::Mat fund = findFundamentalMat( cv::Mat(points1), cv::Mat(points2), matches_mask, CV_FM_RANSAC, outlierDistance, accuracy );

                // determine how many good features were detected
                for( size_t i = 0; i < matches_mask.size(); i++)
                    if(matches_mask[i] == 1)
                        numberOfGood++;
            }// end of filterMethod == FUNDAMENTAL
        }
            break;
        case FILTER_INTELLIGENT:
            matches_mask = vector<uchar>( leftCorrespondences.size(), 0 );
            break;
        default:
            cout << "CalculateInterFrameCorrespondences: (Warn) unrecognized filter method selected, no filtering applied!" << endl;
            filterMethod = FILTER_NONE;
        case FILTER_NONE:
            break;
    }
    if(filterMethod == FILTER_NONE)
    {
        matches_mask = vector<uchar>( leftCorrespondences.size(), 1 );
    }

    // record the correspondences between the two frames
    assert( matches_mask.size() == leftCorrespondences.size() );
    correspondences.clear();

    for( size_t i = 0; i < matches_mask.size(); i++ )
    {
	if( matches_mask[i] )
	    correspondences.push_back( make_pair(
			leftCorrespondences.at(i).queryIdx,
			leftCorrespondences.at(i).trainIdx ) );
    }

//    cout << "Number of detected Features: " << keyp1.size() << " Number of putative inter-frame matches: " 
//	<< leftCorrespondences.size() << " number of filtered inter-frame matches: " << correspondences.size() << endl;

    return;
}

cv::Mat StereoFeatures::getInterFrameDebugImage( const cv::Mat& debug1, const StereoFeatureArray& frame1, const cv::Mat& debug2, const StereoFeatureArray& frame2, std::vector<std::pair<long,long> > *correspondence )
{
    // throw warning message if used incorrectly
    if(debug1.channels() < 3 || debug2.channels() < 3)
      std::cout << "Warning: getInterFrameDebugImage expects 3-channel images, otherwise it will not work properly." << std::endl;

    // create the debug image
    cv::Size debugSize = 
	cv::Size( debug1.size().width, debug1.size().height + debug2.size().height );

    cv::Mat debugImage;
    debugImage.create( debugSize, CV_8UC3 );

    const int debugTopOffset = debug1.size().height;

    cv::Mat upperRoi( debugImage, cv::Rect( 0, 0, debug1.size().width, debug1.size().height ) );
    debug1.copyTo( upperRoi );

    cv::Mat lowerRoi( debugImage, cv::Rect( 0, debugTopOffset, debug2.size().width, debug2.size().height ) );
    debug2.copyTo( lowerRoi );

    const cv::Scalar color = cv::Scalar(255, 0, 0);
    const int width = 1;
    std::vector<std::pair<long,long> > *corr = &correspondences;
    if(correspondence)
      corr = correspondence;
 
    for( size_t i = 0; i < corr->size(); i++ )
    {
	cv::Point center1, center2;
	center1 = eigen2cv( frame1.keypoints[ (*corr)[i].first ].point );
	center2 = eigen2cv( frame2.keypoints[ (*corr)[i].second ].point );

	center2.y += debugTopOffset;
	cv::line( debugImage, center1, center2, color, width);

	// for testing
	center1.x += debugRightOffset;
	center2.x += debugRightOffset;
	center2.y -= debugTopOffset;
	cv::line( debugImage, center1, center2, color, width);
    }

    return debugImage;
}
