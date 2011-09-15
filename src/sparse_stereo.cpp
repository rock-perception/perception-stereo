#include "sparse_stereo.hpp"
#include <Eigen/Core>
#include <iostream>
#include <opencv2/core/eigen.hpp>

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
{
    descriptorExtractor = new cv::SurfDescriptorExtractor(4, 3, false);
    descriptorMatcher = cv::DescriptorMatcher::create("FlannBased");
    initDetector( config.targetNumFeatures );
}

void StereoFeatures::setCalibration( const frame_helper::StereoCalibration &calib )
{
    this->calib.setCalibration( calib );
}

void StereoFeatures::setConfiguration( const FeatureConfiguration &config )
{
    this->config = config;
    initDetector( config.targetNumFeatures );
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
	    }
	    break;
	case DETECTOR_SURF:
	    {
		int &SURFparam = detectorParams.SURFparam;
		// adaptively adjust the parameters for the SURF extractor in order to get around TARGET_NUM_FEATURES features
		//            double surfParamDiff = (localLastNumFeatures - targetNumFeatures);
		double surfParamDiff = (double)localLastNumFeatures / (double)targetNumFeatures;

		SURFparam = (int)((double)SURFparam * surfParamDiff);

		// to prevent the value from running haywire, cap it
		if(SURFparam < 3)
		    SURFparam = 3;
		if(SURFparam > 5500)
		    SURFparam = 5500;

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
		detector = new cv::MserFeatureDetector(mserParam, 5, 500, 1.0, 0.5, 1, 1.0, 0.0, 1);
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
	default: 
	    throw std::runtime_error("Selected feature detector is not implemented.");
    }
}

void StereoFeatures::findFeatures( const cv::Mat &image, FeatureInfo& info )
{
    clock_t start, finish;

    start = clock();
    detector->detect( image, info.keypoints);
    finish = clock();
    info.detectorTime = base::Time::fromSeconds( (finish - start) / (CLOCKS_PER_SEC * 1.0) );

    start = clock();
    descriptorExtractor->compute( image, info.keypoints, info.descriptors );
    finish = clock();
    info.descriptorTime = base::Time::fromSeconds( (finish - start) / (CLOCKS_PER_SEC * 1.0) );
}

void StereoFeatures::processFramePair( const cv::Mat &left_image, const cv::Mat &right_image )
{
    findFeatures( left_image, right_image );
    getPutativeStereoCorrespondences();
    refineFeatureCorrespondences();
    calculateDepthInformationBetweenCorrespondences();
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

    findFeatures( leftImage, leftFeatures );
    findFeatures( rightImage, rightFeatures );

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
    filteredMatches12.clear();

    std::vector<std::vector<cv::DMatch> > matches12, matches21;
    descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
    descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );

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
    if(leftFeatures.descriptors.rows < 5 || rightFeatures.descriptors.rows < 5)
    {
	std::cerr << "GetPutativeStereoCorrespondences_Descriptor: At least 5 features are needed left and right, currently " <<
	    leftFeatures.descriptors.rows << " left and " <<
	    rightFeatures.descriptors.rows << " right detected!" <<
	    std::endl;
        return false;
    }

    std::vector<cv::DMatch> stereoCorrespondences;
    // do good cross check matching
    crossCheckMatching( leftFeatures.descriptors, rightFeatures.descriptors, 
	    stereoCorrespondences, config.knn, config.distanceFactor );

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

    //cout << "Number of refined stereo matches: " << counter << endl;
    return !runDefault;
}

void StereoFeatures::calculateDepthInformationBetweenCorrespondences()
{
    stereoFeatures.clear();

    // currently we always use the surf descriptor (might change)
    stereoFeatures.descriptorType = envire::DESCRIPTOR_SURF;

    // get Q Projection Matrix as Eigen
    Eigen::Matrix4d Q;
    cv2eigen( calib.Q, Q );

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

	stereoFeatures.push_back( 
		vh.head<3>(), kp, 
		Eigen::Map<StereoFeatureArray::Descriptor>( 
		    leftMatches.descriptors.ptr<float>(i), leftMatches.descriptors.cols ) );
    }
}

