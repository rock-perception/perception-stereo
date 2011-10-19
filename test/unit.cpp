#define BOOST_TEST_MODULE StereoTest 
#include <boost/test/included/unit_test.hpp>

#include <frame_helper/CalibrationCv.h>
#include <stereo/sparse_stereo.hpp>
#include <stereo/densestereo.h>

#include "opencv2/highgui/highgui.hpp"

frame_helper::StereoCalibration getTestCalibration()
{
    frame_helper::StereoCalibration calib = 
    {
	// calibration values for test images 
	// this is not meant to be the way to configure a calibration
	// rather save the calibration in a file and load it dynamically 
	// currently this works on the ruby level (e.g. see the orogen module scripts dir)
	{280.88145,   281.69324,  320.14464,   233.86583,  -0.00601,   0.00475,   0.00069,   0.00017},
	{284.24943,   285.02469,  320.04661,   233.68118,  -0.00492,   0.00369,   0.00033,   0.00083},
	{-251.39129,   -0.46770,  -3.91264,   -0.00439,   -0.00603,  0.01186 }
    };

    return calib;
}

void getTestImages( cv::Mat& left, cv::Mat& right )
{
    // load the test images and perform conversion
    // to grayscale and undistort them
    cv::Mat cleft = cv::imread( "test/left.png" );
    cv::Mat cright = cv::imread( "test/right.png" );
    cv::Mat gleft, gright;

    cv::cvtColor( cleft, gleft, CV_BGR2GRAY );
    cv::cvtColor( cright, gright, CV_BGR2GRAY );

    frame_helper::StereoCalibrationCv calib;
    calib.setCalibration( getTestCalibration() );
    calib.setImageSize( cleft.size() );
    calib.initCv();

    calib.camLeft.undistortAndRectify( gleft, left );
    calib.camRight.undistortAndRectify( gright, right );
}

BOOST_AUTO_TEST_CASE( dense_test ) 
{
    // read input images
    cv::Mat left, right;
    getTestImages( left, right );

    // setup dense stereo object
    stereo::DenseStereo dense;

    // update calibration
    dense.setStereoCalibration( getTestCalibration(), left.size().width, left.size().height );

    // process left/right frame and get disparity images
    cv::Mat ldisp, rdisp;
    dense.processFramePair( left, right, ldisp, rdisp, true );

    cv::imwrite( "build/test/ldisp.png", ldisp );
    cv::imwrite( "build/test/rdisp.png", rdisp );

    // convert to distance images
    dense.getDistanceImages( ldisp, rdisp );

    cv::imwrite( "build/test/ldist.png", ldisp );
    cv::imwrite( "build/test/rdist.png", rdisp );
}


BOOST_AUTO_TEST_CASE( psurf_test ) 
{
    // read input images
    cv::Mat left, right;
    getTestImages( left, right );

    // setup dense stereo object and init calibration
    stereo::DenseStereo dense;
    dense.setStereoCalibration( getTestCalibration(), left.size().width, left.size().height );

    // process left/right frame and get disparity images
    cv::Mat ldist, rdist;
    dense.getDistanceImages( left, right, ldist, rdist, false );

    // setup sparse stereo
    stereo::StereoFeatures sparse;
    sparse.setCalibration( getTestCalibration() );

    // setup configuration
    stereo::FeatureConfiguration sparseConfig;
    sparseConfig.filterType = stereo::FILTER_STEREO;
    sparseConfig.knn = 2;
    sparseConfig.distanceFactor = 1.6;
    sparseConfig.maxStereoYDeviation = 3;
    sparse.setConfiguration( sparseConfig );

    // perform sparse processing
    sparse.processFramePair( left, right );

    // write sparse stereo debug image 
    cv::imwrite( "build/test/sparse.png", sparse.getDebugImage() );
}
