#define BOOST_TEST_MODULE StereoTest 
#include <boost/test/included/unit_test.hpp>

#include <frame_helper/CalibrationCv.h>
#include <stereo/sparse_stereo.hpp>
#include <stereo/densestereo.h>

#include "opencv2/highgui/highgui.hpp"

const std::string prefix = "test/";
const std::string prefix_out = "build/test/";

frame_helper::StereoCalibration getTestCalibration( const std::string& name )
{
    return frame_helper::StereoCalibration::fromMatlabFile( prefix + "calib" + name + ".txt" );
}

void getTestImages( const std::string& name, cv::Mat& left, cv::Mat& right )
{
    // load the test images and perform conversion
    // to grayscale and undistort them
    cv::Mat cleft = cv::imread( prefix + "left" + name + ".png" );
    cv::Mat cright = cv::imread( prefix + "right" + name + ".png" );
    cv::Mat gleft, gright;

    cv::cvtColor( cleft, gleft, CV_BGR2GRAY );
    cv::cvtColor( cright, gright, CV_BGR2GRAY );

    frame_helper::StereoCalibrationCv calib;
    calib.setCalibration( getTestCalibration( name ) );
    calib.setImageSize( cleft.size() );
    calib.initCv();

    calib.camLeft.undistortAndRectify( gleft, left );
    calib.camRight.undistortAndRectify( gright, right );
}

BOOST_AUTO_TEST_CASE( dense_test ) 
{
    // read input images
    cv::Mat left, right;
    getTestImages( "", left, right );

    // setup dense stereo object
    stereo::DenseStereo dense;

    // update calibration
    dense.setStereoCalibration( getTestCalibration(""), left.size().width, left.size().height );

    // process left/right frame and get disparity images
    cv::Mat ldisp, rdisp;
    dense.processFramePair( left, right, ldisp, rdisp, true );

    cv::imwrite( prefix_out + "ldisp.png", ldisp );
    cv::imwrite( prefix_out + "rdisp.png", rdisp );

    // convert to distance images
    dense.getDistanceImages( ldisp, rdisp );

    cv::imwrite( prefix_out + "ldist.png", ldisp );
    cv::imwrite( prefix_out + "rdist.png", rdisp );
}


BOOST_AUTO_TEST_CASE( psurf_test ) 
{
    // read input images
    cv::Mat left, right;
    getTestImages( "1", left, right );

    // setup dense stereo object and init calibration
    stereo::DenseStereo dense;
    dense.setStereoCalibration( getTestCalibration("1"), left.size().width, left.size().height );

    // process left/right frame and get disparity images
    cv::Mat ldist, rdist;
    dense.getDistanceImages( left, right, ldist, rdist, false );

    // setup sparse stereo
    stereo::StereoFeatures sparse;
    sparse.setCalibration( getTestCalibration("1") );

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
    cv::imwrite( prefix_out + "sparse.png", sparse.getDebugImage() );
}
