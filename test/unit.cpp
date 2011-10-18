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
	{280.88145,   281.69324,  320.14464,   233.86583,  -0.00601,   0.00475,   0.00069,   0.00017},
	{284.24943,   285.02469,  320.04661,   233.68118,  -0.00492,   0.00369,   0.00033,   0.00083},
	{-251.39129,   -0.46770,  -3.91264,   -0.00439,   -0.00603,  0.01186 }
    };

    return calib;
}


BOOST_AUTO_TEST_CASE( dense_test ) 
{
    stereo::DenseStereo dense;

    cv::Mat left = cv::imread( "test/left.png" );
    cv::Mat right = cv::imread( "test/right.png" );

    dense.setStereoCalibration( getTestCalibration(), left.size().width, left.size().height );

    cv::Mat ldisp, rdisp;

    dense.processFramePair( left, right, ldisp, rdisp, false );

    cv::imwrite( "build/test/ldisp.png", ldisp );
    cv::imwrite( "build/test/rdisp.png", rdisp );
}
