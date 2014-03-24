#include <frame_helper/CalibrationCv.h>
#include <stereo/densestereo.h>

#include "opencv2/highgui/highgui.hpp"

int main( int argc, char* argv[] )
{
    if( argc < 5 )
    {
	std::cout << "usage: undistort leftimage rightimage calibration_file result" << std::endl;
	exit(0);
    }

    cv::Mat cleft = cv::imread( argv[1] );
    cv::Mat cright = cv::imread( argv[2] );

    assert( cleft.size() == cright.size() );
    
    frame_helper::StereoCalibrationCv calib;
    calib.setCalibration( frame_helper::StereoCalibration::fromMatlabFile( argv[3], cleft.size().width, cleft.size().height) );
    calib.setImageSize( cleft.size() );
    calib.initCv();

    cv::Mat result( cleft.size().height, cleft.size().width * 2, cleft.type() );
    cv::Mat rleft( result, cv::Rect( 0, 0, cleft.size().width, cleft.size().height ) );
    cv::Mat rright( result, cv::Rect( cleft.size().width, 0, cleft.size().width, cleft.size().height ) );

    calib.camLeft.undistortAndRectify( cleft, rleft );
    calib.camRight.undistortAndRectify( cright, rright );

    cv::imwrite( argv[4], result );
}
