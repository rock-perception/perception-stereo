#include <frame_helper/CalibrationCv.h>
#include <stereo/densestereo.h>

#include "opencv2/highgui/highgui.hpp"

int main( int argc, char* argv[] )
{
    if( argc < 5 )
    {
	std::cout << "usage: dense_stereo leftimage rightimage calibration_file result" << std::endl;
	exit(0);
    }

    cv::Mat cleft = cv::imread( argv[1] );
    cv::Mat cright = cv::imread( argv[2] );

    std::string prefix_out = argv[4];

    assert( cleft.size() == cright.size() );

    frame_helper::StereoCalibrationCv calib;
    calib.setCalibration( frame_helper::StereoCalibration::fromMatlabFile( argv[3]) );
    calib.setImageSize( cleft.size() );
    calib.initCv();

    // prefilter images
    cv::GaussianBlur( cleft, cleft, cv::Size( 9, 9 ), 0 );
    cv::GaussianBlur( cright, cright, cv::Size( 9, 9 ), 0 );

    cv::imwrite( prefix_out + "cleft.png", cleft );
    cv::imwrite( prefix_out + "cright.png", cright );
    
    // setup dense stereo object
    stereo::DenseStereo dense;

    // update calibration
    dense.setStereoCalibration( calib.getCalibration(), cleft.size().width, cleft.size().height );

    // process left/right frame and get disparity images
    cv::Mat ldisp, rdisp;
    dense.processFramePair( cleft, cright, ldisp, rdisp, true );

    cv::imwrite( prefix_out + "ldisp.png", ldisp );
    cv::imwrite( prefix_out + "rdisp.png", rdisp );

    // convert to distance images
    dense.getDistanceImages( ldisp, rdisp );

    cv::imwrite( prefix_out + "ldist.png", ldisp );
    cv::imwrite( prefix_out + "rdist.png", rdisp );
}
