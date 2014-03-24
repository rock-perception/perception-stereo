#include <frame_helper/CalibrationCv.h>
#include <stereo/densestereo.h>

#include "opencv2/highgui/highgui.hpp"
#include <boost/lexical_cast.hpp>

int main( int argc, char* argv[] )
{
    if( argc < 5 )
    {
	std::cout << "usage: dense_stereo leftimage rightimage calibration_file result <gaussian_blur>" << std::endl;
	exit(0);
    }

    cv::Mat cleft = cv::imread( argv[1] );
    cv::Mat cright = cv::imread( argv[2] );

    std::string prefix_out = argv[4];

    int gaussian_kernel = 0;
    if( argc > 5 )
	gaussian_kernel = boost::lexical_cast<int>( argv[5] );

    assert( cleft.size() == cright.size() );

    frame_helper::StereoCalibrationCv calib;
    calib.setCalibration( frame_helper::StereoCalibration::fromMatlabFile( argv[3], cleft.size().width, cleft.size().height) );
    calib.setImageSize( cleft.size() );
    calib.initCv();

    // setup dense stereo object
    stereo::DenseStereo dense;

    if( gaussian_kernel )
	dense.setGaussianKernel( gaussian_kernel );

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
