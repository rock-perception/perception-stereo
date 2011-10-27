#define BOOST_TEST_MODULE StereoTest 
#include <boost/test/included/unit_test.hpp>

#include <frame_helper/CalibrationCv.h>
#include <stereo/sparse_stereo.hpp>
#include <stereo/densestereo.h>
#include <stereo/homography.h>

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

void testHomography( const base::samples::DistanceImage& dimage, cv::Mat& image )
{
    // pick some test points in the image, calculate the homography and
    // project the result

    // copy input image as debug image
    cv::Mat dbgImg;
    cv::cvtColor( image, dbgImg, CV_GRAY2BGR );
    
    const int step = 50;
    for( int x=0; x<image.size().width; x += step )
    {
	for( int y=0; y<image.size().height; y += step )
	{
	    const float radius = 15.0;

	    // initialize homography class and set debug image
	    stereo::Homography homography;
	    homography.dbgImg = dbgImg;

	    if( homography.estimateFromDistanceImage( dimage, x, y, radius ) )
	    {
		// draw center point
		cv::circle( dbgImg, cv::Point( x, y ), 1, cv::Scalar( 255, 0, 0 ) );

		const int window = radius*2;
		const int step = 5;
		for( int m=-window/2; m<window/2; m += step )
		{
		    for( int n=-window/2; n<window/2; n+= step )
		    {
			// transform to scene point
			Eigen::Vector3f p;
			dimage.getScenePoint( x+m, y+n, p );
			//std::cout << "scene: " << p.transpose() << std::endl;

			//homography = Eigen::Matrix3f::Identity();

			// apply homography
			Eigen::Vector3f hp = homography.getTransform() * p;
			//std::cout << "hscene: " << hp.transpose() << std::endl;

			size_t hx, hy;
			// transform to image point
			if( dimage.getImagePoint( hp, hx, hy ) )
			{
			    //std::cout << "image point: " << hx << " " << hy << std::endl;
			    // paint
			    cv::circle( dbgImg, cv::Point( hx, hy ), 0, cv::Scalar( 0, 255, 0 ) );
			}
		    }
		}

		// test reprojection
		cv::Mat repro;
		homography.reproject( dbgImg, repro, x, y, radius );
		cv::Mat wnd( dbgImg, cv::Rect( x-radius, y-radius, radius * 2, radius * 2 ));
		repro.copyTo( wnd );
	    }
	}
    }

    cv::imwrite( prefix_out + "homography.png", dbgImg );
}


BOOST_AUTO_TEST_CASE( psurf_test ) 
{
    const std::string test = "";
    // read input images
    cv::Mat left, right;
    getTestImages( test, left, right );

    // setup dense stereo object and init calibration
    stereo::DenseStereo dense;
    dense.setStereoCalibration( getTestCalibration(test), left.size().width, left.size().height );

    // process left/right frame and get disparity images
    base::samples::DistanceImage ldist, rdist;
    dense.getDistanceImages( left, right, ldist, rdist, true );

    // test homography
    testHomography( ldist, left );

    // setup sparse stereo
    stereo::StereoFeatures sparse;
    sparse.setCalibration( getTestCalibration(test) );

    // setup configuration
    stereo::FeatureConfiguration sparseConfig;
    sparseConfig.filterType = stereo::FILTER_STEREO;
    sparseConfig.knn = 2;
    sparseConfig.distanceFactor = 1.6;
    sparseConfig.maxStereoYDeviation = 3;
    sparse.setConfiguration( sparseConfig );

    // set the distance images for psurf processing
    sparse.setDistanceImages( ldist, rdist);

    // perform sparse processing
    sparse.processFramePair( left, right );

    // write sparse stereo debug image 
    cv::imwrite( prefix_out + "sparse.png", sparse.getDebugImage() );
}
