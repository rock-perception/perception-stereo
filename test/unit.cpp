#define BOOST_TEST_MODULE StereoTest 
#include <boost/test/included/unit_test.hpp>

#include <frame_helper/CalibrationCv.h>
#ifdef HAS_SPARSE_STEREO
#include <stereo/sparse_stereo.hpp>
#endif
#include <stereo/densestereo.h>
#include <stereo/homography.h>

#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"


#ifdef HAS_SPARSE_STEREO
BOOST_AUTO_TEST_CASE( sparse_test ) 
{
    std::cout << "Testing Sparse Stereo functionality" << std::endl;

    const size_t width = 640, height = 480;

    cv::Mat leftImage(cv::Size(width, height),CV_8UC1);
    cv::Mat rightImage(cv::Size(width, height),CV_8UC1);
    cv::Mat transformation;
    stereo::FeatureConfiguration configuration;
    frame_helper::StereoCalibration calib;

    // create canconical calibration matrices for the two virtual cameras with a 100mm stereo setup 
    calib.camLeft.fx = 200.0;
    calib.camLeft.fy = 201.0;
    calib.camLeft.d0 = calib.camLeft.d1 = calib.camLeft.d2 = calib.camLeft.d3 = 0.0;
    calib.camLeft.cx = width / 2;
    calib.camLeft.cy = height / 2;

    calib.camRight.fx = 200.0;
    calib.camRight.fy = 201.0;
    calib.camRight.d0 = calib.camRight.d1 = calib.camRight.d2 = calib.camRight.d3 = 0.0;
    calib.camRight.cx = width / 2;
    calib.camRight.cy = height / 2;

    calib.extrinsic.tx = 100;
    calib.extrinsic.ty = calib.extrinsic.tz = 0;
    calib.extrinsic.rx = calib.extrinsic.ry = calib.extrinsic.rz = 0.01;

    // create data on left image. Data will be some rectangles as well as 100 randomly drawn circles of differing size and fill all over the image
    // add some filled rectangles
    for(int i = 0; i < 40; ++i)
    {
      cv::Point p1 = cv::Point(std::rand() % (width - 20), std::rand() % (width - 20));
      cv::rectangle(leftImage, p1, cv::Point(p1.x + std::rand() % 70, p1.y + std::rand() % 50), cv::Scalar(std::rand() % 128 + 128), -1);
    }
    // now the circles
    for(int i = 0; i < 100; ++i)
    {
      cv::circle(leftImage, cv::Point(std::rand() % (width - 20) + 10, std::rand() % (width - 20) + 10), std::rand() % 10, cv::Scalar(std::rand() % 128 + 128), std::rand() % 4);
    }
    // now transform that data by -50/0 pixels and 0° into the right image for matching
    double angle = 0.0 / 180.0 * M_PI;
    transformation = (cv::Mat_<double>(2,3) << cos(angle), -sin(angle), -50, sin(angle), cos(angle), 0);
    cv::warpAffine(leftImage, rightImage, transformation, cv::Size(width, height));

    // conversion from the enum DESCRIPTOR_TYPE to text
    std::vector<std::string> detector_types;
    detector_types.push_back("SURF");
    detector_types.push_back("GOOD");
    detector_types.push_back("SURFGPU");
    detector_types.push_back("STAR");
    detector_types.push_back("MSER");
    detector_types.push_back("SIFT");
    detector_types.push_back("FAST");
    detector_types.push_back("SURF_CV_GPU");

    // create the feature class
    stereo::StereoFeatures *features = new stereo::StereoFeatures();
 
    // check if there is GPU support on the machine
    int cuda_devices = cv::gpu::getCudaEnabledDeviceCount();
    std::cout << "Number of CUDA-enabled devices: " << cuda_devices << std::endl;
    if(cuda_devices < 1)
    {
      std::cout << "Skipping GPU tests, no suitable CUDA devices present." << std::endl;
    }
    else
    {
      // initialize the GPU
      cv::gpu::setDevice(0);
    }

    configuration.detectorType = stereo::DETECTOR_SURF; 
    for(int i = 0; i < 8; ++i)
    {
      // skip the GPU tests if there is no GPU
      if(cuda_devices < 1 && (i == 2 || i == 7))
        continue;
      char buf[200];
      std::cout << "Testing " << detector_types[i] << " Detector...";
      features->setConfiguration(configuration);
      features->setCalibration(calib);
      clock_t start = clock();
      features->processFramePair(leftImage, rightImage);
      clock_t finish = clock();
      double lResult = (double)(finish - start) / (double)(CLOCKS_PER_SEC / 1000);
      stereo::StereoFeatureArray stereo_features = features->getStereoFeatures();
      // determine the number of features. If none were detected, there is a problem.
      std::cout << " Number of features: " << stereo_features.keypoints.size() << " in " << lResult << "ms. ";
      if(stereo_features.keypoints.size() > 0)
        std::cout << " Success!" << std::endl;
      else
        std::cout << " Failed!" << std::endl;
      // save debug image
      sprintf(buf, "out_%s.png", detector_types[i].data());
      cv::imwrite(buf, features->getDebugImage());
      // increase the detector type
      configuration.detectorType = (stereo::DETECTOR)(configuration.detectorType + 1);
    }

    std::cout << "Performing test on external (class-foreign)  memory for SURF-descriptor...";

    configuration.detectorType = stereo::DETECTOR_SURF;
    features->setConfiguration(configuration);
    stereo::StereoFeatureArray stereo_features;
    features->processFramePair(leftImage, rightImage, &stereo_features);
    std::cout << " Number of features: " << stereo_features.keypoints.size() << ". ";
    if(stereo_features.keypoints.size() > 0)
      std::cout << " Success!" << std::endl;
    else
      std::cout << " Failed!" << std::endl;

    std::cout << "performing copyTo test...";
    stereo::StereoFeatureArray target;
    stereo_features.copyTo(target);
    if(stereo_features == target)
      std::cout << "done." << std::endl;
    else
      std::cout << "failed!" << std::endl;

    std::cout << "performing save/load test...";

    std::vector<stereo::StereoFeatureArray> m, m2;
    m.push_back(stereo_features);
    m.push_back(stereo_features);

    std::ofstream out;
    out.open("test.dat", std::ios::binary);
    StoreClassVector(m, out);
    out.close();

    std::ifstream in;
    in.open("test.dat", std::ios::binary);
    LoadClassVector(m2, in);
    in.close();

    if(stereo_features == m2[0])
      std::cout << " Success!" << std::endl;
    else
      std::cout << " Failed!" << std::endl;

    std::cout << "Finished all Sparse Stereo tests." << std::endl << std::endl;
}
#endif


const std::string prefix = "test/";
const std::string prefix_out = "build/test/";

frame_helper::StereoCalibration getTestCalibration( const std::string& name, int width, int height )
{
    return frame_helper::StereoCalibration::fromMatlabFile( prefix + "calib" + name + ".txt", width, height );
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
    calib.setCalibration( getTestCalibration( name, cleft.size().width, cleft.size().height ) );
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
    dense.setStereoCalibration( getTestCalibration("", left.size().width, left.size().height ), left.size().width, left.size().height );

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


#ifdef HAS_SPARSE_STEREO
BOOST_AUTO_TEST_CASE( psurf_test ) 
{
    const std::string test = "";
    // read input images
    cv::Mat left, right;
    getTestImages( test, left, right );

    // setup dense stereo object and init calibration
    stereo::DenseStereo dense;
    dense.setStereoCalibration( getTestCalibration(test, left.size().width, left.size().height ), left.size().width, left.size().height );

    // process left/right frame and get disparity images
    base::samples::DistanceImage ldist, rdist;
    dense.getDistanceImages( left, right, ldist, rdist, true );

    // test homography
    testHomography( ldist, left );

    // setup sparse stereo
    stereo::StereoFeatures sparse;
    sparse.setCalibration( getTestCalibration(test, left.size().width, left.size().height) );

    // setup configuration
    stereo::FeatureConfiguration sparseConfig;
    sparseConfig.filterType = stereo::FILTER_STEREO;
    sparseConfig.knn = 2;
    sparseConfig.distanceFactor = 1.6;
    sparseConfig.maxStereoYDeviation = 3;
    sparseConfig.descriptorType = envire::DESCRIPTOR_PSURF;
    sparse.setConfiguration( sparseConfig );

    // set the distance images for psurf processing
    sparse.setDistanceImages( &ldist, &rdist);

    // perform sparse processing
    sparse.processFramePair( left, right );

    // write sparse stereo debug image 
    cv::imwrite( prefix_out + "sparse-psurf.png", sparse.getDebugImage() );

    // try the same without psurf
    sparseConfig.descriptorType = envire::DESCRIPTOR_SURF;
    sparse.setConfiguration( sparseConfig );
    sparse.processFramePair( left, right );

    // write sparse stereo debug image 
    cv::imwrite( prefix_out + "sparse-surf.png", sparse.getDebugImage() );
}
#endif
