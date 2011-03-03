/*
 * calibrationparameters.cpp
 *
 *  Created on: Jul 7, 2010 modified on Aug 18, 2010
 *      Author: dstoyanov, jfraedrich
 */

#include "calibrationparameters.h"

CalibrationParameters::CalibrationParameters() {
	this->cameraMatrix1.create(3,3, CV_64F);
	this->cameraMatrix2.create(3,3, CV_64F);
	this->distCoeffs1.create(1,4, CV_64F);
	this->distCoeffs2.create(1,4, CV_64F);
	this->T.create(3,1, CV_64F);
	this->R.create(3,3, CV_64F);
}

CalibrationParameters::~CalibrationParameters() {
	// TODO Auto-generated destructor stub
}

void CalibrationParameters::loadParameters(){

	imgWidth = 640;
	imgHeight = 480;

	/*
	//Spidercam Calibration
	this->fx1 = 1675.06416; this->fy1 = 1676.80916;
        this->cx1 = 1013.127; this->cy1 = 594.6255;
        this->d01 = -0.11161; this->d11 = 0.32284; this->d21 = 0.00062; this->d31 = 0.01055;
        
	this->fx2 = 1666.63512; this->fy2 = 1662.617;
        this->cx2 = 1018.218; this->cy2 = 574.8345;
        this->d02 = -0.04178; this->d12 = -0.06356; this->d22 = -0.00023; this->d32 = 0.00963;

        this->tx = -335.67259; this->ty = -44.43439;*//* ursprünglich -3.43439 *//* this->tz = -17.18048;
        this->rx = 0.0; this->ry = 0.0; this->rz = 0.0;
*/
	
	/* asguard cam guppy */
	//intrinsic parameters
	//left camera
	this->fx1 = 700.65830; this->fy1 = 702.43840;
	this->cx1 = 318.65453; this->cy1 = 256.03635;
	this->d01 = -0.05278; this->d11 = 0.09852; this->d21 = 0.00072; this->d31 = 0.00170;
	//right camera
	this->fx2 = 698.16544; this->fy2 = 700.20649;
	this->cx2 = 315.86462; this->cy2 = 233.76471;
	this->d02 = -0.04832; this->d12 = 0.09334; this->d22 = -0.00071; this->d32 = 0.00001;

	//extrinsic parameters
	//translation
	this->tx = -251.29734; this->ty = -1.65053; this->tz = -8.26461;
	//rotation
	this->rx = -0.00284; this->ry = -0.01155; this->rz = 0.00804;

	cv::Mat tempRot;
	tempRot.create(3, 1, CV_64F);
	tempRot.at<double>(0,0) = this->rx;
	tempRot.at<double>(1,0) = this->ry;
	tempRot.at<double>(2,0) = this->rz;
	
	//convert from rotation vector to rotation matrix
	Rodrigues(tempRot, this->R);
	//this->R += cv::Mat::eye(this->R.rows, this->R.cols, CV_64F);

//	this->cameraMatrix1.create(3, 3, CV_64F);
	this->cameraMatrix1 = 0.0; //set all entries to 0
	this->cameraMatrix1.at<double>(2,2) = 1.0;
	this->cameraMatrix1.at<double>(0,0) = this->fx1;
	this->cameraMatrix1.at<double>(0,2) = this->cx1;
	this->cameraMatrix1.at<double>(1,1) = this->fy1;
	this->cameraMatrix1.at<double>(1,2) = this->cy1;

//	this->cameraMatrix2.create(3, 3, CV_64F);
	this->cameraMatrix2 = 0.0; //set all entries to zero
	this->cameraMatrix2.at<double>(2,2) = 1.0;
	this->cameraMatrix2.at<double>(0,0) = this->fx2;
	this->cameraMatrix2.at<double>(0,2) = this->cx2;
	this->cameraMatrix2.at<double>(1,1) = this->fy2;
	this->cameraMatrix2.at<double>(1,2) = this->cy2;

//	this->distCoeffs1.create(1, 4, CV_64F);
	this->distCoeffs1.at<double>(0,0) = this->d01;
	this->distCoeffs1.at<double>(0,1) = this->d11;
	this->distCoeffs1.at<double>(0,2) = this->d21;
	this->distCoeffs1.at<double>(0,3) = this->d31;

//	this->distCoeffs2.create(1, 4, CV_64F);
	this->distCoeffs2.at<double>(0,0) = this->d02;
	this->distCoeffs2.at<double>(0,1) = this->d12;
	this->distCoeffs2.at<double>(0,2) = this->d22;
	this->distCoeffs2.at<double>(0,3) = this->d32;

//	this->T.create(3, 1, CV_64F);
	this->T.at<double>(0,0) = this->tx;
	this->T.at<double>(1,0) = this->ty;
	this->T.at<double>(2,0) = this->tz;
}

void CalibrationParameters::calculateUndistortAndRectifyMaps(){
  cv::Mat R1, R2, P1, P2;
  cv::Size newSize = cv::Size(imgWidth, imgHeight);

  //calculate the rectification parameters
  //opencv 2.1:
  stereoRectify(this->cameraMatrix1, this->distCoeffs1, this->cameraMatrix2, this->distCoeffs2, newSize, this->R, this->T, R1, R2, P1, P2, this->Q, 0., cvSize(0,0), &(this->roi1), &(this->roi2));

  //important: an error in the stereoRectify routine can lead the Q matrix to be NAN - check and fix that! Version?
  if(isnan(this->Q.at<double>(3, 3)))
  {
    this->Q.at<double>(0, 3) = -this->cx1;
    this->Q.at<double>(1, 3) = -this->cy1;
    this->Q.at<double>(3, 3) = 0.0;
  }
  
  //calculate the undistort/rectify maps for both images
  initUndistortRectifyMap(this->cameraMatrix1, this->distCoeffs1, R1, P1, newSize, CV_32FC1, this->map11, this->map21);

  initUndistortRectifyMap(this->cameraMatrix2, this->distCoeffs2, R2, P2, newSize, CV_32FC1, this->map12, this->map22);
}
