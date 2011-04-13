/*
 * imageprocessing.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: jfraedrich
 */

#include "imageprocessing.h"

using namespace cv;

ImageProcessing::ImageProcessing() {
	// TODO Auto-generated constructor stub
}

ImageProcessing::~ImageProcessing() {
	// TODO Auto-generated destructor stub
}

int ImageProcessing::acquireImage(IplImage *img, const char *name)
{
	int result = ERR_OK;

	IplImage *tmp = NULL;

	tmp = cvLoadImage(name, CV_LOAD_IMAGE_GRAYSCALE);

	if(tmp == NULL)
	{
		std::cerr << "Not able to load " << name << std::endl;
		result = ERR_NO_VALID_IMAGE;
	} else {
		cvCopy(tmp, img);
		cvReleaseImage(&tmp);
	}

	return result;
}

int ImageProcessing::preprocessImage(IplImage *img, bool right_image, CalibrationParameters *calibration){

	int result = ERR_OK;
	Mat newImage;
	//IplImage *rotatedImage = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);

	//this->rotateImage(img, rotatedImage, CV_PI);
	if(!right_image)
	{ // in case we process left image
	remap(img, newImage, calibration->map11, calibration->map21, INTER_CUBIC);
	} else {// for the right image
	remap(img, newImage, calibration->map12, calibration->map22, INTER_CUBIC);
	}
	CvMat tMat = newImage;
	cvCopy(&tMat, img);
	
	return result;
}

void ImageProcessing::rotateImage(IplImage *src, IplImage *dst, double angle){

	double map[6];
	CvMat map_matrix = cvMat(2, 3, CV_64FC1, map);

	CvPoint2D32f pt = cvPoint2D32f(src->width / 2, src->height / 2);

	cv2DRotationMatrix(pt, angle * 180. / CV_PI, 1.0, &map_matrix);

	cvWarpAffine(src, dst, &map_matrix, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
}
