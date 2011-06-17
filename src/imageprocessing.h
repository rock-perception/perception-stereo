#ifndef IMAGEPROCESSING_H_
#define IMAGEPROCESSING_H_

#include "calibrationparameters.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

namespace dense_stereo {

class ImageProcessing {
public:
	ImageProcessing();
	virtual ~ImageProcessing();

	/** reads the image in path \c name to \c img 
	 * @param img the left image
	 * @param name the left image to load
	 * @result if result != ERR_OK then check which pointer has not changed
	 */
	int acquireImage(IplImage *img, const char *name);

	/** preprocesses image using the calibration data (intrinsic)
	 * @param img image to preprocess (undistort/rectify)
	 * @param right_image set to true if preprocessing right image
	 * @param calibration calibration parameters of the cams*/
	int preprocessImage(cv::Mat &img, bool right_image, CalibrationParameters *calibration);
private:
	/** Rotates image \c src to \c angle radians */
	void rotateImage(IplImage *src, IplImage *dst, double angle);
};

}
#endif /* IMAGEPROCESSING_H_ */
