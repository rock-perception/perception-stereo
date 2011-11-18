/* Author: Jan Frädrich */

#ifndef __DENSE_STEREO_H__
#define __DENSE_STEREO_H__

#include <iostream>
#include <libelas/elas.h>
#include <frame_helper/CalibrationCv.h>
#include "dense_stereo_types.h"
#include <base/samples/distance_image.h>

namespace stereo {

/** 
 * This class performs dense stereo calculation and is mainly a wrapper to
 * libelas. After construction, the setStereoCalibration method needs to be
 * called with a valid calibration. Configuration itself has sane defaults, but
 * can be changed any time.  process_FramePair produces a disparity image,
 * which by itself isn't all that usefull (function will probably be made
 * private soon). getDistanceImage is more likely what you want to call, which
 * produces images where the actual distance is encoded as a float. Use the
 * createDistanceImage method beforehand to get the cv::Mat to pass as a
 * parameter to getDistanceImage to get the base::samples::DistanceImage type
 * filled from this class.
 */
class DenseStereo {
  
public:
  DenseStereo();
  
  virtual ~DenseStereo();
  
  /** sets calibration parameters for stereo camera setup
   * @param stereoCamCal stereo camera calibration data
   */
  void setStereoCalibration(const frame_helper::StereoCalibration& stereoCal,
                            const int imgWidth,
                            const int imgHeight);
  
  /** configures libElas
   * @param libElasParam libElas configuration
   */
  void setLibElasConfiguration(const libElasConfiguration &libElasParam);

  /** 
   * if set to greater than 0, the images will be preprocessed with a 
   * gaussian blur filter with a kernel of the given size. Should be
   * an odd number.
   */
  void setGaussianKernel( int size ) { gaussian_kernel = size; }
  
  /** computes disparities of input frame pair left_frame, right_frame 
   * @param left_frame left input frame
   * @param right_frame right input frame
   * @param left_output_frame left output frame
   * @param right_output_frame right output frame
   * @param isRectified tells the function if the input images are already rectified
   */  
  void processFramePair(const cv::Mat &left_frame, const cv::Mat &right_frame,
			  cv::Mat &left_output_frame, cv::Mat &right_output_frame,
			  bool isRectified = false );

  /**
   * perform conversion from disparity to distance image
   */
  void getDistanceImages( cv::Mat &left_disp_image, cv::Mat &right_disp_image );

  /** 
   * computes the disparity images and calculates the distance images from them
   * @param left_frame left input frame
   * @param right_frame right input frame
   * @param left_output_frame left distance image
   * @param right_output_frame right distance image 
   * @param isRectified tells the function if the input images are already rectified
   */
  void getDistanceImages( const cv::Mat &left_frame, const cv::Mat &right_frame,
			  cv::Mat &left_output_frame, cv::Mat &right_output_frame,
			  bool isRectified = false );

  /** 
   * convenience method that takes base::samples::DistanceImages as the result objects
   *
   * @param left_frame left input frame
   * @param right_frame right input frame
   * @param left_output_frame left distance image
   * @param right_output_frame right distance image 
   * @param isRectified tells the function if the input images are already rectified
   */
  void getDistanceImages( const cv::Mat &left_frame, const cv::Mat &right_frame,
			  base::samples::DistanceImage &left_output_frame, base::samples::DistanceImage &right_output_frame,
			  bool isRectified = false );

  /** 
   * prepare a distance image from the provided camera calibration
   * the resulting cv::Mat shares the same data buffer as the dist_image
   * and can be used as input to getDistanceImages
   *
   * @param calib - the calibration for the camera to use
   * @param dist_image - the dist_image which will be filled with the calib data and image size
   * @result cv::Mat which shares a data buffer with dist_image
   */
  cv::Mat createDistanceImage( 
	  frame_helper::CameraCalibrationCv const& calib, 
	  base::samples::DistanceImage& dist_image );

  /**
   * convenience method which doesn't require a calibration object
   * but uses the camera calibration for the left camera
   */
  cv::Mat createLeftDistanceImage( 
	  base::samples::DistanceImage& dist_image )
  {
      return createDistanceImage( calParam.camLeft, dist_image );
  }

  /**
   * convenience method which doesn't require a calibration object
   * but uses the camera calibration for the right camera
   */
  cv::Mat createRightDistanceImage( 
	  base::samples::DistanceImage& dist_image )
  {
      return createDistanceImage( calParam.camRight, dist_image );
  }
			  
  
private:
  /// see if we need to apply a gaussian filter
  int gaussian_kernel;

  ///instance of libElas
  Elas *elas;
  
  ///calibration parameters
  frame_helper::StereoCalibrationCv calParam;
  
  ///calibration initialized?
  bool calibrationInitialized;
  
  /** undistorts and rectifies an image with openCV 
   * @param image image which should be undistorted and rectified
   * @param calib calibration which should be used for undistortion and rectification
   */
  void undistortAndRectify(cv::Mat &image, const frame_helper::CameraCalibrationCv& calib);
  
  /** converts colour of an image to grayscale (uint8_t) with openCV
   * @param image Image which is converted
   */
  void cvtCvMatToGrayscaleImage(cv::Mat &image);
};

}

#endif
