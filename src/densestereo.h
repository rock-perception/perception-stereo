/* Author: Jan Frädrich */

#ifndef __DENSE_STEREO_H__
#define __DENSE_STEREO_H__

#include <iostream>
#include <libelas/elas.h>
#include <frame_helper/CalibrationCv.h>
#include "dense_stereo_types.h"

namespace stereo {

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
  
  /** computes disparities of input frame pair left_frame, right_frame 
   * @param left_frame left input frame
   * @param right_frame right input frame
   * @param left_output_frame left output frame
   * @param right_output_frame right output frame
   * @param isRectified tells the function if the input images are already rectified
   */  
  void process_FramePair (const cv::Mat &left_frame, const cv::Mat &right_frame,
			  cv::Mat &left_output_frame, cv::Mat &right_output_frame,
			  bool isRectified = false );
  
private:
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
