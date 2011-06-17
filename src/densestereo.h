/* Author: Jan Frädrich */

#ifndef __DENSE_STEREO_H__
#define __DENSE_STEREO_H__

#include <iostream>
#include "elas.h"
#include "configuration.h"
#include "imageprocessing.h"
#include "dense_stereo_types.h"

namespace dense_stereo {

class DenseStereo {
  
public:
  DenseStereo();
  /** load configuration for libelas and calibration data for the cams from file
   * @param conffile filename to load configuration from
   */
  DenseStereo(const std::string &conffile);
  
  virtual ~DenseStereo();
  /** set calibration parameters for stereo camera setup and configure libElas
   * @param stereoCamCal stereo camera calibration data
   * @param libElasParam libElas configuration
   */
  void setCalibrationAndLibElasConfiguration(const StereoCameraCalibration &stereoCamCal, const libElasConfiguration &libElasParam);
  
  /** compute disparities of input frame pair left_frame, right_frame 
   * @param left_frame left input frame
   * @param right_frame right input frame
   * @param left_output_frame left output frame
   * @param right_output_frame right output frame (optionally)
   */  
  void process_FramePair (const cv::Mat &left_frame,const cv::Mat &right_frame,
			  cv::Mat &left_output_frame,cv::Mat &right_output_frame);
  
private:
  ///Instance of libElas
  Elas *elas;
  
  ///calibration parameters
  CalibrationParameters calParam;
  
  /** rectify \c image with openCV 
   * @param image pointer to the image which should be rectified
   * @param right_image decides whether this image is handeld as right one or not
   */
  void rectify(cv::Mat &image, const bool right_image);
  
  /** convert colour of \c image to grayscale (uint8_t) with openCV
   * @param image Image which is converted
   */
  void cvtCvMatToGrayscaleImage(cv::Mat &image);
};

}

#endif
