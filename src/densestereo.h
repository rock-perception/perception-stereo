/* Author: Jan Frädrich */

// Main header file. Include this to use dense_stereo in your code.

#ifndef __DENSE_STEREO_H__
#define __DENSE_STEREO_H__

#include <iostream>
#include "elas.h"
#include "image.h"

class DenseStereo {
  
public:
  DenseStereo();
  virtual ~DenseStereo();
  /** compute disparities of input frame pair left_frame, right_frame 
   * @param left_frame left input frame
   * @param right_frame right input frame
   * @param left_output_frame left output frame
   * @param right_output_frame right output frame
   */  
  void process_FramePair (const cv::Mat &left_frame,const cv::Mat &right_frame,
				       cv::Mat left_output_frame,cv::Mat right_output_frame);
  /** compute disparities of image input pair file_1, file_2 
   * @param file_1 filename of left input image
   * @param file_2 filename of right input image
   */
  void process_images (const char* file_1,const char* file_2);
  
  /** rotates the input image by angle
   * @param img image to be rotated
   * @param rotatedImg rotated image
   * @param angle angle in radians
   */
  void rotateImage(const cv::Mat &img, cv::Mat &rotatedImg, const double angle);
  
private:
  ///Instance of Elas
  Elas *elas;
  
  /** rectify \c image with openCV 
   * @param image pointer to the image which should be rectified
   * @param right_image decides whether this image is handeld as right one or not
   */
  void rectify(IplImage *image, const bool right_image);
};

#endif
