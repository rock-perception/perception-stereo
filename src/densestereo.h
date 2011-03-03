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
  /** compute disparities of image input pair file_1, file_2 
   * @param file_1 filename of left input image
   * @param file_2 filename of right input image
   */
  void process_images (const char* file_1,const char* file_2);
  
};

#endif
