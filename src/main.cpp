/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger, Jan F.

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// program showing how libelas can be used, try "./elas -h" for help

#include <iostream>
#include "densestereo.h"

#include <opencv/cv.h>

using namespace std;

int main (int argc, char** argv) {

  DenseStereo Dense_stereo;
  // run demo
  if (argc==2 && !strcmp(argv[1],"demo")) {
    Dense_stereo.process_images("img/cones_left.pgm",   "img/cones_right.pgm");
    Dense_stereo.process_images("img/aloe_left.pgm",    "img/aloe_right.pgm");
    Dense_stereo.process_images("img/raindeer_left.pgm","img/raindeer_right.pgm");
    Dense_stereo.process_images("img/urban1_left.pgm",  "img/urban1_right.pgm");
    Dense_stereo.process_images("img/urban2_left.pgm",  "img/urban2_right.pgm");
    Dense_stereo.process_images("img/urban3_left.pgm",  "img/urban3_right.pgm");
    Dense_stereo.process_images("img/urban4_left.pgm",  "img/urban4_right.pgm");
    cout << "... done!" << endl;

  // compute disparity from input pair
  } else if (argc==3) {
    Dense_stereo.process_images(argv[1],argv[2]);
    cout << "... done!" << endl;

  // compute disparity for multiple input images (currently png)
  } else if (argc==4) {
    /*string pathToImages = argv[1];
    char str_imgNum[9];

    for(int dense_count = atoi(argv[2]); dense_count < atoi(argv[3]); dense_count++){
      sprintf(str_imgNum, "%04d.png", dense_count);
      string left_tmp = pathToImages + "left_";
      left_tmp += str_imgNum;
      string right_tmp = pathToImages + "right_";
      right_tmp += str_imgNum;
      
      Dense_stereo.process_images(left_tmp.c_str(),right_tmp.c_str());
      cout << "... done!" << endl;*/
    string pathToImages = argv[1];
    char str_imgNum[9];

    for(int dense_count = atoi(argv[2]); dense_count < atoi(argv[3]); dense_count++){
      sprintf(str_imgNum, "%04d.png", dense_count);
      string left_tmp = pathToImages + "left_";
      left_tmp += str_imgNum;
      string right_tmp = pathToImages + "right_";
      right_tmp += str_imgNum;
      
      //read images
      cv::Mat left_frame, right_frame;
      left_frame = cv::imread(left_tmp);
      right_frame = cv::imread(right_tmp);

      cv::Mat left_output_frame = cv::Mat(left_frame.size().height, left_frame.size().width, cv::DataType<float>::type)
      , right_output_frame = cv::Mat(right_frame.size().height, right_frame.size().width, cv::DataType<float>::type);
      
      Dense_stereo.process_FramePair(left_frame, right_frame, left_output_frame, right_output_frame);
      
      ostringstream file_count;
      file_count << str_imgNum;

      //write images
      cv::imwrite(pathToImages + "right_frame_" + file_count.str() ,right_frame);
      cv::imwrite(pathToImages + "left_frame_" + file_count.str() ,left_frame);
      cv::imwrite(pathToImages + "right_frame_disp_" + file_count.str() ,right_output_frame);
      cv::imwrite(pathToImages + "left_frame_disp_" + file_count.str() ,left_output_frame);
      
      cout << "... done!" << endl;
    }
    
  // display help
  } else {
    cout << endl;
    cout << "ELAS demo program usage: " << endl;
    cout << "./elas demo ......................... process all test images (image dir)" << endl;
    cout << "./elas left.pgm right.pgm ........... process a single stereo pair" << endl;
    cout << "./elas path start_count stop_count .. process stop_count-start_count stereo pairs" << endl;
    cout << "./elas -h ........................... shows this help" << endl;
    cout << endl;
    cout << "Note: All images must be pgm greylevel images. All output" << endl;
    cout << "      disparities will be scaled such that disp_max = 255." << endl;
    cout << endl;
  }

  return 0;
}


