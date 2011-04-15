/*
 * configuration.cpp
 *
 *  Created on: Apr 13, 2011
 *      Author: jfraedrich
 */

#include "configuration.h"

void Configuration::loadDefaultParameters(CalibrationParameters &calParam, Elas::parameters &elasParam){
  calParam.loadParameters();
  elasParam.postprocess_only_left = false;
}

void Configuration::loadConfigurationFromFile(const std::string &filename, CalibrationParameters &calParam, Elas::parameters &elasParam){
  //open config file storage from filename
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  
  cv::FileNode calibration = fs["calibration"];
  calParam.loadCalibrationFromFile(calibration);
}

void Configuration::saveConfigurationFile(const std::string &filename, CalibrationParameters &calParam, Elas::parameters &elasParam){
  //open config file storage from filename
  //cv::FileStorage fs(filename, cv::FileStorage::READ);
  
  //TODO save to same format as load expects it
  calParam.saveConfigurationFile(filename);
}