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

void Configuration::loadConfiguration(const StereoCameraCalibration &stereoCamCal, const libElasConfiguration &libElasParam, CalibrationParameters &calParam, Elas::parameters &elasParam){
  calParam.setStereoCalibrationParameters(stereoCamCal);
  
  //TODO better copying between structs
  elasParam.disp_min = libElasParam.disp_min;
  elasParam.disp_max = libElasParam.disp_max;
  elasParam.support_threshold = libElasParam.support_threshold;
  elasParam.support_texture = libElasParam.support_texture;
  elasParam.candidate_stepsize = libElasParam.candidate_stepsize;
  elasParam.incon_window_size = libElasParam.incon_window_size;
  elasParam.incon_threshold = libElasParam.incon_threshold;
  elasParam.incon_min_support = libElasParam.incon_min_support;
  elasParam.add_corners = libElasParam.add_corners;
  elasParam.grid_size = libElasParam.grid_size;
  elasParam.beta = libElasParam.beta;
  elasParam.gamma = libElasParam.gamma;
  elasParam.sigma = libElasParam.sigma;
  elasParam.sradius = libElasParam.sradius;
  elasParam.match_texture = libElasParam.match_texture;
  elasParam.lr_threshold = libElasParam.lr_threshold;
  elasParam.speckle_sim_threshold = libElasParam.speckle_sim_threshold;
  elasParam.speckle_size = libElasParam.speckle_size;
  elasParam.ipol_gap_width = libElasParam.ipol_gap_width;
  elasParam.filter_median = libElasParam.filter_median;
  elasParam.filter_adaptive_mean = libElasParam.filter_adaptive_mean;
  elasParam.postprocess_only_left = libElasParam.postprocess_only_left;
  elasParam.subsampling = libElasParam.subsampling;
}