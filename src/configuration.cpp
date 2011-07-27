#include "configuration.h"

namespace dense_stereo {

void Configuration::loadLibElasDefaultParameters(Elas::parameters &elasParam){
  elasParam.postprocess_only_left = false;
}

void Configuration::loadLibElasConfiguration(const libElasConfiguration &libElasParam, Elas::parameters &elasParam){
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
}