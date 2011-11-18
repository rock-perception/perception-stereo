#include "configuration.h"

namespace stereo {

void copyFromElas( const Elas::parameters *params, libElasConfiguration *config )
{
    config->disp_min = params->disp_min;
    config->disp_max = params->disp_max;
    config->support_threshold = params->support_threshold;
    config->support_texture = params->support_texture;
    config->candidate_stepsize = params->candidate_stepsize;
    config->incon_window_size = params->incon_window_size;
    config->incon_threshold = params->incon_threshold;
    config->incon_min_support = params->incon_min_support;
    config->add_corners = params->add_corners;
    config->grid_size = params->grid_size;
    config->beta = params->beta;
    config->gamma = params->gamma;
    config->sigma = params->sigma;
    config->sradius = params->sradius;
    config->match_texture = params->match_texture;
    config->lr_threshold = params->lr_threshold;
    config->speckle_sim_threshold = params->speckle_sim_threshold;
    config->speckle_size = params->speckle_size;
    config->ipol_gap_width = params->ipol_gap_width;
    config->filter_median = params->filter_median;
    config->filter_adaptive_mean = params->filter_adaptive_mean;
    config->postprocess_only_left = params->postprocess_only_left;
    config->subsampling = params->subsampling;
}

void copyToElas( const libElasConfiguration *config, Elas::parameters *params )
{
    params->disp_min = config->disp_min;
    params->disp_max = config->disp_max;
    params->support_threshold = config->support_threshold;
    params->support_texture = config->support_texture;
    params->candidate_stepsize = config->candidate_stepsize;
    params->incon_window_size = config->incon_window_size;
    params->incon_threshold = config->incon_threshold;
    params->incon_min_support = config->incon_min_support;
    params->add_corners = config->add_corners;
    params->grid_size = config->grid_size;
    params->beta = config->beta;
    params->gamma = config->gamma;
    params->sigma = config->sigma;
    params->sradius = config->sradius;
    params->match_texture = config->match_texture;
    params->lr_threshold = config->lr_threshold;
    params->speckle_sim_threshold = config->speckle_sim_threshold;
    params->speckle_size = config->speckle_size;
    params->ipol_gap_width = config->ipol_gap_width;
    params->filter_median = config->filter_median;
    params->filter_adaptive_mean = config->filter_adaptive_mean;
    params->postprocess_only_left = config->postprocess_only_left;
    params->subsampling = config->subsampling;
}

}
