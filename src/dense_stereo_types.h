#ifndef DENSE_STEREO_TYPES__H
#define DENSE_STEREO_TYPES__H

#include <base/time.h>
#include <vector>

namespace dense_stereo {

  /**
   * Calibration parameters expressing the camera matrix and the coefficients
   * for the lens distortion. See: 
   * http://opencv.willowgarage.com/documentation/camera_calibration_and_3d_reconstruction.html
   * for a description of the values and the model that is used.
   */
  struct CameraCalibration
  {
    double fx, fy, cx, cy, d0, d1, d2, d3;
  };

  struct ExtrinsicCalibration
  {
    double tx, ty, tz;
    double rx, ry, rz;
  };

  struct StereoCameraCalibration
  {
    int imgWidth, imgHeight;
    CameraCalibration CamLeft;
    CameraCalibration CamRight;
    ExtrinsicCalibration extrinsic;
  };

  /** Configuration parameters for lib elas.*/
  struct libElasConfiguration
  {
    int32_t disp_min;               // min disparity
    int32_t disp_max;               // max disparity
    float   support_threshold;      // max. uniqueness ratio (best vs. second best support match)
    int32_t support_texture;        // min texture for support points
    int32_t candidate_stepsize;     // step size of regular grid on which support points are matched
    int32_t incon_window_size;      // window size of inconsistent support point check
    int32_t incon_threshold;        // disparity similarity threshold for support point to be considered consistent
    int32_t incon_min_support;      // minimum number of consistent support points
    bool    add_corners;            // add support points at image corners with nearest neighbor disparities
    int32_t grid_size;              // size of neighborhood for additional support point extrapolation
    float   beta;                   // image likelihood parameter
    float   gamma;                  // prior constant
    float   sigma;                  // prior sigma
    float   sradius;                // prior sigma radius
    int32_t match_texture;          // min texture for dense matching
    int32_t lr_threshold;           // disparity threshold for left/right consistency check
    float   speckle_sim_threshold;  // similarity threshold for speckle segmentation
    int32_t speckle_size;           // maximal size of a speckle (small speckles get removed)
    int32_t ipol_gap_width;         // interpolate small gaps (left<->right, top<->bottom)
    bool    filter_median;          // optional median filter (approximated)
    bool    filter_adaptive_mean;   // optional adaptive mean filter (approximated)
    bool    postprocess_only_left;  // saves time by not postprocessing the right image
    bool    subsampling;            // saves time by only computing disparities for each 2nd pixel
                                    // note: for this option D1 and D2 must be passed with size
                                    //       width/2 x height/2 (rounded towards zero)
  };

  /** 
   * 2D array structure representing a distance image for a pinhole camera model.
   * 
   * The grid pixels are scaled such that (x*scale_x)+center_x = p_x are the
   * projective plane coordinates given a grid index x. This of course applies
   * to y as well.
   *
   * The data array is a row major flattened version of the image matrix,
   * giving the distance value d of the image points.  The relation is such
   * that for a point on the projection plane, the 3D point z can be calculated
   * as (p_x,p_y,1)*d = z.
   */
  struct distance_image
  {
    typedef float scalar;

    /// original timestamp of the camera image
    base::Time time;

    /// distance values stored in row major order. NaN is used as the no value type.
    std::vector<scalar> data;

    /// height (y) value in pixels
    uint16_t height;
    /// width (x) value in pixels
    uint16_t width;

    /// scale value to apply to the x axis
    scalar scale_x;
    /// scale value to apply to the y axis
    scalar scale_y;

    /// center offset to apply to the x axis
    scalar center_x;
    /// center offset to apply to the y axis
    scalar center_y;
  };

}

#endif
