#ifndef DENSE_STEREO_TYPES__H
#define DENSE_STEREO_TYPES__H

#include <base/time.h>
#include <vector>

namespace dense_stereo {

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

  struct libElasConfiguration
  {
    bool postprocess_only_left;
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

    /// distance values stored in row major order
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
