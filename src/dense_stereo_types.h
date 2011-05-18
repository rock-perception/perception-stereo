#ifndef DENSE_STEREO_TYPES__H
#define DENSE_STEREO_TYPES__H

#include <base/time.h>

namespace dense_stereo{

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

  struct disparity_image
  {
    std::vector<float> data;

    StereoCameraCalibration calibration;

    base::Time time;
  };

}

#endif
