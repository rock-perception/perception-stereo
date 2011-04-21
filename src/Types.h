#ifndef TYPES__H
#define TYPES__H

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
  
}

#endif
