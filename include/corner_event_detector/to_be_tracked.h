#pragma once

#include <deque>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>

namespace corner_event_detector
{

class ToBeTracked 
{
public:
  ToBeTracked(void);
  virtual ~ToBeTracked(void);
  bool ClassifyEvent(int x, int y);
  bool ClassifyCorner(int x, int y);
  void Update(int x, int y, int t);
  void ToBeTracked::ActiveCornersUpdate(double t);
  void ToBeTracked::UpdateInactiveCorner(int i, double t);

private:
  Eigen::MatrixXd tracked_corners;
  // Declare some static variables
  static const int ncorners = 10;
  static const int center_x = 150;
  static const int center_y = 40;
  static const int radius = 10;
  static const int window_size = 5; //Actually half of the window
  static constexpr double pi = 3.1415;
  int corner_offset[ncorners];
  int object_center[2];
  static const double dt_max = 0.1;
  static const double init_ts = 0; // Some small number
  int mean_offset;
};

} // namespace
