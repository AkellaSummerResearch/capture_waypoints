
#ifndef HELPER_H_
#define HELPER_H_

#include "geometry_msgs/Quaternion.h"
#include <Eigen/Dense>

#include <cmath>

namespace helper {

double deg2rad(const double &deg);

double rad2deg(const double &rad);

Eigen::Vector3d quat2rpy(geometry_msgs::Quaternion quat);

double getHeadingFromQuat(geometry_msgs::Quaternion quat);

}  // namespace helper

#endif  // HELPER_H_