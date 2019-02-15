
// Standard includes
#include <capture_waypoints/helper.h>

namespace helper {

double deg2rad(const double &deg) {
	return deg*M_PI/180.0;
}

double rad2deg(const double &rad) {
	return rad*180.0/M_PI;
}

Eigen::Vector3d quat2rpy(geometry_msgs::Quaternion quat) {
	double qx, qy, qz, qw, roll, pitch, yaw;
	qx = quat.x;
	qy = quat.y;
	qz = quat.z;
	qw = quat.w;

	//Formulas for roll, pitch, yaw
	roll = atan2(2*(qw*qx + qy*qz) , 1 - 2*(qx*qx + qy*qy) );
	pitch = asin(2*(qw*qy - qz*qx));
	yaw = atan2(2*(qw*qz + qx*qy),1 - 2*(qy*qy + qz*qz) );

	Eigen::Vector3d rpy(roll, pitch, yaw);
	return rpy;
}

double getHeadingFromQuat(geometry_msgs::Quaternion quat) {
	Eigen::Vector3d RPY = quat2rpy(quat);
	return RPY[2];
}

}  // namespace helper
