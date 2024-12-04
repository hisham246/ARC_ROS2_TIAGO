#ifndef BIO_IK_UTILS
#define BIO_IK_UTILS
#include <tf2_eigen/tf2_eigen.hpp>

namespace teleop_utils
{
    double get_angular_distance(Eigen::Isometry3d Quat1, Eigen::Isometry3d Quat2);

    double get_linear_distance(Eigen::Isometry3d pos1, Eigen::Isometry3d pos2);

    double get_angular_movement(std::vector<double> new_position, std::vector<double> old_position);
}

#endif