#include <ik_teleop/Utils.h>

namespace teleop_utils
{
    double get_angular_distance(Eigen::Isometry3d Quat1, Eigen::Isometry3d Quat2)
    {
        // Get angular distances
        auto const q_1 = Eigen::Quaterniond(Quat1.rotation());
        auto const q_2 = Eigen::Quaterniond(Quat2.rotation());
        return q_2.angularDistance(q_1);
    }

    double get_linear_distance(Eigen::Isometry3d pos1, Eigen::Isometry3d pos2)
    {
        return (pos1.translation() - pos2.translation()).norm();
    }

    double get_angular_movement(std::vector<double> new_position, std::vector<double> old_position)
    {   
        double angular_movement = 0;
        for (int i = 0; i < new_position.size(); i++)
        {
            angular_movement += std::fabs(new_position[i]-old_position[i]);
        }
        return angular_movement;
    }
    
}

tf2::Transform eigenIsometry3dToTf2Transform(const Eigen::Isometry3d& eigen_transform) {
    // Extract rotation (as quaternion) and translation
    Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
    Eigen::Vector3d eigen_trans = eigen_transform.translation();

    // Convert to tf2 types
    tf2::Quaternion tf2_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
    tf2::Vector3 tf2_trans(eigen_trans.x(), eigen_trans.y(), eigen_trans.z());

    // Construct tf2::Transform
    return tf2::Transform(tf2_quat, tf2_trans);
}
