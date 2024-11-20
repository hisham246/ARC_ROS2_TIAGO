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
