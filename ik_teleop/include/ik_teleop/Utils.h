#ifndef BIO_IK_UTILS
#define BIO_IK_UTILS
#include <tf2_eigen/tf2_eigen.hpp>
#include <bio_ik/bio_ik.hpp>

namespace teleop_utils
{
    double get_angular_distance(Eigen::Isometry3d Quat1, Eigen::Isometry3d Quat2);

    double get_linear_distance(Eigen::Isometry3d pos1, Eigen::Isometry3d pos2);

    double get_angular_movement(std::vector<double> new_position, std::vector<double> old_position);
}

class MinimizeJointVariableGoal : public bio_ik::Goal
{
    std::string variable_name;
    double variable_position;

public:
    MinimizeJointVariableGoal()
    {
    }
    MinimizeJointVariableGoal(const std::string& variable_name, double weight = 1.0, bool secondary = false)
        : variable_name(variable_name)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    const std::string& getVariableName() const { return variable_name; }
    void setVariableName(const std::string& n) { variable_name = n; }
    virtual void describe(bio_ik::GoalContext& context) const
    {
        Goal::describe(context);
        context.addVariable(variable_name);
    }
    virtual double evaluate(const bio_ik::GoalContext& context) const
    {
        double d = context.getProblemVariableInitialGuess(0) - context.getProblemVariablePosition(0);
        return d * d;
    }
};

class PositionZGoal : public bio_ik::LinkGoalBase
{
    tf2::Vector3 position_;

public:
    PositionZGoal()
        : position_(0, 0, 0)
    {
    }
    PositionZGoal(const std::string& link_name, const tf2::Vector3& position, double weight = 1.0)
        : bio_ik::LinkGoalBase(link_name, weight)
        , position_(position)
    {
    }
    inline const tf2::Vector3& getPosition() const { return position_; }
    inline void setPosition(const tf2::Vector3& position) { position_ = position; }
    virtual double evaluate(const bio_ik::GoalContext& context) const { return abs(context.getLinkFrame().getPosition().z() - getPosition().z()); }
};

#endif