#include <Eigen/Dense>

#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/stack_of_tasks_kinematic.h>

#include <pal_wbc_tutorial/joint_pos_limit_kinematic_task.h>
#include <pal_wbc_tutorial/reference_kinematic_task.h>
#include <pal_wbc_controller/generic_meta_task.h>
#include <pluginlib/class_list_macros.h>
#include <pal_utils/permutation.h>
#include <pal_ros_utils/reference/vector/vector_topic_reference.h>

using namespace pal_wbc;

/**
 * @brief get_default_reference_from_param_server, method to know the default
 * configuration values present in the parameter server
 * @param default_reference_joints - contains the available joints in the robot
 * @param default_reference_posture - default configuration of the each joint (output)
 * @return confirms the presence of the parameters in the parameter server.
 */
bool get_default_reference_from_param_server(const std::vector<std::string> &default_reference_joints,
                                             Eigen::VectorXd &default_reference_posture)
{
  ros::NodeHandle nh;

  if (nh.hasParam("/whole_body_kinematic_controller/default_configuration"))
  {
    ROS_INFO("Getting reference from param server");
    double pos = 0.0;
    nh.getParam("/whole_body_kinematic_controller/default_configuration/joint1", pos);
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("joint1"))) = pos;
    nh.getParam("/whole_body_kinematic_controller/default_configuration/joint2", pos);
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("joint2"))) = pos;
    return true;
  }
  return false;
}

/**
 * @brief The rrbot_stack class
 * This is the classs that sets up the stack with two different tasks, one is the joint
 * limits of the robot and the other one is the reference task of the robot.
 * Joint limits :  defined the limits of the each of the robots joints
 * reference task : defines the default configurations or the reference joint angles that
 * the robot need to consider.
 */
class rrbot_stack : public StackConfigurationKinematic
{
  bool setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh)
  {
    std::vector<double> joint_pos_min_override = stack->getJointPositionLimitMin();
    joint_pos_min_override[stack->getJointIndex("joint1")] = 0.2;
    joint_pos_min_override[stack->getJointIndex("joint2")] = 0.1;

    std::vector<std::string> default_reference_joints;
    default_reference_joints.push_back("joint1");
    default_reference_joints.push_back("joint2");

    // 1. Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
        new JointPositionLimitKinematicAllJointsMetaTask(
            *stack.get(), joint_pos_min_override, stack->getJointPositionLimitMax(),
            stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
            stack->getJointNames(), 1.0, false, nh));

    stack->pushTask("rrbot_joint_limits", joint_position_limit_task);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(rrbot_stack, StackConfigurationKinematic);
PLUGINLIB_EXPORT_CLASS(pal_wbc::ReferenceKinematicTaskRRBot, pal_wbc::TaskAbstract);
