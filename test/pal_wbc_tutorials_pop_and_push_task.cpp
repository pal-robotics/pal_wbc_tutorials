#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <property_bag/property_bag.h>
#include <pal_utils/permutation.h>
#include <pal_wbc_utils/pal_wbc_utils.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <sensor_msgs/JointState.h>

struct JointData
{
  JointData() : joint_angles(), setup(true)
  {
  }

  void callback(const sensor_msgs::JointStateConstPtr &msg)
  {
    if (setup)
    {
      joint_angles.resize(msg->position.size());
      setup = false;
    }
    for (size_t i = 0; i < msg->position.size(); i++)
      joint_angles[i] = (msg->position.at(i));
  }

public:
  std::vector<double> joint_angles;
  bool setup;
};

TEST(PopAndPushTest, DynamicReconfigureTest)
{
  ros::NodeHandle nh;
  JointData j;

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  /// Task Creation
  /// By default, only the joint limits task will be pushed, So we create a new task of
  /// joint reference and push it on to the stack
  pal::WBCServiceHelper srv_helper(nh);

  property_bag::PropertyBag task;
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  task.addProperty("p_pos_gain", 2.0);

  task.addProperty("taskType", std::string("pal_wbc/ReferenceKinematicTaskRRBot"));
  task.addProperty("task_id", std::string("rrbot_default_reference"));

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

  task.addProperty("joint_names", joint_names);
  task.addProperty("signal_reference", std::string("vector_dynamic_reconfigure"));

  Eigen::VectorXd reference_posture(joint_names.size());
  reference_posture.setZero();
  if (nh.hasParam("/whole_body_kinematic_controller/default_configuration"))
  {
    ROS_INFO("Getting reference from param server");
    double pos = 0.0;
    nh.getParam("/whole_body_kinematic_controller/default_configuration/single_rrbot_joint1", pos);

    reference_posture(indexVectorThrow(joint_names, std::string("single_rrbot_joint1"))) = pos;
    nh.getParam("/whole_body_kinematic_controller/default_configuration/single_rrbot_joint2", pos);
    reference_posture(indexVectorThrow(joint_names, std::string("single_rrbot_joint2"))) = pos;
  }

  task.addProperty("reference", reference_posture);

  srv_helper.pushTask(task, "rrbot_default_reference", order, "rrbot_joint_limits");

  ros::Duration(10.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(reference_posture(i), j.joint_angles[i], 1e-4);

  ROS_INFO("Default Configuration has been successfully achieved");

  /// Now we shall make it to new position by popint out the current task and pushing the
  /// new updates position task
  reference_posture(0) = 0.8;
  reference_posture(1) = 0.8;
  task.updateProperty("reference", reference_posture);

  srv_helper.popTask("rrbot_default_reference");
  srv_helper.pushTask(task, "rrbot_default_reference", order, "rrbot_joint_limits");

  ros::Duration(7.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(reference_posture(i), j.joint_angles[i], 1e-3);

  ROS_INFO("Default Configuration has been successfully achieved");

  /// Lets pop up the task added in the previous case, which is of dynamic reconfigure
  /// type
  srv_helper.popTask("rrbot_default_reference");
}

TEST(PopAndPushTest, TopicTest)
{
  ros::NodeHandle nh;
  JointData j;

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  /// Task Creation
  /// By default, only the joint limits task will be pushed, So we create a new task of
  /// joint reference and push it on to the stack
  pal::WBCServiceHelper srv_helper(nh);

  property_bag::PropertyBag task;
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  task.addProperty("p_pos_gain", 2.0);

  task.addProperty("taskType", std::string("pal_wbc/ReferenceKinematicTaskRRBot"));
  task.addProperty("task_id", std::string("rrbot_default_reference"));

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

  task.addProperty("joint_names", joint_names);
  task.addProperty("signal_reference", std::string("vector_topic"));

  Eigen::VectorXd reference_posture(joint_names.size());
  reference_posture.setZero();
  if (nh.hasParam("/whole_body_kinematic_controller/default_configuration"))
  {
    ROS_INFO("Getting reference from param server");
    double pos = 0.0;
    nh.getParam("/whole_body_kinematic_controller/default_configuration/single_rrbot_joint1", pos);

    reference_posture(indexVectorThrow(joint_names, std::string("single_rrbot_joint1"))) = pos;
    nh.getParam("/whole_body_kinematic_controller/default_configuration/single_rrbot_joint2", pos);
    reference_posture(indexVectorThrow(joint_names, std::string("single_rrbot_joint2"))) = pos;
  }

  task.addProperty("reference", reference_posture);

  srv_helper.pushTask(task, "rrbot_default_reference", order, "rrbot_joint_limits");

  ros::Duration(10.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(reference_posture(i), j.joint_angles[i], 1e-4);

  ROS_INFO("Default Configuration has been successfully achieved");

  /// Now we shall make it to new position by popint out the current task and pushing the
  /// new updates position task
  reference_posture(0) = 0.4;
  reference_posture(1) = 1.0;
  task.updateProperty("reference", reference_posture);

  srv_helper.popTask("rrbot_default_reference");
  srv_helper.pushTask(task, "rrbot_default_reference", order, "rrbot_joint_limits");

  ros::Duration(7.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(reference_posture(i), j.joint_angles[i], 1e-3);

  ROS_INFO("Default Configuration has been successfully achieved");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dynamic_reconfigure_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
