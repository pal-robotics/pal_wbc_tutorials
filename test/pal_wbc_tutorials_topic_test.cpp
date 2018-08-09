#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
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

TEST(TopicTest, DefaultConfigurationAngles)
{
  ros::NodeHandle nh;
  JointData j;

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  ros::Time start_time = ros::Time::now();
  while(j.joint_angles.empty() && ((ros::Time::now() - start_time) < ros::Duration(5.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(j.joint_angles.empty());

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
}

TEST(TopicTest, ChangingJoint1AngleParameters)
{
  ros::NodeHandle nh;
  JointData j;

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  ros::Time start_time = ros::Time::now();
  while(j.joint_angles.empty() && ((ros::Time::now() - start_time) < ros::Duration(5.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(j.joint_angles.empty());

  ros::Publisher command = nh.advertise<sensor_msgs::JointState>(
      "/whole_body_kinematic_controller/reference_ref", 10);

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

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

  /// sending the command to move to a particular position through topic
  /// Changing only the first joint in this case
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.name.resize(joint_names.size());
  msg.name = joint_names;
  msg.position.resize(joint_names.size());
  msg.position[0] = 1.0;
  msg.position[1] = reference_posture(1);
  msg.velocity.resize(joint_names.size());
  msg.effort.resize(joint_names.size());

  while (command.getNumSubscribers() == 0)
  {
    ros::Duration(0.05).sleep();
    command.publish(msg);
    ros::spinOnce();
  }

  ros::Duration(10.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    ros::spinOnce();
    ASSERT_NEAR(msg.position[i], j.joint_angles[i], 1e-3);
  }
}

TEST(TopicTest, ChangingJoint2AngleParameters)
{
  ros::NodeHandle nh;
  JointData j;

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  ros::Time start_time = ros::Time::now();
  while(j.joint_angles.empty() && ((ros::Time::now() - start_time) < ros::Duration(5.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(j.joint_angles.empty());

  ros::Publisher command = nh.advertise<sensor_msgs::JointState>(
      "/whole_body_kinematic_controller/reference_ref", 10);

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

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

  /// sending the command to move to a particular position through topic
  /// Changing only the first joint in this case
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.name.resize(joint_names.size());
  msg.name = joint_names;
  msg.position.resize(joint_names.size());
  msg.position[0] = reference_posture(0);
  msg.position[1] = 1.4;
  msg.velocity.resize(joint_names.size());
  msg.effort.resize(joint_names.size());

  while (command.getNumSubscribers() == 0)
  {
    ros::Duration(0.05).sleep();
    command.publish(msg);
    ros::spinOnce();
  }

  ros::Duration(10.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    ros::spinOnce();
    ASSERT_NEAR(msg.position[i], j.joint_angles[i], 1e-3);
  }
}

TEST(TopicTest, LowerJointLimitsTest)
{
  ros::NodeHandle nh;
  JointData j;

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  ros::Time start_time = ros::Time::now();
  while(j.joint_angles.empty() && ((ros::Time::now() - start_time) < ros::Duration(5.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(j.joint_angles.empty());

  std::vector<double> joint_lower_limits;
  joint_lower_limits.push_back(0.1);
  joint_lower_limits.push_back(0.8);

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

  for (size_t i = 0; i < joint_lower_limits.size(); i++)
    system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/joint_limits " +
            std::string(joint_names.at(i)) + "_lower_pos " +
            std::to_string(joint_lower_limits.at(i)))
               .c_str());

  ros::Publisher command = nh.advertise<sensor_msgs::JointState>(
      "/whole_body_kinematic_controller/reference_ref", 10);

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

  ros::Duration(2.0).sleep();
  ros::spinOnce();
  std::vector<double> desired_joint_angles(j.joint_angles);

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    if (desired_joint_angles[i] < joint_lower_limits[i])
      desired_joint_angles[i] = joint_lower_limits[i];
  }

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(desired_joint_angles[i], j.joint_angles[i], 1e-4);

  // increasing the lower limit of the joint1 to see the effect in the robot
  joint_lower_limits[0] = 1.0;
  for (size_t i = 0; i < joint_lower_limits.size(); i++)
    system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/joint_limits " +
            std::string(joint_names.at(i)) + "_lower_pos " +
            std::to_string(joint_lower_limits.at(i)))
               .c_str());

  ros::Duration(2.0).sleep();
  ros::spinOnce();
  desired_joint_angles = j.joint_angles;

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    if (desired_joint_angles[i] < joint_lower_limits[i])
      desired_joint_angles[i] = joint_lower_limits[i];
  }

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(desired_joint_angles[i], j.joint_angles[i], 1e-4);

  /// Setting back the lower limits for the next test
  for (size_t i = 0; i < joint_lower_limits.size(); i++)
    joint_lower_limits[i] = 0.0;

  for (size_t i = 0; i < joint_lower_limits.size(); i++)
    system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/joint_limits " +
            std::string(joint_names.at(i)) + "_lower_pos " +
            std::to_string(joint_lower_limits.at(i)))
               .c_str());

  ROS_INFO("Lower Joint Limit Configuration has been successfully achieved");
}

TEST(TopicTest, HigherJointLimitsTest)
{
  ros::NodeHandle nh;
  JointData j;

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  ros::Time start_time = ros::Time::now();
  while(j.joint_angles.empty() && ((ros::Time::now() - start_time) < ros::Duration(5.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(j.joint_angles.empty());

  std::vector<double> joint_upper_limits;
  joint_upper_limits.push_back(1.4);
  joint_upper_limits.push_back(0.2);

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

  for (size_t i = 0; i < joint_upper_limits.size(); i++)
    system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/joint_limits " +
            std::string(joint_names.at(i)) + "_upper_pos " +
            std::to_string(joint_upper_limits.at(i)))
               .c_str());

  ros::Publisher command = nh.advertise<sensor_msgs::JointState>(
      "/whole_body_kinematic_controller/reference_ref", 10);

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

  ros::Duration(3.0).sleep();
  ros::spinOnce();
  std::vector<double> desired_joint_angles(j.joint_angles);

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    if (desired_joint_angles[i] > joint_upper_limits[i])
      desired_joint_angles[i] = joint_upper_limits[i];
  }

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(desired_joint_angles[i], j.joint_angles[i], 1e-4);

  // decreasing the upper limit of the joint1 to see the effect in the robot
  joint_upper_limits[0] = 0.1;
  for (size_t i = 0; i < joint_upper_limits.size(); i++)
    system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/joint_limits " +
            std::string(joint_names.at(i)) + "_upper_pos " +
            std::to_string(joint_upper_limits.at(i)))
               .c_str());

  ros::Duration(2.0).sleep();
  ros::spinOnce();
  desired_joint_angles = j.joint_angles;

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    if (desired_joint_angles[i] > joint_upper_limits[i])
      desired_joint_angles[i] = joint_upper_limits[i];
  }

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(desired_joint_angles[i], j.joint_angles[i], 1e-4);
  ROS_INFO("Higher Joint Limit Configuration has been successfully achieved");
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dynamic_reconfigure_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
