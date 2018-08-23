#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <dynamic_reconfigure/Reconfigure.h>
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

TEST(DynamicReconfigureTest, UpperJointLimitsTest)
{
  ros::NodeHandle nh;
  JointData j;

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

  std::vector<double> joint_upper_limits;
  joint_upper_limits.push_back(1.5);
  joint_upper_limits.push_back(0.2);

  for (size_t i = 0; i < joint_upper_limits.size(); i++)
    system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/joint_limits " +
            std::string(joint_names.at(i)) + "_upper_pos " +
            std::to_string(joint_upper_limits.at(i)))
               .c_str());

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  ros::Time start_time = ros::Time::now();
  while(j.joint_angles.empty() && ((ros::Time::now() - start_time) < ros::Duration(5.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(j.joint_angles.empty());

  ros::Duration(5.0).sleep();
  ros::spinOnce();
  std::vector<double> desired_joint_angles;
  desired_joint_angles.resize(joint_names.size());

  if (nh.hasParam("/whole_body_kinematic_controller/default_configuration"))
  {
    ROS_INFO("Getting reference from param server");
    nh.getParam("/whole_body_kinematic_controller/default_configuration/single_rrbot_joint1",
                desired_joint_angles[0]);
    nh.getParam("/whole_body_kinematic_controller/default_configuration/single_rrbot_joint2",
                desired_joint_angles[1]);
  }

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    if (desired_joint_angles[i] > joint_upper_limits[i])
      desired_joint_angles[i] = joint_upper_limits[i];
  }

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(desired_joint_angles[i], j.joint_angles[i], 1e-3);

  // increasing the lower limit of the joint1 to see the effect in the robot
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

  ROS_INFO("Upper Joint Limit Configuration has been successfully achieved");

  // setting back the limits to the desirable range
  joint_upper_limits[0] = 1.57;
  joint_upper_limits[1] = 1.57;

  for (size_t i = 0; i < joint_upper_limits.size(); i++)
    system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/joint_limits " +
            std::string(joint_names.at(i)) + "_upper_pos " +
            std::to_string(joint_upper_limits.at(i)))
               .c_str());
}

TEST(DynamicReconfigureTest, ChangingEachAngleParameters)
{
  ros::NodeHandle nh;
  JointData j;

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  ros::Time start_time = ros::Time::now();
  while(j.joint_angles.empty() && ((ros::Time::now() - start_time) < ros::Duration(5.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(j.joint_angles.empty());

  std::vector<double> desired_joint_angles;
  desired_joint_angles.push_back(1.2);
  desired_joint_angles.push_back(1.0);

  system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/reference " +
          std::string(joint_names.at(0)) + " " + std::to_string(desired_joint_angles.at(0)))
             .c_str());

  ros::Duration(5.0).sleep();
  ros::spinOnce();

  std::vector<double> actual_joint_angles(j.joint_angles);

  if (nh.hasParam("/whole_body_kinematic_controller/default_configuration"))
  {
    ROS_INFO("Getting reference from param server");
    nh.getParam("/whole_body_kinematic_controller/default_configuration/single_rrbot_joint2",
                desired_joint_angles[1]);
  }

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(desired_joint_angles[i], j.joint_angles[i], 1e-3);

  desired_joint_angles[1] = 0.5;
  system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/reference " +
          std::string(joint_names.at(1)) + " " + std::to_string(desired_joint_angles.at(1)))
             .c_str());

  ros::Duration(5.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(desired_joint_angles[i], j.joint_angles[i], 1e-3);
}

TEST(DynamicReconfigureTest, ChangingAngleParameters)
{
  ros::NodeHandle nh;
  JointData j;

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

  ros::Subscriber joint_state_sub =
      nh.subscribe("/joint_states", 10, &JointData::callback, &j);

  ros::Time start_time = ros::Time::now();
  while(j.joint_angles.empty() && ((ros::Time::now() - start_time) < ros::Duration(5.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(j.joint_angles.empty());

  std::vector<double> desired_joint_angles;
  desired_joint_angles.push_back(0.8);
  desired_joint_angles.push_back(0.8);

  for (size_t i = 0; i < desired_joint_angles.size(); i++)
  {
    system(("rosrun dynamic_reconfigure dynparam set /whole_body_kinematic_controller/reference " +
            std::string(joint_names.at(i)) + " " + std::to_string(desired_joint_angles.at(i)))
               .c_str());
  }

  ros::Duration(5.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < joint_names.size(); i++)
    ASSERT_NEAR(desired_joint_angles[i], j.joint_angles[i], 1e-3);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dynamic_reconfigure_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
