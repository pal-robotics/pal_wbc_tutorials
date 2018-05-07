#include <eigen3/Eigen/Core>
#include <pal_wbc_utils/pal_wbc_utils.h>
#include <pal_utils/permutation.h>
#include <property_bag/serialization/eigen_boost_serialization.h>
#include <property_bag/serialization/ros_boost_serialization.h>
#include <property_bag/serialization/property_boost_serialization.h>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_reference_pos_task");
  ros::NodeHandle nh;

  // Service Helper that helps us to push the task
  pal::WBCServiceHelper srv_helper(nh);

  property_bag::PropertyBag task;
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  // Adding the properties required by the task into the property bag
  task.addProperty("p_pos_gain", 2.0);

  task.addProperty("taskType", std::string("pal_wbc/ReferenceKinematicTaskRRBot"));
  task.addProperty("task_id", std::string("rrbot_default_reference"));

  std::vector<std::string> joint_names;
  joint_names.push_back("single_rrbot_joint1");
  joint_names.push_back("single_rrbot_joint2");

  task.addProperty("joint_names", joint_names);

  // In this case, the signal reference is "vector_topic", so that we can control joint
  // angles by rostopics
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

  // pushing the task on to the stack
  srv_helper.pushTask(task, "rrbot_default_reference", order, "rrbot_joint_limits");

  // publisher to publish the joint angles
  ros::Publisher command = nh.advertise<sensor_msgs::JointState>(
      "/whole_body_kinematic_controller/reference_ref", 10);

  //// msg to publish the angles in rostopics
  //    ros::Duration(10).sleep();
  //    sensor_msgs::JointState msg;
  //    msg.header.stamp = ros::Time::now();
  //    msg.name.resize(joint_names.size());
  //    msg.name = joint_names;
  //    msg.position.resize(joint_names.size());
  //    msg.position[0] = 1.0;
  //    msg.position[1] = 1.57;
  //    msg.velocity.resize(joint_names.size());
  //    msg.effort.resize(joint_names.size());

  //    command.publish(msg);

  ros::spin();
}
