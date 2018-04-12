///////////////////////////////////////////////////////////////////////////////

// Copyright (C) 2014, 2015 PAL Robotics S.L.

// All rights reserved.

//////////////////////////////////////////////////////////////////////////////

// Author Hilario Tomé

#ifndef _REFERNCE_KINEMATIC_TASK_
#define _REFERNCE_KINEMATIC_TASK_

#include <pal_wbc_controller/task_abstract.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <pal_ros_utils/reference/vector/vector_reference_abstract.h>
#include <pal_ros_utils/pluginlib_helpers.h>

namespace pal_wbc
{
class StackOfTasksKinematic;

struct ReferenceKinematicTaskRRBotParam
{
  std::vector<std::string> names;
  pal_robot_tools::VectorReferenceAbstractPtr reference;
  double p_pos_gain;
//  std::vector<double> joint_angles;

  ReferenceKinematicTaskRRBotParam()
  {
    p_pos_gain = 1.5;
//    joint_angles.resize(1);
//    joint_angles.at(0) = 0.0;
  }
};

/**
 * @brief The ReferenceKinematicTaskRRBot class defenides a goal configuration for a set of
 * joints
 */
class ReferenceKinematicTaskRRBot
    : public TaskAbstractWithParameters<ReferenceKinematicTaskRRBotParam, StackOfTasksKinematic>
{
public:
  ReferenceKinematicTaskRRBot();

  virtual ~ReferenceKinematicTaskRRBot();

  bool setUpTask();
  bool setUpTask(const ReferenceKinematicTaskRRBotParam &ct, StackOfTasksKinematic &st,
                 ros::NodeHandle &nh) override;
  bool setUpTaskPropertyBag(const property_bag::PropertyBag &properties,
                            StackOfTasksKinematic &st, ros::NodeHandle &nh) override;
  bool startTask() override;
  void update(const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot, const ros::Time &time) override;
  bool stopTask() override;
  void debug(const Eigen::VectorXd &solution, const ros::Time &time);
  virtual std::string getType() override
  {
    return demangledTypeName(this);
  }

private:
  RigidBodyDynamics::Model *model_;
  int nDof_;
  int nState_;
  std::vector<int> jointIndex_;

  Eigen::VectorXd desiredPosition_;
  Eigen::VectorXd error_;

  boost::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> dd_reconfigure_;

  pal_robot_tools::PluginlibHelperPtr<pal_robot_tools::VectorReferenceAbstract> reference_loader_;

  std::vector<std::string> registered_variables_;
};

typedef boost::shared_ptr<ReferenceKinematicTaskRRBot> ReferenceKinematicTaskRRBotPtr;


/**
 * @brief The ReferenceKinematicTaskRRBotAllJointsMetaTask class is a helper class to configure
 * the
 * ReferenceKinematic task
 */
class ReferenceKinematicTaskRRBotAllJointsMetaTask : public ReferenceKinematicTaskRRBot
{
public:
  virtual ~ReferenceKinematicTaskRRBotAllJointsMetaTask()
  {
  }

  ReferenceKinematicTaskRRBotAllJointsMetaTask(StackOfTasksKinematic &st,
                                          const std::vector<std::string> &joint_names,
                                          pal_robot_tools::VectorReferenceAbstractPtr reference,
                                          double gain, ros::NodeHandle &nh);

  ReferenceKinematicTaskRRBotAllJointsMetaTask(StackOfTasksKinematic &st,
                                          const std::vector<std::string> &joint_names,
                                          const Eigen::VectorXd &referencePosition,
                                          ros::NodeHandle &nh, const double &gain);
  virtual std::string getType()
  {
    return demangledTypeName(this);
  }
};

typedef boost::shared_ptr<ReferenceKinematicTaskRRBotAllJointsMetaTask> ReferenceKinematicTaskRRBotAllJointsMetaTaskPtr;
};

#endif
