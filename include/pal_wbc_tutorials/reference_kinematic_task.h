/**
 * Copyright (C) 2019 PAL Robotics S.L.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 **/
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
 * @brief The ReferenceKinematicTaskRRBot class defenides a goal configuration for a set
 * of
 * joints
 */
class ReferenceKinematicTaskRRBot
    : public TaskAbstractWithParameters<ReferenceKinematicTaskRRBotParam, StackOfTasksKinematic>
{
public:
  ReferenceKinematicTaskRRBot();

  virtual ~ReferenceKinematicTaskRRBot();

  bool reconfigureTask();
  bool configureTask(ros::NodeHandle &nh) override;
  bool configureConstraintsFromPropertyBag(ros::NodeHandle &nh, StackOfTasksKinematic *st,
                                            const property_bag::PropertyBag &properties,
                                            ReferenceKinematicTaskRRBotParam *parameters) override;
  bool start() override;
  void update(const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot, const ros::Time &time) override;
  bool stop() override;
  void debug(const Eigen::VectorXd &solution, const ros::Time &time);
  virtual std::string getType() override
  {
    return demangledTypeName(this);
  }

private:
  std::vector<int> jointIndex_;

  Eigen::VectorXd desiredPosition_;
  Eigen::VectorXd error_;

  boost::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> dd_reconfigure_;

  pal_robot_tools::PluginlibHelperPtr<pal_robot_tools::VectorReferenceAbstract> reference_loader_;

  pal_statistics::RegistrationsRAII registered_variables_;
};

typedef boost::shared_ptr<ReferenceKinematicTaskRRBot> ReferenceKinematicTaskRRBotPtr;


/**
 * @brief The ReferenceKinematicTaskRRBotAllJointsMetaTask class is a helper class to
 * configure
 * the
 * ReferenceKinematic task
 */
class ReferenceKinematicTaskRRBotAllJointsMetaTask : public ReferenceKinematicTaskRRBot
{
public:
  virtual ~ReferenceKinematicTaskRRBotAllJointsMetaTask()
  {
  }

  ReferenceKinematicTaskRRBotAllJointsMetaTask(const std::string &task_id,
                                               StackOfTasksKinematic &st,
                                               const std::vector<std::string> &joint_names,
                                               pal_robot_tools::VectorReferenceAbstractPtr reference,
                                               double gain, ros::NodeHandle &nh);

  ReferenceKinematicTaskRRBotAllJointsMetaTask(const std::string &task_id,
                                               StackOfTasksKinematic &st,
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
