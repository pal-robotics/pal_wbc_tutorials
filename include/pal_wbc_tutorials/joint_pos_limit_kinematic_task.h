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
#ifndef _JOINT_POS_LIMIT_KINEMATIC_TASK_
#define _JOINT_POS_LIMIT_KINEMATIC_TASK_

#include <pal_wbc_controller/task_abstract.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace pal_wbc
{
class StackOfTasksKinematic;

struct JointPositionLimitParams
{
  virtual ~JointPositionLimitParams()
  {
  }
  std::vector<std::string> names;
  std::vector<Bound::bound_t> bound_type;
  std::vector<double> upper_bound_position;
  std::vector<double> lower_bound_position;
  std::vector<double> upper_bound_velocity;
  std::vector<double> lower_bound_velocity;
  double vel_limit_gain;
  bool disable_vel_limit;

  /// @todo remove this constructor
  JointPositionLimitParams()
  {
    vel_limit_gain = 1.0;
    disable_vel_limit = false;
  }

  JointPositionLimitParams(std::vector<std::string> names, std::vector<Bound::bound_t> bound_type,
                           std::vector<double> upper_bound_position,
                           std::vector<double> lower_bound_position,
                           std::vector<double> upper_bound_velocity,
                           std::vector<double> lower_bound_velocity,
                           double vel_limit_gain, double disable_vel_limit)
  {
    this->names = names;
    this->bound_type = bound_type;
    this->upper_bound_position = upper_bound_position;
    this->lower_bound_position = lower_bound_position;
    this->upper_bound_velocity = upper_bound_velocity;
    this->lower_bound_velocity = lower_bound_velocity;
    this->vel_limit_gain = vel_limit_gain;
    this->disable_vel_limit = disable_vel_limit;
  }
};
/**
*@brief The JointPositionLimitTask class enforces postion and velocity limits as
* inequality
*constraints. The velocity limit can be disabled or scaled
*/
class JointPositionLimitTask
    : public TaskAbstractWithParameters<JointPositionLimitParams, StackOfTasksKinematic>
{
public:
  JointPositionLimitTask();

  virtual ~JointPositionLimitTask()
  {
  }

  bool reconfigureTask() override;
  bool  configureTask(ros::NodeHandle &nh) override;
  void update(const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot, const ros::Time &time);
  void debug(const Eigen::VectorXd &solution, const ros::Time &time);
  virtual std::string getType() override
  {
    return demangledTypeName(this);
  }

private:
  boost::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> dd_reconfigure_;
};

typedef boost::shared_ptr<JointPositionLimitTask> JointPositionLimitTaskPtr;


/**
*@brief The JointPositionLimitKinematicAllJointsMetaTask class is a helper class of the
*JointPositionLimitKinematic task
*/
class JointPositionLimitKinematicAllJointsMetaTask : public JointPositionLimitTask
{
public:
  virtual ~JointPositionLimitKinematicAllJointsMetaTask()
  {
  }

  JointPositionLimitKinematicAllJointsMetaTask(const std::string &task_id, StackOfTasksKinematic &st, std::vector<double> joint_min_positoin,
                                               std::vector<double> joint_max_position, std::vector<double> joint_min_velocity,
                                               std::vector<double> joint_max_velocity, std::vector<std::string> joint_names,
                                               double vel_limit_gain, bool disable_vel_limit, ros::NodeHandle &nh);
};

typedef boost::shared_ptr<JointPositionLimitKinematicAllJointsMetaTask> JointPositionLimitKinematicAllJointsMetaTaskPtr;
};

#endif
