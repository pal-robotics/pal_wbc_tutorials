#include <pal_wbc_tutorials/joint_pos_limit_kinematic_task.h>
#include <pal_wbc_controller/stack_of_tasks_kinematic.h>
#include <iostream>
#include <pal_utils/formater.h>
#include <pal_utils/exception_utils.h>

/// Change to configure for selective joints

namespace pal_wbc
{
JointPositionLimitTask::JointPositionLimitTask()
{
}

bool JointPositionLimitTask::reconfigureTask()
{
  StackOfTasks *st = getStackOfTasks();
  JointPositionLimitParams *parameters = getParameters();

  level_.J_.resize(parameters->names.size(), getStateSize());
  level_.J_.setZero();
  level_.bounds_.resize(parameters->names.size());

  if ((st->getFormulationType() == +formulation_t::velocity))
  {
    if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::XYZ_Quaternion)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        level_.J_(i, i + 6) = 1.0;
      }
    }
    else if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::XY_Yaw)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        level_.J_(i, i + 3) = 1.0;
      }
    }
    else if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::FixedBase)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        level_.J_(i, i) = 1.0;
      }
    }
    else
    {
      PAL_THROW("Floating base type not supported");
      return false;
    }
  }
  else
  {
    PAL_THROW("Formulation not supported");
    return false;
  }

  return true;
}

/**
 * @brief JointPositionLimitTask::setUpTask, Sets up the task from the passed parameters
 * this function usually defines the structure of the task
 * @param ct - JointPositionLimitParams is the structure we define in the header, that
 * contains the paramters that are required to run the task, Parameters of the Joint
 * Limits(min and max are defined)
 * @param st - Prameter that contains the info about the Stack of Tasks
 * @return Returns true or false based on the status of setting up the tasks
 */
bool JointPositionLimitTask::configureTask(ros::NodeHandle &nh)
{
  ROS_DEBUG_STREAM("Setting up joint limit task");

  JointPositionLimitParams *parameters = getParameters();

  if (!reconfigureTask())
  {
    return false;
  }

  dd_reconfigure_.reset(
        new ddynamic_reconfigure::DDynamicReconfigure(ros::NodeHandle(nh, "joint_limits")));
  dd_reconfigure_->RegisterVariable(&parameters->vel_limit_gain, "vel_limit_gain");
  dd_reconfigure_->RegisterVariable(&parameters->disable_vel_limit, "disable_vel_"
                                                                     "limit_");
  for (unsigned int i = 0; i < parameters->names.size(); ++i)
  {
    dd_reconfigure_->RegisterVariable(&parameters->lower_bound_position[i],
                                      std::string(parameters->names[i] + "_lower_pos"), -3.14, 3.14);
    dd_reconfigure_->RegisterVariable(&parameters->upper_bound_position[i],
                                      std::string(parameters->names[i] + "_upper_pos"), -3.14, 3.14);
    dd_reconfigure_->RegisterVariable(&parameters->lower_bound_velocity.data()[i],
                                      parameters->names[i] + "_lower_vel", -1000, 1000);
    dd_reconfigure_->RegisterVariable(&parameters->upper_bound_velocity.data()[i],
                                      parameters->names[i] + "_upper_vel", -1000, 1000);
  }
  dd_reconfigure_->PublishServicesTopics();

  return true;
}

/**
 * @brief JointPositionLimitTask::update, this function is the one that runs frequently in
 * a task and usually the process of the task is done in this method
 * @param Q - Position values (joint angle)
 * @param QDot - Velocity Values (joint velocity)
 * @param time - ros::Time
 */
void JointPositionLimitTask::update(const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot,
                                    const ros::Time &time)
{
  StackOfTasks *st = getStackOfTasks();
  JointPositionLimitParams *parameters = getParameters();

  PAL_ASSERT_PERSIST_EQUAL(st->getModel()->q_size, Q.rows());

  if ((st->getFormulationType() == +formulation_t::velocity))
  {
    double k = 1.0 / st->getDt().toSec();
    if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::XYZ_Quaternion)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        double min_pos_boud = k * (parameters->lower_bound_position[i] - Q(i + 6));
        double max_pos_boud = k * (parameters->upper_bound_position[i] - Q(i + 6));

        if (!parameters->disable_vel_limit)
        {
          if (max_pos_boud > parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            max_pos_boud = parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (max_pos_boud < -parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            max_pos_boud = -parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (min_pos_boud < -parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            min_pos_boud = -parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (min_pos_boud > parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            min_pos_boud = parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }
        }


        level_.bounds_(i) = Bound(min_pos_boud, max_pos_boud);
      }
    }
    else if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::FixedBase)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        double min_pos_boud = k * (parameters->lower_bound_position[i] - Q(i));
        double max_pos_boud = k * (parameters->upper_bound_position[i] - Q(i));

        if (!parameters->disable_vel_limit)
        {
          if (max_pos_boud > parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            max_pos_boud = parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (max_pos_boud < -parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            max_pos_boud = -parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (min_pos_boud < -parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            min_pos_boud = -parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (min_pos_boud > parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            min_pos_boud = parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }
        }

        level_.bounds_(i) = Bound(min_pos_boud, max_pos_boud);
        ;
      }
    }
    else if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::XY_Yaw)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        double min_pos_boud = k * (parameters->lower_bound_position[i] - Q(i + 3));
        double max_pos_boud = k * (parameters->upper_bound_position[i] - Q(i + 3));


        if (!parameters->disable_vel_limit)
        {
          if (max_pos_boud > parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            max_pos_boud = parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (max_pos_boud < -parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            max_pos_boud = -parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (min_pos_boud < -parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            min_pos_boud = -parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }

          if (min_pos_boud > parameters->vel_limit_gain * parameters->upper_bound_velocity[i])
          {
            min_pos_boud = parameters->vel_limit_gain * parameters->upper_bound_velocity[i];
          }
        }

        level_.bounds_(i) = Bound(min_pos_boud, max_pos_boud);
      }
    }
    else
    {
      PAL_THROW("Floating base type not supported");
    }
  }
  else
  {
    PAL_THROW("Forumulation type not supported");
  }
}

void JointPositionLimitTask::debug(const Eigen::VectorXd &solution, const ros::Time &time)
{
}

/**
 * @brief
 * JointPositionLimitKinematicAllJointsMetaTask::JointPositionLimitKinematicAllJointsMetaTask,
 * This is a metaTask method that is useful for the user to configure his task, In simple
 * words, it is an instantiation of a Task with a particular configuration.
 * @param st - Prameter that contains the info about the Stack of Tasks
 * @param joint_min_position - joint min. position values restrictions for each joint
 * @param joint_max_position - joint max. position values restrictions for each joint
 * @param joint_min_velocity - joint min. velocity values restrictions for each joint
 * @param joint_max_velocity - joint max. velocity values restrictions for each joint
 * @param joint_names - joint names of the robot
 * @param vel_limit_gain - velocit limit gain parameter
 * @param disable_vel_limit - parameter to choose whether to enable the velocity limit or
 * not.
 * @param nh - ros node handler
 */
JointPositionLimitKinematicAllJointsMetaTask::JointPositionLimitKinematicAllJointsMetaTask(const std::string &task_id,
                                                                                           StackOfTasksKinematic &st, std::vector<double> joint_min_position,
                                                                                           std::vector<double> joint_max_position, std::vector<double> joint_min_velocity,
                                                                                           std::vector<double> joint_max_velocity, std::vector<std::string> joint_names,
                                                                                           double vel_limit_gain, bool disable_vel_limit, ros::NodeHandle &nh)
{
  PAL_ASSERT_PERSIST_EQUAL(joint_names.size(), joint_min_position.size());
  PAL_ASSERT_PERSIST_EQUAL(joint_names.size(), joint_max_position.size());
  PAL_ASSERT_PERSIST_EQUAL(joint_names.size(), joint_min_velocity.size());
  PAL_ASSERT_PERSIST_EQUAL(joint_names.size(), joint_max_velocity.size());

  ROS_DEBUG_STREAM("Creating JointPositionLimitKinematicAllJointsMetaTask");

  JointPositionLimitParams joint_limit_params;
  joint_limit_params.vel_limit_gain = vel_limit_gain;
  joint_limit_params.disable_vel_limit = disable_vel_limit;

  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    joint_limit_params.names.push_back(joint_names[i]);
    joint_limit_params.lower_bound_position.push_back(joint_min_position[i]);
    joint_limit_params.upper_bound_position.push_back(joint_max_position[i]);
    joint_limit_params.lower_bound_velocity.push_back(joint_min_velocity[i]);
    joint_limit_params.upper_bound_velocity.push_back(joint_max_velocity[i]);
    joint_limit_params.bound_type.push_back(Bound::BOUND_DOUBLE);
  }

  if (!this->setUp(task_id, joint_limit_params, &st, nh))
  {
    PAL_THROW_DEFAULT("problem configuring task");
  };

}
}
