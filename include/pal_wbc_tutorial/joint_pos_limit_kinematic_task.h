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

  bool setUpTask();
  bool setUpTask(const JointPositionLimitParams &ct, StackOfTasksKinematic &st,
                 ros::NodeHandle &nh);
  void update(const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot, const ros::Time &time);
  void debug(const Eigen::VectorXd &solution, const ros::Time &time);
  virtual std::string getType() override
  {
    return demangledTypeName(this);
  }

private:
  boost::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> dd_reconfigure_;
  RigidBodyDynamics::Model *model_;
  int nDof_;
  int nState_;
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

  JointPositionLimitKinematicAllJointsMetaTask(
      StackOfTasksKinematic &st, std::vector<double> joint_min_positoin,
      std::vector<double> joint_max_position, std::vector<double> joint_min_velocity,
      std::vector<double> joint_max_velocity, std::vector<std::string> joint_names,
      double vel_limit_gain, bool disable_vel_limit, ros::NodeHandle &nh);
};

typedef boost::shared_ptr<JointPositionLimitKinematicAllJointsMetaTask> JointPositionLimitKinematicAllJointsMetaTaskPtr;
};

#endif
