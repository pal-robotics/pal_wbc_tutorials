#include <pal_wbc_tutorials/reference_kinematic_task.h>
#include <pal_wbc_controller/stack_of_tasks_kinematic.h>
#include <pal_ros_utils/reference/pose/pose_reference_abstract.h>
#include <pal_ros_utils/reference/vector/vector_pointer_reference.h>
#include <pal_ros_utils/reference/vector/vector_reference_dynamic_reconfigure.h>
#include <dynamic_introspection/dynamic_introspection.h>
#include <pal_utils/exception_utils.h>
#include <pal_utils/permutation.h>

using namespace pal_robot_tools;

namespace pal_wbc
{
ReferenceKinematicTaskRRBot::ReferenceKinematicTaskRRBot()
  : reference_loader_(new PluginlibHelper<VectorReferenceAbstract>(
        "pal_ros_utils", "pal_robot_tools::VectorReferenceAbstract"))
{
}

ReferenceKinematicTaskRRBot::~ReferenceKinematicTaskRRBot()
{
  reference_loader_.reset();
  dd_reconfigure_.reset();
}

bool ReferenceKinematicTaskRRBot::reconfigureTask()
{
  ReferenceKinematicTaskRRBotParam *parameters = getParameters();
  StackOfTasks *st = getStackOfTasks();

  if (parameters->names.size() <= 0)
  {
    PAL_THROW("the number of constraints should be bigger than zero");
  }

  level_.J_.resize(parameters->names.size(), getStateSize());
  level_.bounds_.resize(parameters->names.size());
  level_.J_.setZero();
  this->jointIndex_.resize(parameters->names.size());

  error_.resize(parameters->names.size());
  error_.setZero();

  if (st->getFormulationType() == +formulation_t::velocity)
  {
    if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::XYZ_Quaternion)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        unsigned int index = st->getJointIndex(parameters->names[i]);
        jointIndex_[i] = index;
        level_.J_(i, index + 6) = 1.0;
      }
    }
    else if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::XY_Yaw)
    {
      this->jointIndex_.resize(parameters->names.size());
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        unsigned int index = st->getJointIndex(parameters->names[i]);
        jointIndex_[i] = index;
        level_.J_(i, index + 3) = 1.0;
      }
    }
    else if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::FixedBase)
    {
      this->jointIndex_.resize(parameters->names.size());
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        unsigned int index = st->getJointIndex(parameters->names[i]);
        jointIndex_[i] = index;
        level_.J_(i, index) = 1.0;
      }
    }
    else
    {
      PAL_THROW("floating base type not supported");
    }
  }
  else
  {
    PAL_THROW("formulation not supported");
  }

  return true;
}

/**
 * @brief ReferenceKinematicTaskRRBot::setUpTask, This method helps us to setup the
 * paramter of the task and This method usually defines the variables and intializes them
 * to a particular value and this is the place where the variables gets linked with the
 * ddynamic reconfigure parameters
 * @param ct - ReferenceKinematicTaskRRBotParam is the structure we define in the header,
 * that
 * contains the paramters that are required to run the task.
 * @param st - Prameter that contains the info about the Stack of Tasks
 * @param nh - ros node handler
 * @return returns true or false based on the status of setting up the task.
 */
bool ReferenceKinematicTaskRRBot::configureTask(ros::NodeHandle &nh)
{
  if (!reconfigureTask())
  {
    return false;
  }

  ReferenceKinematicTaskRRBotParam *parameters = getParameters();
  desiredPosition_.resize(parameters->names.size());
  desiredPosition_.setZero();

  error_.resize(parameters->names.size());

  std::string service_name =
      nh.getNamespace() + "/reference_posture_" + parameters->names[0] + "/set_parameters";

  /// @todo hack to avoid repeated names in ddynamic reconfigure
  dd_reconfigure_.reset(new ddynamic_reconfigure::DDynamicReconfigure(
      ros::NodeHandle(nh, "reference_posture_" + parameters->names[0])));
  //    for(unsigned int i=0;i <parameters->names.size(); ++i){
  //      dd_reconfigure_->RegisterVariable(&parameters->reference_posture.data()[i],
  //      parameters->names[i], -M_PI, M_PI);
  //    }
  dd_reconfigure_->RegisterVariable(&parameters->p_pos_gain, "proportional_position_e_"
                                                             "gain");
  dd_reconfigure_->RegisterVariable(level_.getRegularizationPtr(), "regularization", 0, 10);
  dd_reconfigure_->RegisterVariable(level_.getWeightPtr(), "weight", 0, 1000);

  dd_reconfigure_->PublishServicesTopics();

  return true;
}

/**
 * @brief ReferenceKinematicTaskRRBot::startTask, method that is used to start the Task
 * @return
 */
bool ReferenceKinematicTaskRRBot::start()
{
  return true;
}

/**
 * @brief ReferenceKinematicTaskRRBot::setUpTaskPropertyBag, Retrieves the info from the
 * property bag and calls the Setup method to setup the task with the corresponding
 * defined properties
 * @param properties - Propertybag parameter
 * @param st - Prameter that contains the info about the Stack of Tasks
 * @param nh - ros node handler
 * @return returns the status of the task setup
 */
bool ReferenceKinematicTaskRRBot::configureConstraintsFromPropertyBag(
    ros::NodeHandle &nh, StackOfTasksKinematic *st,
    const property_bag::PropertyBag &properties, ReferenceKinematicTaskRRBotParam *parameters)
{
  if (properties.exists("p_pos_gain"))
  {
    properties.getPropertyValue<double>("p_pos_gain", parameters->p_pos_gain);
  }

  properties.getPropertyValue<std::vector<std::string> >(
      "joint_names", parameters->names, property_bag::RetrievalHandling::THROW);

  std::string signal_reference;
  properties.getPropertyValue("signal_reference", signal_reference,
                              property_bag::RetrievalHandling::THROW);

  std::vector<std::string> joint_names_model = st->getJointNames();
  std::vector<double> joint_limit_min_model = st->getJointPositionLimitMin();
  std::vector<double> joint_limit_max_model = st->getJointPositionLimitMax();

  std::vector<double> joint_min;
  std::vector<double> joint_max;
  for (size_t i = 0; i < parameters->names.size(); ++i)
  {
    int index = indexVector(joint_names_model, parameters->names[i]);
    if (index < 0)
    {
      ROS_ERROR_STREAM("joint: " << parameters->names[i] << " not found in the robot model");
    }
    joint_min.push_back(joint_limit_min_model[index]);
    joint_max.push_back(joint_limit_max_model[index]);
  }

  Eigen::VectorXd referencePosition;
  properties.getPropertyValue<Eigen::VectorXd>("reference", referencePosition,
                                               property_bag::RetrievalHandling::THROW);

  PAL_ASSERT_PERSIST_EQUAL(referencePosition.rows(), parameters->names.size());

  parameters->reference = reference_loader_->load(signal_reference);

  if (!parameters->reference->configure(nh, parameters->names.size(), "reference",
                            property_bag::PropertyBag("joint_names", parameters->names, "joint_min",
                                                      joint_min, "joint_max", joint_max, "target_state", referencePosition)))
    return false;

  parameters->reference->setPositionTarget(referencePosition);
  return true;
}

/**
 * @brief ReferenceKinematicTaskRRBot::update, this function is the one that runs
 * frequently in a task and usually the process of the task is done in this method
 * @param Q - Position values (Joint Angles)
 * @param QDot - Velocity values (Joint Velocity)
 * @param time - ros::time
 */
void ReferenceKinematicTaskRRBot::update(const Eigen::VectorXd &Q,
                                         const Eigen::VectorXd &QDot, const ros::Time &time)
{
  StackOfTasks *st = getStackOfTasks();
  ReferenceKinematicTaskRRBotParam *parameters = getParameters();

  PAL_ASSERT_PERSIST_EQUAL(st->getModel()->q_size, Q.rows());

  parameters->reference->integrate(time, st->getDt());
  parameters->reference->getPositionTarget(desiredPosition_);

  if (st->getFormulationType() == +formulation_t::velocity)
  {
    /// @todo put as parameters
    if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::XYZ_Quaternion)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        unsigned int index = jointIndex_[i];
        error_(i) = parameters->p_pos_gain * (desiredPosition_(i) - Q(index + 6));
        if (error_(i) > 1.)
        {
          error_(i) = 1;
        }
        if (error_(i) < -1)
        {
          error_(i) < -1;
        }
        level_.bounds_(i) = Bound(error_(i));
      }
    }
    else if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::XY_Yaw)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        unsigned int index = jointIndex_[i];
        error_(i) = parameters->p_pos_gain * (desiredPosition_(i) - Q(index + 3));
        level_.bounds_(i) = Bound(error_(i));
      }
    }
    else if (st->getFloatingBaseType() == +RigidBodyDynamics::FloatingBaseType::FixedBase)
    {
      for (size_t i = 0; i < parameters->names.size(); ++i)
      {
        unsigned int index = jointIndex_[i];
        error_(i) = parameters->p_pos_gain * (desiredPosition_(i) - Q(index));
        level_.bounds_(i) = Bound(error_(i));
      }
    }
    else
    {
      PAL_THROW("floating base type not supported");
    }
  }
  else
  {
    PAL_THROW("Forumulation type not supported");
  }
}

void ReferenceKinematicTaskRRBot::debug(const Eigen::VectorXd &solution, const ros::Time &time)
{
}

/**
 * @brief ReferenceKinematicTaskRRBot::stopTask, method to stop a task
 * @return returns the status of the request
 */
bool ReferenceKinematicTaskRRBot::stop()
{
  return true;
}

/// Metatasks
/**
 * @brief
 * ReferenceKinematicTaskRRBotAllJointsMetaTask::ReferenceKinematicTaskRRBotAllJointsMetaTask,
 * This
 * is a metaTask method that is useful for the user to configure his task, In simple
 * words, it is an instantiation of a Task with a particular configuration.
 * @param st - Prameter that contains the info about the Stack of Tasks
 * @param joint_names - joint names of the robot
 * @param reference - vector reference of the joint angles
 * @param gain - proportional position gain
 * @param nh - ros node handler
 */
ReferenceKinematicTaskRRBotAllJointsMetaTask::ReferenceKinematicTaskRRBotAllJointsMetaTask(
    const std::string &task_id, pal_wbc::StackOfTasksKinematic &st,
    const std::vector<std::string> &joint_names,
    pal_robot_tools::VectorReferenceAbstractPtr reference, double gain, ros::NodeHandle &nh)
{
  ReferenceKinematicTaskRRBotParam param;
  param.p_pos_gain = gain;
  param.names = joint_names;
  param.reference = reference;
  if (!this->setUp(task_id, param, &st, nh))
  {
    PAL_THROW_DEFAULT("problem configuring task");
  }
}

/**
 * @brief
 * ReferenceKinematicTaskRRBotAllJointsMetaTask::ReferenceKinematicTaskRRBotAllJointsMetaTask,
 * This is a metaTask method that is useful for the user to configure his task, In simple
 * words, it is an instantiation of a Task with a particular configuration.
 * @param st - Prameter that contains the info about the Stack of Tasks
 * @param joint_names - joint names of the robot
 * @param referencePosition - reference position in which the robot needs to be held
 * @param nh - ros node handler
 * @param gain - proportional position gain
 */
ReferenceKinematicTaskRRBotAllJointsMetaTask::ReferenceKinematicTaskRRBotAllJointsMetaTask(
    const std::string &task_id, StackOfTasksKinematic &st,
    const std::vector<std::string> &joint_names, const Eigen::VectorXd &referencePosition,
    ros::NodeHandle &nh, const double &gain)
{
  PAL_ASSERT_PERSIST_EQUAL(joint_names.size(), referencePosition.rows());

  ReferenceKinematicTaskRRBotParam param;
  param.p_pos_gain = gain;
  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    param.names.push_back(joint_names[i]);
  }
  pal_robot_tools::VectorReferenceAbstractPtr reference(
      new pal_robot_tools::VectorPointerReference(nh, joint_names.size(), "reference"));

  reference->setPositionTarget(referencePosition);
  param.reference = reference;
  if (!this->setUp(task_id, param, &st, nh))
  {
    PAL_THROW_DEFAULT("problem configuring task");
  }
}
}
