# PAL Whole Body Controller Tutorial

This package is a tutorial for pal_wbc in order to help the users to create new whole body control tasks and run them on the robot.

### Prerequisites

In order to compile this package you vill need_

* ROS Kinetic installed on your computer.
* The pal_wbc package.
* The RRBOT robot ROS package.
* The [pluginlib](http://wiki.ros.org/pluginlib) ROS package

### Installing

Create a new [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Clone this repository on your workspace

```
git clone ...
```

And then compile it

```
catkin build
```

## Running the tutorials

In this tutorial, the RRBOT robot can be controller in two different ways:

1. **dynamic_reconfigure**
2. **ros topics**

To launch the robot to control in dynamic reconfigure mode, launch:
```bash
# To launch rrbot to be controlled by rqt_reconfigure
roslaunch pal_wbc_tutorial rrbot_wbc_dynamic_reconfigure.launch
```
Then the user can control the robot by using the slides of the rqt_joint_reconfigure. It's also possible to change online the joint limits from the robot using the rqt_reconfigure.

The second opstion is to control the robot giving a reference for each joint:
```bash
# To launch rrbot to be controlled by rostopic
roslaunch pal_wbc_tutorial rrbot_wbc_topic.launch
```
Then the user can control it by publishing on the topic **/whole_body_kinematic_controller/reference_ref**


## Creation of the new tasks

In this package two new whole body control tasks are created:

1. **Joint limit task**: to ensure joint limts.
2. **Reference task**: where we specify a reference task for the joints of the robot.

The **Joint limit task** shows the user how to create new tasks based on inequalities, while the **Reference task** shows the user how to create a new tasks based on equalities.

Since Whole Body Control is a PAL's implementation of the Stack of Tasks, which is based on optmization. All the new tasks have to be defined as constraints of the optimization problem.


## Run the tests

To run the diferents tests.

Compile the tests:

```
catkin build --make-args tests
```

And execute them 

```
rostest pal_wbc_tutorial pal_wbc_tutorial_ddynamic_reconfigure_test_one.test
```
```
rostest pal_wbc_tutorial pal_wbc_tutorial_ddynamic_reconfigure_test_two.test
```
```
rostest pal_wbc_tutorial pal_wbc_tutorial_pop_and_push_task.test
```
```
rostest pal_wbc_tutorial pal_wbc_tutorial_topic_test.test
```

## Authors

* **Sai Kishor**
* **Adrià Roig**

## License

This project is licensed under the Proprietary License.



