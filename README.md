# PAL Whole Body Controller Tutorial Package

This tutorial is made with RRBot robot model. The whole tutorial package runs on kinematic simulaotor rviz. In this tutorial, the robot can be controller in two different ways:

1. **dynamic_reconfigure**
2. **ros topics**

To launch the robot to control in dynamic reconfigure mode, launch:
```bash
# To launch rrbot to be controlled by rqt_reconfigure
roslaunch pal_wbc_tutorial rrbot_wbc_dynamic_reconfigure.launch
```

and, to control the robot using rostopics, launch:
```bash
# To launch rrbot to be controlled by rqt_reconfigure
roslaunch pal_wbc_tutorial rrbot_wbc_topic.launch
```
