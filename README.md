# TAL_BRABO
Tal Brabo is 5 DOF robotic arm developed at TAL pvt. Ltd. This document contains information about development of software interface (driver) for controlling robot using ROS. A usb camera is attached at end-effector of robot. Robot is required to follow a circular object.

-------------------------Prerequisite------------------------------------
1. cv_bridge ros package is required for object tracking application
2. rviz is required for visual display of robot current state

-------------------------Starting Robot (ros driver)---------------------
1. Switch on power supply of robot.
2. Start controller button on control box on robot.
3. Connect arduino's and power on them
3. Start tal_brabo_driver node (tal_brabo/tal_driver.py)
4. Enter positon of first 3 joints (white or balck)
5. Wait for homing to be completed
6. Now you can start other nodes

------------------------Starting nodes for object tarcking task-----------
1. Start ros driver as explained above
2. Attach camera to end_effector
3. Start usb_cam_node
4. Start object_tracker node (tal_brabo/object_tracker.py)
5. Start rviz (brabo_description/launch/display.launch)
6. Start velocity controller node (tal_brabo/velocity_controller.py)

------------------------Important Notes for safety------------------------
1. Always start controller button on control box for reducing vibration in motors.
2. Streaming rate to arduino is 100. Thus any message published on the topic '/joint_command' would result in publishing a command to arduino. Arduino would try to achieve desired angle in 10ms. tal_brabo_node keeps track of current joint angles of the robot. When any node publishes to '/joint_command' topic tal_brabo_node calculates velocity based on published angles and current angles and accordingly sends command to arduino. This velocity will be high if differece between current angles (tracked by tal_brabo_driver) and angles published on '/joint_command' is large as time step is only 10ms. tal_brabo_driver node publish current joint angles on the topic '/joint_states' so that other nodes can use that to publish desired angles. Therefore always use current angles published on topic '/joint_states' to publish desired joint angles. Otherwise a high velocity would result in safety problems.
