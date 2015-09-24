Instructions to run:

1. Put this package in your catkin workspace src directory
2. Run catkin_make from the root catkin workspace directory
3. Make sure to remember to run "source devel/setup.bash" if you haven't "overlaid" this workspace before
4. In 3 different tabs, run (in this order):
  roslaunch pr2_gazebo pr2_empty_world.launch
  rosrun rviz rviz
  roslaunch sample_move_arm move_arm.launch
  
