launch:

roscore
if something goes wrong with the razer hydra, i could only fix it killing roscore and firing it up again

simulation_setup.launch
for the gazebo

hydra_and_tf.launch
for the hydra driver publishing gazebo and a static transform to link both to base_link

rviz.launch
obvious thing

roslaunch reem_moveit_config moveit_planning_execution.launch
for having moveit running



