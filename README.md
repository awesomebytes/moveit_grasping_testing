Package with stuff for testing grasping with REEM.

Installation instructions for using this package:

* 1) Installed (in a workspace called reem-sim_ws for example) REEM simulation:
Instructions: http://wiki.ros.org/Robots/REEM/Tutorials/Launching%20a%20REEM%20Gazebo%20simulation
They use the following rosinstall:
https://raw.githubusercontent.com/pal-robotics/pal-ros-pkg/master/reem-sim-hydro.rosinstall


* 2) Installed (in a workspace called ork_ws for example) Object Recognition Kitchen:
Instructions: http://wg-perception.github.io/object_recognition_core/install.html#rosinstall-file
Using this rosinstall file:
http://wg-perception.github.io/object_recognition_core/_downloads/ork.rosinstall

Before doing catkin_make of ORK do:
```
source ~/reem-sim_ws/devel/setup.bash
```
So you correctly overlay workspaces (info: http://wiki.ros.org/catkin/Tutorials/workspace_overlaying)

Then I would recommend you to do another workspace (just to keep things separated and skipping compiling everything all the time):

* 3) Install this package rosinstall in a new workspace (reem_manipulation_ws for example):
```
cd ~
mkdir -p ~/reem_manipulation_ws_ws/src && cd ~/reem_manipulation_ws_ws/src
catkin_init_workspace

wstool init .
wstool merge https://raw.githubusercontent.com/awesomebytes/moveit_grasping_testing/master/.rosinstall

wstool update -j8

cd ..
rosdep install --from-paths src --ignore-src --rosdistro hydro -y

source ~/ork_ws/devel/setup.bash
catkin_make
```

=== Launch stuff ===

* Launch simulation of REEM in a world with a table and some objects over it (world:=objects_in_table), also, launch it with the Xtion over REEM head (robot:=rgbd)
```
roslaunch reem_gazebo reem_gazebo.launch world:=objects_in_table robot:=rgbd
```
* You probably want to reduce the rate of publishing of the rgbd data:
```
roslaunch reem_rgbd_launch simulation_reduce_rate.launch
```

* Launch block grasp generator server and grasp object server:
```
roslaunch reem_tabletop_grasping tabletop_grasping.launch
```
