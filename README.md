# Vision

A swarm of autonomous mobile robots are moving throughout a factory. They each have several items that they need to collect in order to finish an order. The scenario is inspired by the packing of Lego bricks for a bag, which is one part of what goes into Lego set. The robots should never collide and are thus equipped with sensors so that they themselves can avoid collision. 

# Directories

This repository consists of two projects
1. [Simulation](https://github.com/DEIS-Tools/Swarm-Robotics/tree/master/Simulation) of a swarm of autonomous mobile robots
2. [Robot-code](https://github.com/DEIS-Tools/Swarm-Robotics/tree/master/Robot-code) used for communication between the actual swarm robot and Uppaal

For futher information and setup instructions check ReadMe in directories mentioned above

# Map image conversion to point map

1. Install Ubuntu 16.04 virtual machine (easiest with VMware)
2. Follow [Guide](http://wiki.ros.org/kinetic/Installation/Ubuntu) in order to install [ROS (Robot Operating System)](http://wiki.ros.org) mainly **ros-kinetic-desktop-full**
3. Inside the virtual machine (running Ubuntu 16.04) install the map server using `sudo apt-get install ros-kinetic-map-server`
4. Store map image as well as map.yaml in a known location on VM and `cd` into it
5. Inside map.yaml correct the image location 
6. Run `roscore` command. (if it fails to run make sure you did the `source ~/.bashrc` step)
7. Open new terminal and run the map server `rosrun map_server map_server playground.yaml`
8. In new terminal run `rosrun rviz rviz`
9. A new window shall open. Click button **add** next select **By Topic** tab and click **Map** and press OK.
10. One may navigate around the map using this tool and make changes.
11. Open new terminal and run `rostopic list`. This will show all the active topics in use right now, but the one of interest is **/map**. Now we may extract the map using `rostopic echo map >> data.txt`
