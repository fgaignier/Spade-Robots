# SSH SUR LE TURTLEBOT
# lancer les noeuds minimaux turtlebot (sur le robot en ssh) 
roslaunch turtlebot_bringup minimal.launch 
# lancer la navigation en utilisant une carte (donnee)
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/maps/espace_technologique.yaml 

# sur ma machine
source ROS/t1.sh
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen

# teleoperation (controle de la tortue a partir du clavier
roslaunch turtlebot_teleop keyboard_teleop.launch 

# outil pour obtenir la position de la tortue
#soit SSH, soit on source le fichier t1.sh

rosrun tf tf_echo /map /base_link

# outil pour simuler un turtlebot sur ma machine
sudo apt-get install ros-kinetic-turtlebot-gazebo 

# pour lancer
roslaunch turtlebot_gazebo turtlebot_world.launch

roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/gaignier/ROS/maps/espace_technologique.yaml

# pour le bras
cd ~/ros_ws/src

1) git clone https://github.com/corot/turtlebot_arm.git

2) git clone https://github.com/vanadiumlabs/arbotix_ros

3) rosdep install -r --from-paths src --ignore-src --rosdistro kinetic -y 

4) cd..

modifier les makefiles. Ajouter set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ros_ws/src/turtlebot_arm/turtlebot_arm_ikfast_plugin/CMakeLists.txt
ros_ws/src/turtlebot_arm/turtlebot_arm_block_manipulation/CMakeLists.txt 

5) catkin_make

6) ajouter ros_ws/devel/lib/python2.7/dist-packages au PYTHONPATH

7) demarrer le bras:
roslaunch turtlebot_arm_bringup arm.launch


# pour le bras
 http://learn.trossenrobotics.com/projects/186-widowx-arm-with-ros-getting-started-guide.html

# pour lancer le bras
roslaunch widowx_arm_controller widowx_arm_controler.launch
# pour lancer le gui simple:
cd ~/widox_arm/src/arbotix_ros/arbotix_python/bin
./arbotix_gui

# pour lancer le gui complique (sans les etapes d'avant)
roslaunch widowx_arm_bringup arm_moveit.launch sim:=false sr300:=false




