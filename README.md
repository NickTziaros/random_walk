# Mapping with Robot Swarms

This Ros package maps the environment by implementing a random walk algorithm on a Swarm of Turtlebots 

## **Dependencies**

#### TurtleBot3
  Install all the TurtleBot3 packages from Robotis.
  
  Follow the instructions in the links bellow
  
  * [Turtlebot Ros Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
  
  * [Turtlebot Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
  
  
  
#### NumPy

  ```bash
  pip install numpy
  ```
#### SciPy 

  ```bash
  sudo pip install scipy
  ```

## **Usage**
### Start Gazebo Simulation with Slam

  ```bash
  roslaunch random_walk multi_turtlebot3.launch 
  ```
### Start MultiRobot map merge node

  ```bash
  roslaunch random_walk multi_map_merge.launch
  ```
### Start Random Walk node

  ```bash
  roslaunch random_walk levy_rw_node.launch 
  ```
## **Adding more turtlebots to the simulation**


  ###  Uncomment the comments in the following files:
   * random_walk/launch/multi_turtlebot3.launch
   * random_walk/launch/multi_turtlebot3_slam.launch
   * random_walk/launch/multi_map_merge.launch
   * random_walk/launch/levy_rw_node.launch

<<<<<<< HEAD

=======
>>>>>>> 328e2bd7b8dbed757628c91e57da03c8c15e66e4
