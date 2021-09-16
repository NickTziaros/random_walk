# Mapping with Robot Swarms

This Ros package maps the environment by implementing a random walk algorithm on a Swarm of Turtlebots 

# Depenencies

## NumPy

```bash
pip install numpy
```
## SciPy 

```bash
sudo pip install scipy
```

# Usage
## Starg Gazebo Simulation with Slam

```bash
roslaunch random_walk multi_turtlebot3.launch 
```
## Starg MultiRobot map merge node

```bash
roslaunch random_walk multi_map_merge.launch
```
## Starg Random Walk node

```bash
roslaunch random_walk levy_rw_node.launch 
```



