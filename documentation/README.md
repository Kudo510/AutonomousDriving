# Trajectory planning
Package to use: move_base

Process: follow the image available in documentation

## For base local planner:
Package to use: base_local_planner. Does it mean that the trajectory is given before generating the grid map? Probably not, because it can’t generate it as long as it didn’t see the obstacles (car, etc…)

Provides a controller that drives the car in the plane by connecting the path planner to the car. Using a map, the planner creates a kinematic trajectory for the car to get from start to goal. Along the way, the planner creates locally around the car a value function, represented as a grid map. This value function encodes the costs of traversing through the grid cells. The controller's job is to use this value function to determine dx, dy, dtheta velocities to send to the car.
The basic idea of both the Trajectory Rollout and Dynamic Window Approach (DWA) algorithms is as follows:

-	Discretely sample in the robot's control space (dx,dy,dtheta)
-	For each sampled velocity, perform forward simulation from the robot's current state to predict what would happen if the sampled velocity were applied for some (short) period of time.
-	Evaluate (score) each trajectory resulting from the forward simulation, using a metric that incorporates characteristics such as: proximity to obstacles, proximity to the goal, proximity to the global path, and speed. Discard illegal trajectories (those that collide with obstacles).
-	Pick the highest-scoring trajectory and send the associated velocity to the mobile base.
-	Rinse and repeat.

-	How to import the map? How to work with Rviz? 

-	How to see the sensor output as an input for example obstacles?




# Useful links and sources

http://wiki.ros.org/move_base

http://wiki.ros.org/base_local_planner#Overview

https://www.youtube.com/watch?v=B-7-xK-_rYw
