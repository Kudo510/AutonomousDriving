# Project

## Work Package 0:

- Run simulation and drive the car around the pre-defined path
- Record all messages using rosbag, so that everybody has some simulation data to start working on

## Work Package 1: Generate a voxel grip representation of the environment

- Generate a point cloud from depth image (using ROS's package depth_image_proc)
- Generate an occupancy grid (voxel grid representation) from a point cloud message (using Octomap)
- Merge the voxel grid representation into a global map
- Use SLAM algorithm to generate a map and estimate a path? or should it be part of work-package 3?
- Integrate the functionality into a function in the controller? or publish a message with the global map?

## Work Package 2: Semantic Object Detection and Classification

- Process info from semantic camera to extract a list of detections (i.e. belonging to classes: car, traffic light, pedestrian?)
- Process info from semantic camera to extract relevant segmentation information?
- Integrate detections and segmentation into global map? and/or add timestamp info and pose info?

## Work Package 3: Path Planning (Geometric representation of a route from start to end, high-level route information)

- Where do we get the predefined track from?
- Generate a sequence of waypoints (poses?) from start to goal position.
    - based on the environment and semantic object information.
    - It should be collision-free (avoiding obstacles).
    - It does not consider timing information and dynamics of the vehicle.
- Publish message with path info

## Work Package 4: Trajectory Planning

- Subscribe to path messages
- Predict trajectory for detected cars for the next 10 timestamps using
    - knowledge-based methods like state-estimation with a CTRA model?
        - or a learning-based algorithm like encoder-decoder algorithm or GNN (Graph neural network)?
    - Calculate collision probabilities
        - e.g., using Mahalanobis distance and X2 distribution
- Develop an algorithm that generates a trajectory based on the path and vehicle dynamics.
    - time-parametrized sequence of poses that the vehicle should follow
    - including velocities, acceleration, and braking?

## Work Package 5: Vehicle Control

- Implement motion control algorithms to regulate acceleration, braking, and steering rate.
- Incorporate a state machine to manage traffic lights (and road rules like ?)