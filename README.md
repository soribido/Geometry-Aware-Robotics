# Geometry-Aware-Robotics

This repository constitutes a collection of geometry-aware robotics algorithms and simulations. We plan to add various robotics-related implementations in the future.

## Obstacle Avoidance (Artificial Potential Field)

This directory contains implementations of the Artificial Potential Field (APF) algorithm for robot path planning and obstacle avoidance.

### Included Files

*   **`obstacle_avoidance/APF.py`**
    *   Implements the classic APF algorithm.
    *   Uses **Attractive Force** to pull the robot toward the goal.
    *   Uses **Repulsive Force** to push the robot away from obstacles.

*   **`obstacle_avoidance/APF_improved.py`**
    *   An improved version of the APF algorithm.
    *   **Tangential Force**: Adds a force tangential to the obstacle boundary to help the robot navigate around obstacles smoother, rather than just being pushed away.
    *   **Random Noise**: Adds small random perturbations to help the robot escape local minima situations where attractive and repulsive forces might cancel out.

### Visualization

Below is a demonstration of the improved APF algorithm in action:

![APF Simulation](media/APF.gif)

## AprilTag Generation

This directory contains a tool for generating AprilTag markers, which are widely used in robotics for localization, calibration, and object tracking.

### Features

*   **`apriltag/make_apriltag.py`**
    *   Generates AprilTag markers from the tag36h11 family
    *   Creates individual tag images with cutting guide boxes
    *   Arranges multiple tags on A4/A3 paper layouts for printing
    *   Configurable tag size, DPI, margins, and grid layout
    *   Outputs high-quality PNG files suitable for physical printing

### Usage

The script generates both individual tag files and a complete sheet layout, making it easy to print and cut out physical AprilTag markers for your robotics applications.

### Example Output

<p align="center">
  <img src="media/apriltags_A3_example.png" width="50%" alt="AprilTag A3 Layout Example">
</p>
