Trial Project: ROS2 Simulation – uArm Lite 6 Pick & Place
Objective
Set up a ROS2 simulation using the xarm_ros2 for a uFactory uArm Lite 6.
The robot should transfer rectangular products from one box to another.

Requirements
1. Setup

Use Docker / Docker Compose to provide a reproducible environment

Install ROS2 and run the robot in simulation using:
https://github.com/xarm-Developer/xarm_ros2

2. Scene

Two boxes: source (filled) and target (empty)

Products arranged in a configurable 3D grid

Default: 3 × 5 × 10

3. Functionality

Pick each product from the source box

Place it into the corresponding position in the target box

Avoid collisions and ensure stable motion

4. Motion & Control

Use ROS2-compatible planning (e.g., MoveIt2)

Implement a basic pick-and-place pipeline

5. Configurability

Grid size, product dimensions, and box positions configurable (e.g., YAML / ROS2 params)

6. Code Quality

Clean, modular structure (scene, planning, execution separated)

7. Documentation

README with setup, usage, and design decisions

Deliverables
Git repository

Docker setup (Dockerfile + docker-compose.yml)

Run instructions

Demo (video or screen recording)

Full project handover after completion

Time Expectation
Estimated effort: 2–3 hours

Bonus (Optional)
Simple frontend (e.g., web UI or ROS tool) to:

Start/stop the process

Visualize current robot state / position

