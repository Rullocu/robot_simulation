# Lite6 Pick-and-Place Simulation

ROS2 (Jazzy) simulation of a **UFACTORY Lite6** robot transferring rectangular products from a source box to a target box, using **MoveIt2** for motion planning.

---

## Quick Start

### 1. Build the Docker image (first time only)

```bash
docker compose build
```

> Takes ~10–15 min on first run (downloads ROS2 Jazzy + MoveIt2 + xarm_ros2).

### 2. Start the container and build the custom package

```bash
# Start a container shell
docker compose run --rm sim bash

# Inside the container — build once after the bind-mount is live:
cd /ws
colcon build --packages-select lite6_pick_place --symlink-install
source install/setup.bash
```

### 3. Launch the simulation

```bash
# Inside the container (with display forwarded — see Display section below):
ros2 launch lite6_pick_place pick_place.launch.py
```

RViz opens showing the Lite6 robot, two boxes on a table, and the product grid.  
After ~8 seconds the pick-and-place node starts and executes the transfer.

---

## Display forwarding (WSL2 / WSLg)

WSLg provides native display forwarding on Windows 11.  
No extra setup is needed — the `docker-compose.yml` already mounts the
WSLg sockets.

If RViz does not open:
```bash
# On the Windows/WSL2 host:
export DISPLAY=:0
# Or let WSLg set it automatically
```

---

## Configuration

Edit `config/params.yaml` (bind-mounted into `/ws/config/params.yaml`) without rebuilding:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_cols` | 3 | Products in X direction |
| `grid_rows` | 5 | Products in Y direction |
| `grid_layers` | 10 | Products stacked in Z |
| `product_size_x/y/z` | 0.04 / 0.04 / 0.03 m | Product dimensions |
| `source_box_x/y/z` | 0.23 / 0.18 / 0.05 m | Source box floor centre |
| `target_box_x/y/z` | 0.23 / -0.18 / 0.05 m | Target box floor centre |
| `approach_height` | 0.08 m | Height above product for approach |
| `retreat_height` | 0.12 m | Height above product after pick/place |
| `max_products` | 6 | `-1` = all products; positive = demo limit |
| `velocity_scaling` | 0.3 | Joint velocity scale [0–1] |
| `pick_orientation_x/y/z/w` | 1/0/0/0 | EEF orientation (top-down grip) |

Run with a custom params file:
```bash
ros2 launch lite6_pick_place pick_place.launch.py \
    params_file:=/ws/config/params.yaml
```

---

## Project structure

```
robot_simulation/
├── Dockerfile                    # ROS2 Jazzy + MoveIt2 + xarm_ros2
├── docker-compose.yml            # Container with display + bind-mounts
├── config/
│   └── params.yaml               # Editable config (bind-mounted)
└── src/
    └── lite6_pick_place/
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── params.yaml       # Default params (installed into share/)
        ├── launch/
        │   └── pick_place.launch.py
        └── lite6_pick_place/
            ├── scene_manager.py  # Collision objects: table, boxes, products
            ├── motion_planner.py # MoveItPy wrapper: move / pick / place
            └── pick_place_node.py # Main orchestrator
```

---

## Architecture / Design decisions

### Module separation

| Module | Responsibility |
|--------|---------------|
| `scene_manager.py` | All MoveIt2 planning-scene mutations (add/remove collision objects) |
| `motion_planner.py` | All motion: `move_home`, `move_to_pose`, `pick_product`, `place_product` |
| `pick_place_node.py` | ROS2 parameter loading, loop ordering, logging |

### Scene representation

Each box is modelled as **5 solid cuboids** (floor + 4 walls, open top).
This lets the planner see the box walls as obstacles while still allowing
the robot to reach into the box from above.

### Simulated grasping

The Lite6 gripper in xarm_ros2 is declared as a **passive (fixed) joint**
in the SRDF for the fake-hardware simulation.  There is no actuated gripper
planning group.  Grasping is therefore simulated by:

1. **Attach** — `AttachedCollisionObject` moves the product from the world
   scene into the robot's kinematic chain.  MoveIt2 then treats it as part
   of the robot for collision checking during the carry phase.
2. **Detach** — the product is released back into the world at the target
   position.

The gripper mesh is still rendered in RViz and the motion looks natural.

### Iteration order

Products are picked **top layer → bottom layer** in the source box so the
arm never has to reach past an unremoved product.  They are placed at the
**same (col, row, layer)** position in the target box ("corresponding
position" per the task spec), which is valid because the target starts
empty.

### Pick orientation

The EEF approaches from directly above using quaternion **(x=1, y=0, z=0, w=0)**,
which corresponds to RPY = (π, 0, 0) — the standard "gripper pointing down"
orientation in xarm_ros2's coordinate system (matches the SDK examples
`pose: [x, y, z, 3.14, 0, 0]`).

---

## Rebuilding after a code change

Because `--symlink-install` is used, Python file edits take effect
immediately without rebuilding.  If you add new Python files or change
`setup.py`, run:

```bash
colcon build --packages-select lite6_pick_place --symlink-install
source install/setup.bash
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| RViz black / no display | Check `DISPLAY` env var; try `LIBGL_ALWAYS_SOFTWARE=1` |
| `move_group` crash | Re-run; first launch sometimes races with controller spawners |
| Planning always fails | Increase `planning_time` or reduce `velocity_scaling` in params.yaml |
| Products out of reach | Reduce `source_box_x/y` so boxes are closer to robot base |
| 150-product run takes too long | Set `max_products: 10` in params.yaml for a quicker demo |
