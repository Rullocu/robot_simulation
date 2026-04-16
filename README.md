# Lite6 Pick-and-Place Simulation

ROS2 Jazzy simulation of a **UFACTORY Lite6** robot transferring rectangular
products from a source box to a target box, using **MoveIt2** for motion
planning — no physical robot or GPU required.

---

## Quick Start

### 1. Build the Docker image (first time only)

```bash
docker compose build
```

> Takes ~10–15 min on the first run (downloads ROS2 Jazzy + MoveIt2 + xarm_ros2).

### 2. Start the simulation

**WSL2 (Windows 11 + WSLg):**
```bash
docker compose -f docker-compose.yml -f docker-compose.wsl2.yml run --rm sim
```

**Native Ubuntu:**
```bash
xhost +local:docker   # allow the container to use your X server (once per session)
docker compose run --rm sim
```

> Use `run --rm sim` (not `up`) — `run` allocates an interactive TTY so you
> land in a bash shell. `up` only streams logs and `exec bash` would exit
> immediately without a terminal.

The container builds the ROS2 package automatically and drops to a shell.
You will see:

```
====================================================
Package built. Run the simulation with:
  ros2 launch lite6_pick_place pick_place.launch.py
====================================================
```

### 3. Launch the simulation

Inside the container shell:
```bash
ros2 launch lite6_pick_place pick_place.launch.py
```

RViz opens showing the Lite6 robot, two open-top boxes on a table, and the
product grid. After ~8 seconds the pick-and-place node starts and runs
through all products.

---

## Configuration

Edit `config/params.yaml` on the host — it is bind-mounted into the container
at `/ws/config/params.yaml`. Changes take effect on the next launch without
rebuilding the image.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_cols` | 2 | Products along X axis |
| `grid_rows` | 2 | Products along Y axis |
| `grid_layers` | 1 | Products stacked in Z |
| `product_size_x/y/z` | 0.040 / 0.040 / 0.030 m | Product dimensions |
| `source_box_x/y/z` | 0.230 / 0.180 / 0.050 m | Source box floor centre |
| `target_box_x/y/z` | 0.230 / −0.180 / 0.050 m | Target box floor centre |
| `box_height` | 0.10 m | Height of box walls |
| `box_margin` | 0.030 m | Gap between product grid edge and box wall |
| `product_spacing` | 0.030 m | Gap between adjacent products |
| `approach_height` | 0.110 m | OMPL approach height above product centre |
| `retreat_height` | 0.130 m | Transit height used for all vertical lift + lateral moves |
| `max_products` | −1 | −1 = all; positive number = demo limit |
| `velocity_scaling` | 0.3 | Joint velocity scale [0–1] |
| `planning_time` | 5.0 s | OMPL time budget per plan attempt |
| `pick_orientation_x/y/z/w` | 1/0/0/0 | EEF quaternion — top-down grip (RPY = π,0,0) |

---

## Project structure

```
robot_simulation/
├── Dockerfile                     # ROS2 Jazzy + MoveIt2 + xarm_ros2
├── docker-compose.yml             # Base config (native Ubuntu + WSL2 common)
├── docker-compose.wsl2.yml        # WSL2 override (WSLg sockets + Wayland vars)
├── config/
│   └── params.yaml                # Editable config (bind-mounted at runtime)
└── src/
    └── lite6_pick_place/
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── params.yaml        # Default params (installed into share/)
        ├── launch/
        │   └── pick_place.launch.py
        └── lite6_pick_place/
            ├── scene_manager.py   # Planning scene: table, boxes, products
            ├── motion_planner.py  # MoveIt2 service wrapper: OMPL + Cartesian
            └── pick_place_node.py # ROS2 params, loop ordering, logging
```

---

## Architecture

### Module separation

| Module | Responsibility |
|--------|----------------|
| `scene_manager.py` | All MoveIt2 planning-scene mutations via `/apply_planning_scene` |
| `motion_planner.py` | Motion planning and execution via `/plan_kinematic_path`, `/compute_cartesian_path`, `/execute_trajectory` |
| `pick_place_node.py` | ROS2 parameter loading, pick-and-place loop, logging |

No `moveit_py` bindings are used — everything goes through standard
`moveit_msgs` services and actions so the package works with the standard
`ros-jazzy-moveit` apt package.

### Two-planner motion strategy

Each pick-and-place cycle uses two different planners for different segments:

| Segment | Planner | Reason |
|---------|---------|--------|
| Approach to transit height | OMPL (`/plan_kinematic_path`) | Free-space motion, no constraints needed |
| Lateral transit between boxes | OMPL | Large free-space move at safe height |
| **Descend to product / place height** | **Cartesian** (`/compute_cartesian_path`) | Forces a straight vertical line — prevents OMPL's joint-space arc from sweeping the gripper body through the product's bounding box |
| **Ascent after pick / place** | **Cartesian** | Same reason |

The root cause of path failures with OMPL alone: OMPL plans in **joint space**.
A joint-space straight line converts to a Cartesian arc, and somewhere along
that arc the `uflite_gripper_link` body can sweep sideways into the 40×40×30 mm
product box even though the start and goal poses are both collision-free.
Cartesian planning interpolates at 5 mm steps along a straight line in task
space, making this physically impossible.

### Scene representation

Each box is modelled as **5 solid cuboids** (floor + 4 walls, open top), so
the planner sees the walls as obstacles while the arm can still approach from
above. A margin (`box_margin`) is added around the product grid to give the
gripper clearance from the walls.

### Simulated grasping

The Lite6 gripper in xarm_ros2 is declared as a **passive (fixed) joint** in
the SRDF for fake-hardware simulation. There is no actuated gripper group.
Grasping is simulated by:

1. **Pick** — while the gripper is at approach height, the product is attached
   to the EEF as an `AttachedCollisionObject`. MoveIt2 treats it as part of the
   robot's kinematic chain during carry and collision checking.
2. **Place** — the product is silently detached (removed from attached state
   without adding to the world) before the ascent, then added back to the world
   at the target position after the gripper has cleared.

### Iteration order

Products are picked **top layer → bottom layer** so the arm never has to reach
past a still-present product below the one being picked. They are placed at the
same `(col, row, layer)` position in the target box.

---

## Rebuilding after code changes

`--symlink-install` is used, so Python edits take effect immediately.
Only needed if you add files or change `setup.py`:

```bash
# Inside the container:
colcon build --packages-select lite6_pick_place --symlink-install
source install/setup.bash
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| RViz black / no window | Check `DISPLAY`; on native Ubuntu run `xhost +local:docker` first |
| RViz segfaults on WSL | Uncomment `LIBGL_ALWAYS_SOFTWARE=1` in `docker-compose.yml` |
| `move_group` crashes on launch | Re-run; first launch sometimes races with controller spawners |
| Cartesian descent returns fraction < 1.0 | Product or box wall is blocking the straight-line path — check `approach_height` clears the box rim: must satisfy `approach_height > box_height − product_size_z/2` |
| Planning always fails | Increase `planning_time` or reduce `velocity_scaling` in params.yaml |
| Products out of reach | Move boxes closer to robot base by reducing `source_box_x/y` |
| Full 150-product run too slow | Set `max_products: 10` for a quick demo |
