# Entity Spawner

ROS 2 (Jazzy) package providing service-based management for loading and spawning STL meshes into a MoveIt planning scene and, optionally, into a running Gazebo (ros_gz_sim) world.

## Services

- `loadEntity (entity_spawner/srv/LoadEntity)`
  - **Request:** `stl_path`, `entity_name`
  - **Response:** `success`, `message`
  - Validates the path, parses the STL mesh, and enqueues the entity for later spawning.
- `spawnEntity (entity_spawner/srv/SpawnEntity)`
  - **Request:** `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `scale`, `entity_name`, `frame_id`, `add_to_planning_scene`, `spawn_in_sim_world`, `sim_world`, `sim_reference_frame`
  - **Response:** `success`, `message`
  - Applies the queued mesh (scaled by `scale`) as a collision object in the planning scene (`add_to_planning_scene`) and/or spawns a static model in the specified Gazebo world (`spawn_in_sim_world`). Leaving both booleans unset/false preserves the legacy behaviour of updating the planning scene only.

### Simulation defaults

`EntitySpawnerNode` exposes two parameters to tailor simulation spawning:

| Parameter | Default | Description |
| --- | --- | --- |
| `simulation.default_world` | `default` | Gazebo world name used when the request does not set `sim_world`. |
| `simulation.reference_frame` | `world` | Frame used for positioning unless `sim_reference_frame` overrides it. |
| `simulation.service_timeout` | `5.0` | Timeout (seconds) when waiting for `/world/<world>/create`. |

> **Note:** Gazebo forbids spaces and most punctuation in entity names. The spawner replaces unsupported characters with underscores when creating simulation models.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select entity_spawner
source install/setup.bash
```

## Run

```bash
ros2 run entity_spawner entity_spawner_node
```

## Example usage

1. Load a mesh:
   ```bash
   ros2 service call /loadEntity entity_spawner/srv/LoadEntity "{stl_path: '/absolute/path/model.stl', entity_name: 'sample_model'}"
   ```
2. Spawn it into the planning scene only:
   ```bash
   ros2 service call /spawnEntity entity_spawner/srv/SpawnEntity "{x: 0.0, y: 0.0, z: 0.1, roll: 0.0, pitch: 0.0, yaw: 0.0, scale: 0.01, entity_name: 'sample_model', frame_id: 'world', add_to_planning_scene: true, spawn_in_sim_world: false, sim_world: '', sim_reference_frame: ''}"
   ```
3. Spawn into both the planning scene and the default Gazebo world:
   ```bash
   ros2 service call /spawnEntity entity_spawner/srv/SpawnEntity "{x: 0.5, y: -0.3, z: 0.1, roll: 0.0, pitch: 0.0, yaw: 1.57, scale: 1.0, entity_name: 'sample_model', frame_id: 'world', add_to_planning_scene: true, spawn_in_sim_world: true, sim_world: '', sim_reference_frame: ''}"
   ```
