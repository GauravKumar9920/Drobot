## Examples

Before launching, ensure to build and source the workspace:

```bash
colcon build && source install/setup.bash
```

### 1. Launching REXROV in an empty world

```bash
ros2 launch dave_robot_launch robot_in_world.launch.py z:=2.0 namespace:=rexrov world_name:=empty.sdf paused:=false
```

### 2. Launching REXROV in dave_ocean_waves.world

```bash
ros2 launch dave_robot_launch robot_in_world.launch.py z:=-5 namespace:=rexrov world_name:=dave_ocean_waves paused:=false
```

### 3. Launching Slocum Glider in an empty world

```bash
ros2 launch dave_robot_launch robot_in_world.launch.py z:=0.2 namespace:=glider_slocum world_name:=empty.sdf paused:=false
```

### 4. Launching Slocum Glider in dave_ocean_waves.world

```bash
ros2 launch dave_robot_launch robot_in_world.launch.py x:=4 z:=-1.5 namespace:=glider_slocum world_name:=dave_ocean_waves paused:=false
```
