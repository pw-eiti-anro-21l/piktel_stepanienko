## How to use:

### Using launch file:
```
ros2 launch turtle_manipulation move_turtle.py
```

### Without launch file:
#### Launch turtlesim turtlesim_node beforehand:
```
ros2 run turtlesim turtlesim_node
```
#### Next launch turtle steering node
```
ros2 run turtle_manipulation move --ros-args --params-file config/params.yaml
```
