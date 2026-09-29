# key_control

一个仿照 turtlesim `turtle_teleop_key` 的键盘速度控制节点（ROS 2 / rclcpp）。
与 `turtle_teleop_key` 的"按住持续输出"不同，本节点采用"每按一次累加一步"的方式：

| 按键 | 效果 |
| --- | --- |
| `↑` 上箭头 | `linear.x += 0.1` |
| `↓` 下箭头 | `linear.x -= 0.1` |
| `←` 左箭头 | `angular.z += 0.1`（逆时针 / 左转） |
| `→` 右箭头 | `angular.z -= 0.1`（顺时针 / 右转） |
| `Space` 空格 | `linear.x = 0, angular.z = 0` |

节点以固定频率（默认 50 Hz）在 `/cmd_vel` 话题上发布 `geometry_msgs/msg/Twist`。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd MujocoSim
colcon build --symlink-install --base-paths src
source install/setup.bash
```

> 注意：本包位于 `MujocoSim/src`，而项目原有包位于 `MujocoSim/scr`。
> 若需与原有包一起构建，可执行：
> ```bash
> colcon build --symlink-install --base-paths src scr
> ```

## 运行

```bash
ros2 run key_control key_control_node
```

或使用 launch 文件：

```bash
ros2 launch key_control key_control.launch.py
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `cmd_vel_topic` | `/cmd_vel` | 发布速度指令的话题名 |
| `linear_step` | `0.1` | 每次按键线速度累加步长 |
| `angular_step` | `0.1` | 每次按键角速度累加步长 |
| `publish_rate` | `50.0` | 发布频率（Hz） |

按 `Ctrl-C` 退出。
