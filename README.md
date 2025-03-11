# ti5robot_ros2_dep
## 启动步骤
```bash
source install/setup.bash
```
IMU 节点 启动 IMU 传感器节点以获取姿态数据：

```bash
ros2 launch imu_ros2 imu_spec_meg.launch.py
```
控制器节点：
```bash
ros2 launch ros2_control_t170 t170_controller.launch.py
```
