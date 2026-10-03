# hh 任务三独立测试包

## 放置位置

把整个 `hh` 文件夹放到小车：

```text
/home/orangepi/wheel_robot/src/hh
```

不要拆开复制内部文件。

## 编译

```bash
cd /home/orangepi/wheel_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select hh
source install/setup.bash
```

`hh`依赖小车原有的 `wheel_robot` 和 `wheeled_legged_pkg`。如果依赖尚未编译，可以执行：

```bash
colcon build --packages-select wheeled_legged_pkg wheel_robot hh
source install/setup.bash
```

## 启动菜单

```bash
ros2 run hh hh_menu
```

单项测试彼此独立，不检查正式比赛状态机是否完成前序任务。例如可以直接测试人行道画面、环岛画面或台阶检测。

状态机只供以后整场自动比赛模式使用，确保正式运行顺序为台阶、环岛、人行道、起点计圈。

## 单独启动

```bash
# 黑色单线、环岛、人行道，只识别不动车
ros2 launch hh hh_visual_test.launch.py enable_motion:=false

# 黑色单线低速循迹
ros2 launch hh hh_visual_test.launch.py enable_motion:=true

# 完整台阶接近和带速跳跃
ros2 launch hh hh_step_test.launch.py start_base:=true jump_enabled:=true
```

视觉参数：

```text
/home/orangepi/wheel_robot/src/hh/config/hh_test.yaml
```

修改源码配置后需重新编译并重新 `source install/setup.bash`。
