# robot

两轮轮足机器人比赛综合工作区。Windows 主副本目录为本项目的 `robot` 文件夹，小车部署目录为 `/home/orangepi/robot`。

- `ros2_ws/src/`：ROS2 视觉、比赛状态机、相机、串口桥和消息包。
- `firmware/LeggedRobot/`：候选 ESP32 下位机固件；未经版本核对不得烧录。
- [测试手册](docs/TESTING.md)：从静态检查到两圈比赛的分阶段测试步骤。
- [比赛版文件清单](docs/competition-file-manifest.md)：说明保留内容、已移除的非比赛资料和恢复边界。
- `checksums/SHA256SUMS.txt`：传输到小车后的完整性校验。

仓库外层的 `src_first/` 是不可修改的原始基线；本目录是后续工作副本。小车原有 `/home/orangepi/wheel_robot` 工作区不要覆盖或删除，以便回退。

初次运行请强制使用 `enable_motion:=false`，并保持遥控器不在 ROS 控制模式。视觉干跑通过不代表电机或跳跃已验证；只有检查相机、串口、诊断、急停并完成支撑架低速测试后，才进入实车运动。不要仅凭本目录中的源码就烧录候选固件或调整平衡 PID。
