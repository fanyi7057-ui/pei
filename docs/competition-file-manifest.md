# 比赛版文件清单 / Competition-only file manifest

`robot` 现在是部署到橙派 `/home/orangepi/robot` 的比赛工作副本。它只保留菜单中的编译、检查预览、支架联调、原地站立、正式比赛、日志查看和使用说明所需代码；原始完整资料仍由仓库外层 `src_first/` 和 Git 历史保存。

## 保留的 ROS2 包

- `motor_temperature_pkg`：电机温度消息定义。
- `wheeled_legged_pkg`：串口桥、遥控/SBUS、里程计、姿态与 `/cmd_vel`、`/cmd_posture` 等控制话题。
- `wheel_robot`：Astra 相机节点、原始回退入口 `icn.py`、当前自适应入口 `icn_adaptive.py`、`adaptive_runtime.py` 趋势/标志控制、`race_runtime.py` 原实时参数校验、巡线/人行横道/台阶检测、站立节点、菜单启动文件和比赛 YAML。

`wheel_robot/vendor/OrbbecSDK_ROS2/orbbec_camera/SDK/include` 与 `SDK/lib/arm64` 被保留，因为 Orange Pi 的 `astra_camera_node` 编译和运行都依赖它们。

## 已移除的非比赛内容

- ROS 教程转换包、URDF/RViz/仿真包、演示包；
- 不由当前菜单 1--4 调用的教程、键盘遥控、旧巡线和备份 Python 脚本；
- MobileNet 演示模型、Orbbec 相机描述网格、x64/arm32 SDK 和上游示例/文档；
- 对应的教学启动文件与 YAML。

这不是算法重写：比赛入口仍是 `ros2 launch wheel_robot icn.launch.py`，也没有删除正式比赛的相机、底盘桥、深度台阶支持或姿态脚本。

## 传输与恢复

传输当前 `robot` 到小车后，在 `/home/orangepi/robot` 执行：

```bash
sha256sum -c checksums/SHA256SUMS.txt
```

校验清单只覆盖 Git 跟踪的 `robot` 文件，不包含小车生成的 `build/`、`install/`、`log/` 或缓存。若以后确实需要被移除的教程或模型，请从 `src_first/` 或对应 Git 提交恢复，不要把未知来源文件直接混入比赛版目录。

## 验证边界

本次验证仅包括 Windows 上的文件清单、Python/菜单静态测试和校验哈希。没有在 Orange Pi 重新 `colcon build`，也没有执行任何电机、跳跃或正式比赛动作；部署后仍要先编译，再从菜单 2 和菜单 3 开始实车检查。
