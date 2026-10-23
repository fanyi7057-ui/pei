# robot 比赛代码测试手册

这份手册按“小车上的新工作区 `/home/orangepi/robot`”编写。每一阶段通过后再进入下一阶段。Windows 上的静态检查不能替代小车 Ubuntu 编译、传感器测试或实车测试。

## 安全边界

初次测试不烧录固件、不调 PID、不删除 `/home/orangepi/wheel_robot`。涉及电机时，先把车体可靠架起、轮子离地，安排一人看终端、一人持遥控急停。没有现场人员确认前，不发送速度或跳跃命令。

`enable_motion:=false` 仅用于关闭比赛主程序的运动控制，不代表其他节点或外部话题发布者绝对安全；联合干跑时仍须保持遥控器 SWA 在非 ROS 位置。

## 0. 从电脑传到小车

在 MobaXterm 用 `orangepi` 用户 SSH 登录小车。传输前先查看小车现有目录，不覆盖已有同名文件夹：

```bash
whoami
echo "$HOME"
ls -ld /home/orangepi/robot /home/orangepi/wheel_robot 2>&1
```

若 `/home/orangepi/robot` 已存在，先核对内容与来源，不要直接合并覆盖。确认目标位置后，通过 MobaXterm 左侧 SFTP 面板将 Windows 的 `D:\GitProjects\Two_Wheel\robot` 整个文件夹传到 `/home/orangepi/` 下。传完后完整路径应为 `/home/orangepi/robot/ros2_ws/src`；不要传成 `/home/orangepi/robot/robot`，也不要传到旧的 `wheel_robot` 目录。

## 0.1 中文菜单（推荐入口）

将仓库内 `robot/tools/robot_menu.sh` 额外复制到小车的 `/home/orangepi/robot_menu.sh`。菜单必须在 `robot` 目录外，因为它的高风险选项可以在用户双重确认后清理 `/home/orangepi/robot`：

```bash
cp /home/orangepi/robot/tools/robot_menu.sh /home/orangepi/robot_menu.sh
chmod 755 /home/orangepi/robot_menu.sh
bash -n /home/orangepi/robot_menu.sh
bash /home/orangepi/robot_menu.sh
```

若 `bash -n` 出现 `$'\r'`、`syntax error near unexpected token` 或 `set: invalid option`，说明脚本被 Windows CRLF 换行污染；先只转换脚本（不会启动 ROS 或电机），再重新做语法检查：

```bash
sed -i 's/\r$//' /home/orangepi/robot/tools/robot_menu.sh /home/orangepi/robot_menu.sh
bash -n /home/orangepi/robot_menu.sh
bash /home/orangepi/robot_menu.sh
```

菜单现在会自动设置 `DISPLAY=:0` 与 `XAUTHORITY=/home/orangepi/.Xauthority`，随后以 `xset q` 检查小车本地 X11 显示环境。菜单 2-1、2-2 和 4 的比赛视觉节点还会在节点级别固定 `DISPLAY=:0`，因此不会继承 MobaXterm 的 `localhost:10` 转发显示。菜单 3 会在车载 HDMI 显示深度距离预览；它仍不巡线、不跳跃、零轮速。出现“小车屏幕连接正常”后，不再需要在每个新 SSH 终端手工 export。出现“小车屏幕不可用”只表示本地调试画面不能显示，不阻止编译或非视觉检查，也不证明运动安全。

更新到本版本时，必须同时传输整个 `robot` 目录、重新执行菜单 1 编译，并更新菜单副本：

```bash
cp /home/orangepi/robot/tools/robot_menu.sh /home/orangepi/robot_menu.sh
chmod 755 /home/orangepi/robot_menu.sh
bash -n /home/orangepi/robot_menu.sh
bash /home/orangepi/robot_menu.sh
```

若旧版菜单在选项 1 或 2-1 报 `AMENT_TRACE_SETUP_FILES: unbound variable`，它是在 `set -u` 下加载 ROS Humble 环境时失败，尚未开始编译或启动节点。先对当前小车上的两份旧脚本执行以下临时恢复，再重新打开菜单；电脑的最新版已改为只在加载 setup 时短暂关闭该限制：

```bash
sed -i 's/^set -u$/# temporarily disabled: ROS setup needs unset variables/' \
  /home/orangepi/robot/tools/robot_menu.sh /home/orangepi/robot_menu.sh
bash -n /home/orangepi/robot_menu.sh
bash /home/orangepi/robot_menu.sh
```

若视觉干跑提示 `step_detector_node.py not found`，而 `ls -l` 显示它是指向源码的链接且源码权限为 `-rw-rw-r--`，则是 Windows 传输未保留 Python 的可执行权限。执行下面一行后重新打开菜单；它只修改脚本权限，不重新上传、不编译、不启动节点：

```bash
chmod +x /home/orangepi/robot/ros2_ws/src/wheel_robot/scripts/*.py
```

最新版菜单的选项 1 会在每次 `--symlink-install` 构建前自动完成这一步。

若深度相机节点报 `libOrbbecSDK.so.1.10: cannot open shared object file`，最新版菜单会在构建和相机相关启动项前自动创建所需版本名链接，并把 `install/wheel_robot/lib/wheel_robot` 加入当前菜单会话的 `LD_LIBRARY_PATH`。对已经上传的旧版本，可在该目录手工创建 `libOrbbecSDK.so.1.10 -> libOrbbecSDK.so.1.10.37`，再导出相同目录到 `LD_LIBRARY_PATH` 后重启菜单；不需要把库安装到系统目录。

菜单中的推荐顺序是：`1 编译 → 2-1 视觉干跑 → 3 比赛姿态原地站立检查 → 2-2 支架联合检查 → 4 正式比赛 → 6 备份提示`。每次重新打开菜单后，必须先选 `1` 并成功编译；其他启动项只接受本次菜单会话内编译成功的当前 `robot`，因此不会误运行遗留的旧 `install`。选项 3、2-2 和 4 都要求输入数字 `1` 确认；正式比赛由 `icn.py` 再执行“两秒站立 + ROS 开始确认”的安全门。不要在 `ros2 launch` 所在终端按 Enter：该输入不会可靠地传递给 Python 节点。

选项 3 持续发送零轮速、`joint_height=0.20 m`、`joint_roll=0°`、`joint_pitching=+25°`、`joint_slide=0 m`，同时启动 Astra 深度相机、距离图和车载 HDMI 预览；不启动巡线、跳跃或 `/cmd_vel`。该预览是深度/障碍物图，不是菜单 4 的比赛巡线状态图。参数已保存且通过本地静态检查，不代表它已实车验证稳定；遥控器仍必须完成 SBUS 初始化并保持急停可用。

选项 7 不是常规调试结束步骤：它无法确认 Windows 备份、Git 提交或 GitHub 上传是否成功。它会在两次数字确认前检查相关程序、在最后确认后再次检查，然后才删除固定目录 `/home/orangepi/robot`。除非已在电脑上下载、核对并保存需要的源码、参数、日志和视频，否则不要选择 7。

## A. 确认小车环境

```bash
whoami
echo "$HOME"
ls -ld /home/orangepi/robot /home/orangepi/robot/ros2_ws/src
ls /opt/ros
```

预期用户为 `orangepi`、HOME 为 `/home/orangepi`。如果用户名或 HOME 不符，先停止并修正工作区中的绝对路径。以下命令假定小车装的是 ROS2 Humble；若 `/opt/ros/humble` 不存在，也先停止核对实际 ROS2 版本。

## B. 校验传输完整性

```bash
cd /home/orangepi/robot
sha256sum -c checksums/SHA256SUMS.txt
```

清单中的每一项都必须显示 `OK` 且命令退出码为 0；清单覆盖当前 `robot` 内的版本化源码、菜单、文档和测试，不包含小车运行后产生的 `build/`、`install/`、`log/` 或 Python 缓存。任何一项失败都先重新传输对应文件，不进入编译和运动测试。

## C. 定向构建比赛包

```bash
cd /home/orangepi/robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select motor_temperature_pkg wheeled_legged_pkg wheel_robot \
  --event-handlers console_direct+
source /home/orangepi/robot/ros2_ws/install/setup.bash
```

若编译失败，保存第一条真实错误，不删除旧工作区。新终端需要重新执行两条 `source` 命令。不要把 Windows 的 `build/`、`install/` 或 `log/` 拷到小车。

## D. 视觉干跑（不接底盘）

```bash
cd /home/orangepi/robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot icn.launch.py \
  enable_motion:=false \
  start_base:=false \
  start_step_support:=true
```

观察彩色相机、深度相机、巡线画面、圆环状态、台阶检测和日志。此阶段不启动串口底盘、不证明轮子或跳跃正确。如果相机启动失败，先检查 USB、Orbbec SDK 的 arm64 库、相机参数和错误日志。

## E. 底盘只读检查

车架可靠支撑、轮子离地，遥控器 SWA 保持非 ROS 位置。终端一：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/robot/ros2_ws/install/setup.bash
ros2 launch wheel_robot base.launch.py serial_port:=/dev/ttyUSB0
```

终端二：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/robot/ros2_ws/install/setup.bash
ros2 topic hz /odom
ros2 topic echo /robot_diagnostics --once
ros2 topic echo /sbus_channels --once
ros2 topic echo /motor_temperatures --once
```

`/odom` 应持续刷新；诊断、SBUS 和温度应能读到。若没有诊断消息，车内固件可能不是本目录中的候选版本，先核对版本，不要因此直接烧录。若串口名不是 `/dev/ttyUSB0`，先用 `ls -l /dev/ttyUSB*` 核实设备，再更改 `serial_port`。四项未核清前不进入运动测试。

## F. 支撑架低速轮测

只有 E 通过、车体支撑稳固、遥控急停已验证，并由两人现场配合时，才切换到 ROS 控制并发送极小速度。先确认左右轮方向、停车响应和 `/cmd_vel` 超时保护（代码阈值 0.30 秒）。测试结束发送零速度，并切回遥控关闭位置。本手册不提供无人看守可直接执行的运动命令。

## G. 视觉与底盘联合干跑

车仍架起、SWA 非 ROS，停止 E 中单独启动的底盘节点后运行：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/robot/ros2_ws/install/setup.bash
ros2 launch wheel_robot icn.launch.py \
  enable_motion:=false \
  start_base:=true \
  serial_port:=/dev/ttyUSB0 \
  start_step_support:=true
```

确认相机、串口、诊断和比赛状态机能同时运行。不得仅因这一阶段通过就放车上地自动跑。

## H. 人工确认跳跃

保持 `manual_confirm_before_jump: true`。先在清空场地做受控单次低跳，再做台阶接近和跳跃。每次保存终端日志、原始画面、视觉画面和诊断；记录起跳位置、前进速度、着地状态和偏航。连续至少五次没有碰撞、侧翻、明显偏航或漏触发后，才评估关闭人工确认。

注意当前 `icn.yaml` 标的是 0.15→0.33 m，但底盘桥会将最小值夹到 0.16 m；这次目录整合没有改变该运动参数。

## I. 单圈和两圈比赛

前述阶段全部通过、现场安全人员与遥控急停就位后，才允许 `enable_motion:=true`：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/robot/ros2_ws/install/setup.bash
ros2 launch wheel_robot icn.launch.py \
  enable_motion:=true \
  start_base:=true \
  serial_port:=/dev/ttyUSB0 \
  start_step_support:=true
```

正式比赛启动后，小车会先持续发布零轮速和比赛姿态 `0.15 m / +28°` 两秒，并显示“等待开始确认”。**只有在小车已安全放在赛道起点、遥控急停可用、周围人员已撤离后**，在当前比赛操作台输入数字 `1`。台阶停车并显示确认提示后，在同一终端输入 `2` 执行跳跃；输入 `0` 或按 `Ctrl+C` 结束。

预期当前终端打印“比赛开始确认已收到”。若 `live_tuning.buzzer_enabled=1`，程序会继续保持零轮速完成开始提示的完整时间（默认两次、每次 `0.10 s`、中间 `1.00 s`），随后才开始巡线；目前这是屏幕/日志软件事件，**不会驱动物理蜂鸣器**。确认消息过早、`false` 消息或重复确认都会被忽略，未确认前小车持续保持零轮速和比赛姿态；正常巡线期间也会持续发布同一姿态，避免底盘桥一秒姿态超时。台阶跳跃接管时，既有跳跃辅助姿态临时覆盖；跳跃状态释放后的下一次普通控制周期恢复 `0.15 m / +28°`。这仅记录代码路径，不代表姿态、巡线、跳跃或恢复已经实车成功。

## I.1 保存即生效的实时调参（仅安全停放时）

在 MobaXterm 左侧 SFTP 文件栏只打开 `/home/orangepi/robot/ros2_ws/src/wheel_robot/config/race_tuning.yaml`。只改 `live_tuning:` 下的冒号后数值，保留字段名和缩进后按保存。保存后约 0.5 秒自动生效，**不需要重新编译、重启菜单或输入 `ros2 param set`**；车载 HDMI 画面会显示“实时文件：文件参数已生效”或拒绝原因。完整 `icn.yaml` 中的 `runtime:`、`camera:`、`drive:`、`roundabout:`、`step:` 仍是启动参数，日常调车无需打开它。

这个文件是唯一实时运动来源：普通巡线速度、加/减速平滑、弯道/边缘救线/急弯速度和转向、短时丢线续行、环岛进环/丢线/出环、人行横道、终点方式、四个位姿、台阶与人行横道开关及软件蜂鸣都只改此文件。每项的用途、单位、调大/调小效果和限制都写在文件内。速度、距离、加速度和大部分转向值没有软件最大上限，但必须是有限数；“最低速度不得大于直道速度”，`drive_curve_slowdown` 仍须在 `0..1`。无效 YAML、缺字段、未知字段或违反上述关系会整份拒绝，上一份有效参数保持不变。

`drive_forward_speed: 0.8` 现在可以被软件接受和热加载，但这不代表小车必然达到 `0.8 m/s`；串口桥、ESP32、电机、电池、轮胎和地面仍可能限速。第一次使用较大数值时，先架起小车、轮子离地、确认遥控急停可用；再从低速地面试跑逐步增加，不要首次就用正式比赛场地高速运行。

正常巡线和赛前站立时，`posture_height_m`、`posture_roll_deg`、`posture_pitch_deg`、`posture_slide_m` 会在 0.5 秒平滑过渡；台阶接管时拒绝正常姿态改动和新旧策略切换。新台阶策略的后退对线、保存偏航、冲刺丢线起跳、落地找线参数已放在 `race_tuning.yaml` 的 H 段，可保存热加载；旧台阶动作、相机和视觉阈值仍是启动参数。详见 [新台阶流程](STEP_VISUAL_STRATEGY.md)。

人行横道顺序是：视觉确认 → 继续巡线 `approach_distance` → 停车 `crosswalk_stop_seconds` → 以 `crosswalk_clear_forward_speed` 直行 `crosswalk_clear_seconds`，这段时间完全忽略巡线和横道黑条 → 恢复找终点。默认停车 3 秒、清线 3 秒、清线速度 0.15 m/s。蜂鸣器当前只能记录并显示软件事件；即使打开 `buzzer_enabled`，ESP32 也不会发出物理声音。

`finish_mode: 0` 保留原流程：每次横道处理完再找黄色线，由黄色线换圈或结束。`finish_mode: 1` 用于赛道上黄色线不稳定的情况：第一处确认横道仍会接近、停车和清线，清线后直接进入下一圈；第二处确认横道不再停车，而是继续按 `second_crosswalk_finish_distance_m` 巡线后结束。第一次只在架空/低风险场地验证模式 1；确认第二处横道后遥控急停必须随时可用。

建议调试顺序：先架空加载并确认 HDMI 的“文件参数已生效”；再只改变一个直线速度值；弯道优先调 `drive_curve_*` 的速度帽，再调 `drive_linear_accel` 与 `drive_angular_accel`；在安全场地、有人持遥控急停时才故意遮住环岛线验证 `roundabout_blind_recovery_*`。不要一次同时提高直线、弯道和环岛速度，也不要把 Windows 本地测试当成 Orange Pi 或实车验证。

先分段，再单圈，最后两圈。失败后保留日志，按起跳位置、航向、速度、跳跃高度、落地段的顺序一次只调整一个参数，最后才考虑下位机 PID。

## 回退旧工作区

新增环岛逐帧跟随的流程、E 段调参与完整包部署步骤见 [ROUNDABOUT_FRAME_FOLLOW.md](ROUNDABOUT_FRAME_FOLLOW.md)。
源码更新后先成功编译一次，之后只保存实时表即可热加载。先视觉干跑，再有遥控急停的分段检查；Windows 自动测试不代表实车进环成功。

停止新工作区的节点，打开新终端，再切换到旧工作区：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/wheel_robot/install/setup.bash
```

回退只切换当前终端环境，不删除 `/home/orangepi/robot` 的日志和失败证据；不要在同一终端连续 source 新旧两个工作区后判断程序版本。
