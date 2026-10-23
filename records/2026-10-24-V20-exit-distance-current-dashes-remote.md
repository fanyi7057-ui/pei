# V20：出口距离、环内当前虚线与遥控证据隔离

基准V19 3d89e73。日志d480df5f第二段5522 ENTRY、5526 FOLLOW，5529后丢线；5532约360度提名EXIT，5533一秒无可靠线保护。人工1继续后5541才有visual_exit，不是无干预完赛证明。

## 改动与不变项

- gyro出口找线从旧1秒/名义0.12米，改为实测前向目标0.22米或2.3秒先到先保护，多约0.1米。航向偏离既有35度容差或过期立即保护；当前可靠路线优先接管，但仍需原出口稳定/距离证明，不能冒充出环。
- 候选起点记录odom/yaw，距离取前向净位移投影；侧移/倒车不给正向距离。无odom或起点无odom时没有实测距离声明，时间上限仍有效。
- FOLLOW现有窄窗匹配失败时，0.22图宽内按已选曲线形状做当前块平移共识；至少2个分离的实测黑块，3块近端证据才可能route_observed，当前白色两侧筛选仍保留。单点、错误形状、相近双候选拒绝。仅保底，不覆盖可靠黑线/道路，旧路线不能生成假像素。ENTRY和入口识别不启用此扩展。
- SBUS保护：核实firmware/LeggedRobot/LeggedRobot.ino SelectTelecontrol=1、SBUS_chMin=192/chMax=1792，sbus_swa=map(sbus[4],192,1792,0,2)，2才上位机。robot_mode区分底盘/关节，不可替代遥控档。
- 新鲜10通道/sbus_channels的SWA达到1792才允许实车ROS运动；unknown/manual/stale时保持零速和预览，不退出、不自动恢复。未完成环岛作废并重找，保留已完成圈/横道凭据，暂停重置IMU/odom证据，切回自动后还需1。没有新图时也先执行接管保护，不重发旧速度。
- 本监测证明开关状态，不证明实际执行ROS；ROS话题时间也不能替代射频failsafe。固件SelectTelecontrol=0或其他映射必须重新校准，不可照搬阈值。当前不改固件/C++，不要求重刷或重新编译。
- 普通0.65、环内上限0.30、预引导0.12、入口方向、横道0.25米/3秒、原始PathSelector/FixedSpeedController、台阶测试禁用均不变。

## 参数文件

ros2_ws/src/wheel_robot/config/patrol_test.yaml：

- ring_gyro_exit_seek_distance_m=0.22：gyro候选后找出口的前向净位移目标。
- ring_gyro_exit_seek_seconds=2.3：硬上限，不等于必定走够目标；小于等于exit_acquire_max_seconds=2.5。
- ring_curve_reacquire_enabled=1；max_shift_ratio=0.22：FOLLOW局部当前块找回开关/范围，非入口条件。
- remote_guard_enabled=1；remote_auto_channel_index=4；remote_auto_channel_min=1792；remote_signal_timeout_seconds=0.35：对应本包已核对的SWA通道及ROS话题期限。

## 实车边界清单

1. 先视觉干跑（不依赖SWA运动许可）；实车自动档输入1，看到version20、authority=auto_switch、remote_raw。
2. 0.65普通/0.30环内上限保持；4号入口仍能进入，5号不能反进。
3. FOLLOW虚线偏出旧窗后当前块找回；遮挡、单点、两条近似线不得乱跳，完全无像素仍保护。
4. 360候选后的exit_seek_m增长至约0.22或先找到路线；停滞2.3秒保护；缺odom、反向、侧移、航向跳变均不能假完成。
5. 环内切遥控：原任务冻结、当次未完成环岛作废；任意搬动/转向不计证据；切自动不会自行跑，输入1才继续重找。已完成圈不清零。
6. 拔掉SBUS话题/停止发布、非法消息、重复相机帧时零速保护；射频丢失另按固件failsafe验证。
7. 人行道仍0.25米接近/停3秒，两圈/终点顺序与初次站立确认不变。

## 部署

## 离线结果与检查限制

巡线/VV/V17/V18/V19/V20关联623项通过；新增V20共26项，含实节点速度、遥控接管、重复图、干跑、当前图像曲线重定位及不覆盖可靠黑线。全套790通过、11跳过、23失败，失败名称与V19逐项一致。Python3.10语法、重复定义、LF无BOM、基础PathSelector/FixedSpeedController源码保持不变与git diff --check通过。

本机Git Bash初始化被沙箱拒绝（NtCreateDirectoryObject权限），本次不能声称执行通过bash -n；两个菜单只改版本号和单引号printf提示，不改控制结构。未做ROS2/香橙派实车验证。

### 运行方式

同步E:/Two本版内容到/home/orangepi/Two_patrol；不要混用旧目录。已有正常编译底盘时，本次Python修改不需编译。

`cd /home/orangepi/Two_patrol`

`sha256sum -c checksums/PATROL_TEST_ADDITIONS_SHA256SUMS.txt && bash tools/patrol_test_menu.sh`

先1干跑，再重启选2实车；SWA自动档、原调零/使能、站立完成后1开始；2暂停、3新场、0退出，保留遥控急停。离线测试不能保证实车完赛；0.22米是odom目标，不是无误差世界坐标定位。
