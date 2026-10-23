# V17：缩短入环通过距离、持续虚线接管与360度出口保底

基准是E:/Two的V16（585823a）。普通巡线、黑线/轮廓预处理、PathSelector、FixedSpeedController、原比赛/台阶和C++底盘不改。只改环岛分支及失败后的有界恢复。

## 这次对应日志929a5007

第一趟物理左拐时程序一直APPROACH，未进入ENTRY；第一次普通主虚线被误当成需要通过的5号。第二趟启动ENTRY却始终route_owned=0，在55度后失去固定引导，最后约178度放弃，再以0.65m/s盲搜。日志没有FOLLOW/EXIT，环内1.3倍率未启用，不是360度误判完成。

## 当前流程

- 只有主虚线：候选，不认定已遇到5号、不启动冷却。
- 明确5号向前汇聚：继续沿当前主路通过，冷却0.5等效运动秒，同时至少前进0.12米。
- 明确4号向前发散：可直接接管，不把真实入口当成第一次应忽略的出口。几何仍是候选证据，不能声称二维画面绝不误判。
- APPPROACH/ENTRY/FOLLOW/EXIT线速度统一硬上限0.30m/s。普通SEEK/非环岛巡线仍0.65m/s；旧1.3倍率不再突破环岛上限。
- 入环以后沿当前已选弯曲小黑块跟踪，不再要求直行主虚线同时存在；每帧必须有真实黑块，记忆只用于关联，不绘制假线。
- 当前真实同路虚线取得控制优先，55度不再是唯一交接条件。达到55度仍无可靠路线时，以0.12m/s、0.35rad/s温和左探；获取总期限3秒，即使IMU缺失也有期限。
- 失败后清除旧搜索方向/速度，最多1秒低速恢复，3个连续真实路线新帧才回普通巡线；没有可靠路径则保护停车等人工，不再高速盲搜。人工1继续会解除过期恢复期限，但从未看见路径时原控制器仍不会盲起步。
- 保留实际入环、真实EXIT、每圈环岛/横道完成凭据；不能靠时间或自旋假装完成。两圈台阶仍SKIPPED_TEST占位，不执行跳跃。

## 唯一参数表

ros2_ws/src/wheel_robot/config/patrol_test.yaml。完整覆盖Python/新脚本/YAML后重启一次；后续改表约0.5秒热加载。开关用0/1，非法整表不应用。

| 参数 | 默认 | 含义 |
|---|---:|---|
| vv_dash_confirm_frames | 1 | 同类型当前虚线新帧确认数，主路/入口/出口不能互相预填 |
| vv_dash_min_blocks | 4 | 新入口识别的主虚线最少独立黑块数；已选路线续跟不要求直行主线仍在 |
| roundabout_first_marker_cooldown_seconds | 0.5 | 仅明确5号后的等效运动时间 |
| roundabout_first_marker_pass_distance_m | 0.12 | 冷却同时满足的最小前进行程 |
| roundabout_speed_m_s | 0.30 | 接近、入环、环内、出环的线速度上限，普通巡线不受影响 |
| roundabout_entry_turn_rad_s | 1.20 | 建立入环的左探角速度rad/s，仍受全局最大角速度约束 |
| ring_gyro_exit_enabled | 1 | 使用现有可靠IMU/odom偏航的保底开关，不新增第二套积分 |
| ring_gyro_exit_degrees | 360 | 从ENTRY起的保底净转角，另需已实际取得环内路线 |
| ring_gyro_min_distance_m | 0.80 | 保底所需实际行程；没有实测位移时原地转圈不能达标 |
| ring_gyro_exit_seek_seconds | 1.0 | 达标后无可靠线时低速直探出口的最多时间 |
| ring_recovery_speed_m_s | 0.12 | 未取线温和搜索、失败恢复、无路线找出口的速度 |
| ring_recovery_turn_rad_s | 0.35 | 未取线温和左探与失败恢复角速度rad/s |
| ring_recovery_seconds | 1.0 | 入环失败恢复的最多时间 |

冷却用实测位移/当前正向指令折算运动秒，无odom沿用命令估计并打印来源；因此默认名义通过约0.30×0.5=0.15米，不是0.65米。0.12米只是同时要求的下限，不能保证准确物理位置，真实速度/惯性需实测。

旧roundabout_loop_min_turn_deg和roundabout_loop_min_distance_m只兼容读取，V17保底目标改调ring_gyro_*；回向容差与传感器新鲜度/一致性检查仍保留。旧入口跨帧证明参数不参与当前入口判定。

## 保底不是直接判完成

现有MeasuredRingTurn解绕实测偏航，必须已经FOLLOW、净转角达到360度、实测行程至少0.80米且同向性/新鲜度可靠，才提名EXIT。可靠视觉优先；没有可靠路径时以恢复速度直探最多1秒，偏航陈旧或到期仍无路线则保护停车。之后仍须真实出口路线确认与行程才能授权横道。

## 复制与MobaXterm

结束旧程序，备份车上可用目录，将E:/Two内容合并到/home/orangepi/Two_patrol，不能套Two/Two。务必复制patrol_dash_entry.py和完整新参数表。

```bash
bash /home/orangepi/Two_patrol/tools/robot_menu.sh
```

先2视觉干跑，再3站立、4实测。已编译新基准且有IMU_QUATERNION_MAPPING_V12底盘标记时，本次无需重编C++。否则先1编译。进入测试后1开始/继续、2暂停、3重置起点、0退出；遥控急停保持可用。

应见version=17、ring_cap=0.30。关注entry_reason、vv_reason、reason=vv_owned_curve、route_owned、route_frames、measured_turn_deg、ring_receipts、mode=ring_failure_recovery/gyro_exit_seek、PATROL_ROUTE_FAULT。

## 测试边界与风险

依次测试普通直线/弯道/S弯不变，明确5号通过距离，4号直接接管，主线离开视野后的弯曲虚线，55度未取线的低速控制，3秒无路线退出，1秒失败恢复与人工继续，360度但无行程拒绝、偏航陈旧停车、真实出口和两圈横道顺序。

单帧仍可能误判复杂黑块/反光；连续虚线与其它线接近时可能串线；0.30/1.20的名义半径约0.25米，未标定地图半径或平衡能力；1秒恢复失败会停车，不承诺永不停车。离线合成图像与状态机测试不代表Orange Pi帧率、实车已完赛。校验使用PATROL_TEST_ADDITIONS_SHA256SUMS，历史SHA256SUMS的既有失败单独列明。
