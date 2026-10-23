# V16：新版 V15 基准 + VV 单帧环岛入口

## 修改边界

以用户2026-10-23重新提供的Two为基准。普通黑线/轮廓提取、PathSelector、FixedSpeedController、速度参数、共享IMU修复、台阶源码和固件均保留。入口只调用patrol_dash_entry.py，不再运行旧完整/部分虚线跨帧跟踪与EntryApproachProof；旧辅助类/字段只作历史离线测试与参数表兼容，不构成另一条入口授权路径。

现有两圈任务顺序、实际取线、实测偏航、真实出环凭据、横道停车与终点许可保留。台阶1/2仍仅SKIPPED_TEST占位，不代表已执行跳跃。

## 唯一入口流程

SEEK：原6秒/1米启用条件满足后，VV检测到当前纵向小黑块列，默认1个新帧确认；先记首次标记为需要通过的5号岔口，进入APPROACH，不立即左转。

APPROACH：继续现有巡线；若当前主虚线列有真实近端多行观测，沿它通过岔口，不追随返回的弯支线。没有该观测则按现有基础巡线/有界丢线保护，不造线、不倒车、不通过冷却命令停车，也不固定向左盲转。

默认经过1.0等效运动秒且至少前进0.20米，才允许当前左侧向前发散的弯曲虚线触发ENTRY。5号向前汇聚的返回支线即使再次出现也不能入环。方向由弯支线相对主虚线的近/中间距变化判断，不能仅看绝对屏幕倾斜方向。

有新鲜odom时按实测位移/当前正向指令折算运动秒数，避免10Hz里程计与30Hz图像导致冷却被拉长；无odom沿用原命令估计，日志route_progress会区别来源。停车/倒车不累计运动时间，暂停不吃掉冷却，重复时间戳不计新帧；重试或第二圈清空首次标记。

ENTRY以后沿用V15：建立左转、实际取到环内路线后FOLLOW、确认真实出口后才发放本圈横道许可。当前真实弯支点仅初始化路线记忆，不等同已取线，更不能直接形成完成凭据。5号返回支线可作为FOLLOW的出口候选，但仍受原时间/行程/转角和真实EXIT路线确认约束。

单帧检测不等于任意一个黑点。仍要求同列小黑块、白路邻域、纵向白色间隙、近端弯支方向。二维几何不能绝对证明物理4/5位置；首次标记顺序与现场行驶方向必须正确，从入口中途开机可能先跳过它。

## 参数位置

唯一热加载表：ros2_ws/src/wheel_robot/config/patrol_test.yaml。必须将本版Python、全部YAML和新增脚本一起覆盖后重启一次，后续只调数值约0.5秒热加载。非法整表保留上一有效值；不混用旧参数表。

| 参数 | 默认 | 含义 |
|---|---:|---|
| vv_dash_confirm_frames | 1 | 首次标记/当前入口所需连续新帧；二者计数分开，出口帧不能预填入口计数 |
| vv_dash_min_blocks | 4 | 同一纵向列最少黑块数 |
| vv_dash_roi_top_ratio / bottom_ratio | 0.22 / 0.90 | 仅VV检测ROI，不改普通巡线ROI |
| vv_dash_max_x_spread_px80 | 18 | 80列参考网格上的虚线列横向容差 |
| vv_dash_min_gap_rows56 | 3 | 56行参考网格上的块间白色行数 |
| vv_dash_entry_near_ratio | 0.66 | 当前左弯支线至少到达的整图高度比例 |
| roundabout_first_marker_cooldown_seconds | 1.0 | 首次标记后的等效运动时间，非停车等待 |
| roundabout_first_marker_pass_distance_m | 0.20 | 冷却同时要求的最少前进行程 |

0.65m/s时1秒约前进0.65米；两个岔口间距更短时，1秒可能过长。先看cooldown_remaining、marker_travel与当前入口形态，不能一味加大冷却。

速度不变：fixed_speed_m_s=0.65；roundabout_follow_speed_multiplier=1.30，仅满足原可靠FOLLOW/稳定/实测转角条件才可能目标0.845m/s。入环角速度roundabout_entry_turn_rad_s=1.20，建立角度roundabout_entry_acquire_yaw_deg=55；本次没有另叠加0.5秒强转。环内倍率偏快可另行对照1.0，但本次未擅自修改。

偏航参数保持：roundabout_heading_source=0自动选有效IMU/odom；roundabout_loop_min_turn_deg=300、loop_return_tolerance_deg=35、loop_min_distance_m=1.0；这些只参与环行进展证明，require_exit_route=1仍要求真实出环，不能拿原地自旋当完成。

## 日志排查

应出现version=16、entry_detector=vv。重点：
- entry_reason=vv_first_marker_pass_5：首次标记；
- vv_cooling_keep_tracking：冷却并巡线，不是停车命令；
- vv_reject_return_or_no_left_fork：仍为5号返回形态，或当前没有可靠左弯分支；
- vv_current_left_diverging_entry_4：当前4号入口触发；
- vv_blocks、vv_entry、vv_exit、vv_entry_frames：结构与新帧计数；
- cooldown_remaining、marker_travel、route_progress：通过进度及其来源；
- route_status、heading_reason、ring_receipts、cw_block：取线/偏航/两圈许可。

## 复制与MobaXterm运行

先在原终端结束程序，备份车上可用目录，将E盘Two内容复制/合并到/home/orangepi/Two_patrol，不能多套一层Two。新增patrol_dash_entry.py必须复制，不能只传两个旧Python。

```bash
bash /home/orangepi/Two_patrol/tools/robot_menu.sh
```

菜单2视觉干跑，检查路线和VV日志；再3站立检查、4实测。若首次部署该新包或驱动不含IMU_QUATERNION_MAPPING_V12，先菜单1编译。已经正确部署本次新基准的共享底盘，本次只改Python/YAML/说明，无需重编译C++。菜单1/2/3/4含义以屏幕为准。

启动后1开始/继续、2暂停、3重置起点任务、0退出；遥控急停保持可用。输入3后不会自行运动。

## 验证与风险清单

离线需覆盖：普通类源码摘要不变；实际图像单帧4入口与5返回分离；主路倾斜仍保持相对分支判断；实线/横道/蓝区/少量噪点拒绝；冷却持续巡线；静止odom/倒车/重复帧/暂停不能绕过顺序；低频odom不拉长冷却；第二圈清标记；真实EXIT与横道/终点凭据不能越级。

实车依次测试直线、普通弯、S弯、5号通过、4号进入、弧顶短失线、5号驶出、第一横道停3秒、第二圈重识别及终点。另测反光、入口只闪1帧、缺odom、偏航不一致、帧率降低与相机断流。

风险：1秒可能错过近距离入口；单帧误检风险高于多帧；首次标记误识别会错置顺序；中途放车不能当起点；环内0.845m/s可能仍太快；轮滑/人工搬车会损害里程计证据。保留相机超时、定位最终保护与人工急停，不保证永不停车、无限盲走或已实车完赛。

原SHA256SUMS是新包继承的历史清单，基准本身已有不一致；当前增量以PATROL_TEST_ADDITIONS_SHA256SUMS核对。既有整包失败必须与未修改基准对照记录，不能把合成测试当实车验证。
