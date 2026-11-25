# V16：新版 V15 基准 + VV 单帧环岛入口

## 修改边界

以用户2026-10-23重新提供的Two为基准。普通黑线/轮廓提取、PathSelector、FixedSpeedController、速度参数、共享IMU修复、台阶源码和固件均保留。入口只调用patrol_dash_entry.py，不再运行旧完整/部分虚线跨帧跟踪与EntryApproachProof；旧辅助类/字段只作历史离线测试与参数表兼容，不构成另一条入口授权路径。

现有两圈任务顺序、实际取线、实测偏航、真实出环凭据、横道停车与终点许可保留。台阶1/2仍仅SKIPPED_TEST占位，不代表已执行跳跃。

## 唯一入口流程

SEEK：原6秒/1米启用条件满足后，VV检测到当前纵向小黑块列，默认1个新帧确认；先记首次标记为需要通过的5号岔口，进入APPROACH，不立即左转。

APPROACH：继续现有巡线；若当前主虚线列有真实近端多行观测，沿它通过岔口，不追随返回的弯支线。没有该观测则按现有基础巡线/有界丢线保护，不造线、不倒车、不通过冷却命令停车，也不固定向左盲转。

合格虚线锁存后先沿当前路线前进 `ring_entry_advance_m=0.12`；当前帧确认左侧向前发散的弯曲虚线后才左探，累计到 `ring_entry_approach_max_m=0.60` 仍无证据就退回普通巡线。5号向前汇聚的返回支线即使再次出现也不能入环。方向由弯支线相对主虚线的近/中间距变化判断，不能仅看绝对屏幕倾斜方向。

失败重找冷却（默认2秒/0.50米）内仍持续测量虚线：若当前帧已确认发散左支，`roundabout_entry_branch_rearm_enabled=1`允许立即重新锁存APPROACH并继续0.12m左探，不再等冷却走完。只豁免重找冷却，不绕过最初的6秒/1米启用门，也不把汇聚返回支线当成入口；置0可现场对照旧行为。2026-11-15三份实车采集显示左支恰好在冷却期出现，冷却结束再锁存时左支已消失，因此旧行为会连续错过入口。

ENTRY 固定左探 0.50s（约34°）后，只有在**当前帧仍确认发散左支或已看到可用环内路线**时才继续探左，最长到 `ring_entry_probe_max_seconds=0.90`（约62°）或累计转角 `ring_entry_probe_max_turn_deg=60°`，先到即停；证据一消失立即按原0.25s扩展/超时退出。`ring_entry_probe_extend_enabled=0` 回到固定0.50s+0.25s。这条规则不制造盲转：达到上限仍无路线就以 `entry_probe_turn_limit`/`entry_confirm_timeout` 退回普通巡线。

有新鲜odom时按实测位移/当前正向指令折算运动秒数，避免10Hz里程计与30Hz图像导致冷却被拉长；无odom沿用原命令估计，日志route_progress会区别来源。停车/倒车不累计运动时间，暂停不吃掉冷却，重复时间戳不计新帧；重试或第二圈清空首次标记。

ENTRY以后沿用V15：建立左转、实际取到环内路线后FOLLOW、确认真实出口后才发放本圈横道许可。当前真实弯支点仅初始化路线记忆，不等同已取线，更不能直接形成完成凭据。5号返回支线可作为FOLLOW的出口候选，但仍受原时间/行程/转角和真实EXIT路线确认约束。

真实 `visual_exit` 之后，`roundabout_post_exit_guard_enabled=1` 会启动 3.0 s 航向保护：保护期内普通巡线的大左转被限到 `roundabout_post_exit_left_limit_rad_s=0.20`，右修不受限，防止出口左支/回环分支把车再卷回环岛。只有真实 visual_exit 会启动；失败入环、时间释放、未确认出口都不启动，也不延长环岛任务或横道许可。暂停会顺延保护时间，开关置0可关闭。

单帧检测不等于任意一个黑点。仍要求同列小黑块、白路邻域、纵向白色间隙、近端弯支方向。二维几何不能绝对证明物理4/5位置；首次标记顺序与现场行驶方向必须正确，从入口中途开机可能先跳过它。

## 普通巡线丢线后的找线方向与弯前降速

SEEK 丢线后，控制器原来冻结丢线前的弯道转向。若丢线前最后一帧已明显看到线在另一侧（S弯反向、发卡弯），车会朝旧方向找线而越走越远。现在 `SearchSideMemory` 记录最近 0.25 s 内 `|near_error|>=0.30`、近/中同侧且非救线/交接帧的观测；丢线时把该侧作为**找线方向**（0.80 rad/s），当前帧再看到强侧观测也会即时纠正方向。它只影响找线方向，不确认路径、不刷新恢复里程/时间、不改速度；环岛任务仍使用自身的显式方向。

SEEK 中若线已到画面边缘（`max(|near_error|,|mid_error|)>=search_edge_cap_error=0.40`），或上一帧转向已接近饱和（`|w|>=search_edge_cap_prev_ratio=0.60*max_turn_rad_s`）且误差`>=0.30`，目标速度先降到 `search_recovery_speed_m_s=0.30`。另外换向帧（上一帧转向与当前误差方向相反、|prev_w|>=`reversal_angular_ratio=0.50*max_turn`、|error|>=0.20）进一步降到 `reversal_speed_m_s=0.22`（0=关闭）。只降速：不确认路径、不改转向增益/任务、不改变固定速度参数；速度按 `linear_accel_m_s2` 平滑下降（0.65→0.30 约0.54s，仍受此限制）。

急弯权限不再只看误差大小，还看当前路径置信度（参考 V2.3 的 `_steer`）：`corner_min_confidence=0.45` 以下不给急弯满权限，`corner_full_confidence=0.75` 才给满；急弯时误差滤波上限 `corner_filter_alpha_ceiling=0.88`，不再升到 1.0。低置信度的大误差多是画面在两块黑结构之间跳，旧写法会按满权限甩 2.0 rad/s，出弯后表现为持续抖动。三个参数都可热调，关闭门控可把 min/full 都设很低（不建议）。

2026-11-25 19:20/19:22 采集：第二个左弯 w 顶到 +2.0（0.30 m/s 下半径约0.15m），线被转到右侧后控制器猛反向，反向过程中丢线（两次都是同一形态，延续50/32帧右找）。先做单变量 `max_turn_rad_s: 2.0 → 1.5`（0.30 m/s 半径约0.20m）试验；若第一个弯转不够或第二个仍过冲，下一个单变量是 `steering_kd` 0.018→0.04 增加反向阻尼。日志 process 仅 11.8~18fps、algo 70~110ms，控制环延迟会放大过冲；必要时先关 `contour_takeover_enabled` 排除轮廓耗时。

找线转向强度默认从 0.80 提高到 1.10（`search_turn_rad_s`、`roundabout_search_turn_rad_s`）：0.30 m/s 下 0.80 对应半径 0.375 m，两段 11-16 采集的重获时间普遍 1.5~2.5s。左探失败释放后 0.8s 内（`roundabout_entry_search_seconds`）直接以左方向进入普通找线，不再先 `(0,0)` 直行/停车约 0.5s；窗口只改变找线方向，不确认路线、不延长环岛任务、不改速度任务。

同一环岛连续左探失败达到 `roundabout_max_failed_entries=2`（0=不限）时，报 `ring_entry_unconfirmed` 停车等人工，不再在环内无限绕圈、每圈重试；真实出环、重置课程或人工恢复会清零。approach 距离上限、时间释放等非探针失败不计入。

入环左探起点由 `ring_entry_advance_m` 控制（默认0.12，2026-11-16实验改为0.25）：锁存虚线支线后先在主线上走这么远再探左。20:05采集显示0.12时探左0.5s就把入环纵向虚线甩到画面右缘，随后回正错过横线；先只延后起点验证“偏转过早”。若25cm太晚则回调0.18~0.20；若仍丢线，下一步单变量是把 `roundabout_entry_turn_rad_s` 1.2→0.8，而不是再改这个距离。

2026-11-23 采集暴露另一种入环位姿：**没有分叉左支，只有一条向左弯的主虚线链**（VV main 有效、branch=0）。曾允许主链连续左弯作为探左证据（`ring_entry_main_curve_*`），但 19:53 采集显示**车自己正在走弯时也会满足**，造成过早/错误进环；现已把 `ring_entry_main_curve_enabled` 设为 0，回到只认分叉左支（代码保留），宁可不进也不错进。相关参数：`ring_entry_main_curve_near=-0.03`、`mid=-0.08`、`frames=2`、`min_near_ratio=0.75`、`max_near=0.25`、`max_mid=0.30`。

19:15 采集又发现“主链左弯”会被**车自己偏左**误满足（near -0.55/mid -0.64 也触发），造成误入环岛；现在再加 `ring_entry_main_curve_max_near=0.25`、`ring_entry_main_curve_max_mid=0.30`，只允许接近居中、略向左弯的主链。离线重放：19:15 不再误入 ENTRY，19:02 的真实入环仍保留。

道路/轮廓兜底路径置信度低时不再全速：`road_low_confidence_cap=0.45`（0=关闭），SEEK 中 `source=="road"` 且 `confidence<0.45` 时目标速度降到 `search_recovery_speed_m_s=0.30`。19:15 采集 S 弯前正是 road conf 0.23~0.34 仍以 0.65 行驶后丢线。

## 轮廓/拐点巡线（patrol_contour_line.py，2026-11-17）

普通巡线新增一条并行的轮廓链：连通域道路掩膜（白+不确定灰，排除蓝）→逐行左右边线→中线/near/mid/far 误差。开关：

| 参数 | 默认 | 作用 |
|---|---:|---|
| contour_takeover_enabled | 1 | 黑线和道路路径都无效时，用轮廓中线做短时接管 |
| contour_corner_enabled | 0 | 拐点拉线；2026-11-17 19:50 实车转向被拖慢/误拉后关闭，仅保留代码 |
| contour_corner_min_angle_deg | 55 | 拐点最小转角，越小越敏感 |
| contour_corner_entry_pull / seconds | 0.28 / 0.80 | 入环拐点向左拉线量与持续 |
| contour_corner_exit_pull / seconds | 0.22 / 0.90 | 出环右折角向左拉线量与持续 |
| contour_corner_cooldown_seconds | 2.0 | 同类拐点重复触发间隔 |
| contour_takeover_max_seconds | 0.60 | 轮廓接管总时长上限；超时回普通搜索 |
| contour_takeover_max_gap_seconds | 0.35 | 距最近黑线最长时间；超过按真正丢线搜索 |
| contour_corner_min_ring_seconds | 1.50 | 入环后至少多久才允许出环拐点 |

轮廓接管只在“可靠单车道走廊”上启用：dual_fraction≥0.55、宽度 0.18~0.80、宽度MAD≤0.22、分叉行≤0.10、连通域面积≤0.85；不满足时 `corridor=False`，不接管、不触发拐点。2026-11-17 采集按新门复算 0/581 帧满足（该场地白地面是宽大厅+蓝胶带，不是封闭车道），所以该版轮廓链只做诊断，实车回到黑线+搜索；要让轮廓参与控车，需改用蓝胶带/道路双边作为边界。

方案B第一版已把边界源改为**蓝胶带双边**：每行取参考中心（最近黑线 near/mid 构造，否则图像中心）左右最近蓝胶带内沿作左右边线；只看到一侧时用跨帧宽度先验补另一侧（`blue_single_edge`）。接管/拐点额外要求蓝中线与最近黑线 near/mid 差 ≤ `contour_takeover_agree_delta=0.30`。离线验证 581 帧：蓝走廊 234 帧，但与黑线 near |Δ| 中位数 0.43 / p90 1.35，mid 0.55 / p90 1.40，agree 门把绝大多数挡在控车之外——蓝胶带当前太断续、配对偏移大，尚不能接管；下一步做跨帧跟踪或复用 `road_centerline` 双边。

2026-11-17 19:50 实车定位到更严重的问题：轮廓每帧计算（约15.3ms）把控制环从 ~24fps 拖到 **~12fps**（dt 86ms），转弯指令延迟导致“转不过去”；拐点拉线还会在低置信度道路路径（conf 0.04）上误触发、和右转抢方向。现在：`contour_corner_enabled=0`（拐点只留代码），`contour_takeover_enabled=1` 作为纯辅助，且 `contour_update_seconds=0.15` 节流、只在黑线丢失/最近丢过时计算；正常巡线恢复黑线主控。

拐点窗口内忽略旧 VV 的 `roundabout_left_acquire` 覆盖；VISION 日志有 `corner=`/`cobs=`。参数/区域/拉线量按 `records/2026-11-17-contour-corner-nav.md` 的顺序标定后再打开 `contour_corner_enabled`。

| 参数 | 默认 | 含义 |
|---|---:|---|
| corner_min_confidence | 0.45 | 低于此置信度不授予急弯满权限 |
| corner_full_confidence | 0.75 | 达到此置信度才给满急弯权限 |
| corner_filter_alpha_ceiling | 0.88 | 急弯时滤波 alpha 上限，替代原来的 1.0 |

2026-11-15 15:39 采集里，757 帧丢线前 756 帧黑线在左侧 near=-0.83，旧逻辑却按丢线前的右转继续向右找线 90 帧；998 帧丢线前 997 帧线在右侧 near=+0.47，旧逻辑仍向左找线。17:05 采集的 452 帧 near=-0.31/mid=-0.77 已明显在左，旧阈值 0.45 没有记录，456 帧起向右找了 64 帧；现在 0.30 阈值会改为向左。

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
| roundabout_entry_branch_rearm_enabled | 1 | 重找冷却内已确认发散左支即可重新锁存；0=旧行为，只等冷却 |
| ring_entry_probe_extend_enabled | 1 | ENTRY探左按当前证据延长；0=固定0.50s+0.25s |
| ring_entry_probe_max_seconds | 0.90 | 有证据时探左最长时间，秒，须≥ring_entry_probe_seconds |
| ring_entry_probe_max_turn_deg | 60.0 | 有证据时探左最大累计转角，度 |
| roundabout_post_exit_guard_enabled | 1 | 仅真实visual_exit后启动出环后航向保护；0=关闭 |
| roundabout_post_exit_guard_seconds | 3.0 | 保护时长，秒；暂停顺延 |
| roundabout_post_exit_left_limit_rad_s | 0.20 | 保护期内最大左转角速度；右修不受限 |

0.65m/s时1秒约前进0.65米；两个岔口间距更短时，1秒可能过长。先看cooldown_remaining、marker_travel与当前入口形态，不能一味加大冷却。

速度不变：fixed_speed_m_s=0.65；roundabout_follow_speed_multiplier=1.30，仅满足原可靠FOLLOW/稳定/实测转角条件才可能目标0.845m/s。入环角速度roundabout_entry_turn_rad_s=1.20，建立角度roundabout_entry_acquire_yaw_deg=55；本次没有另叠加0.5秒强转。环内倍率偏快可另行对照1.0，但本次未擅自修改。

偏航参数保持：roundabout_heading_source=0自动选有效IMU/odom；roundabout_loop_min_turn_deg=300、loop_return_tolerance_deg=35、loop_min_distance_m=1.0；这些只参与环行进展证明，require_exit_route=1仍要求真实出环，不能拿原地自旋当完成。

## 日志排查

应出现version=16、entry_detector=vv。重点：
- entry_reason=vv_first_marker_pass_5：首次标记；
- entry_reason=vv_rearm_on_confirmed_left_branch：重找冷却内左支已确认，提前重新锁存；
- ring_release_source=entry_probe_turn_limit：有证据探左到60°上限仍未取线，按原规则退出；
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
