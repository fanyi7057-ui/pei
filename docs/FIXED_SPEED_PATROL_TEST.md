# V16：新版 V15 基准 + VV 单帧环岛入口

## 修改边界

以用户2026-10-23重新提供的Two为基准。普通黑线/轮廓提取、PathSelector、FixedSpeedController、速度参数、共享IMU修复、台阶源码和固件均保留。入口只调用patrol_dash_entry.py，不再运行旧完整/部分虚线跨帧跟踪与EntryApproachProof；旧辅助类/字段只作历史离线测试与参数表兼容，不构成另一条入口授权路径。

现有两圈任务顺序、实际取线、实测偏航、真实出环凭据、横道停车与终点许可保留。台阶1/2仍仅SKIPPED_TEST占位，不代表已执行跳跃。

## 唯一入口流程

SEEK：原6秒/1米启用条件满足后，VV检测到当前纵向小黑块列，默认1个新帧确认；先记首次标记为需要通过的5号岔口，进入APPROACH，不立即左转。

APPROACH：继续现有巡线；若当前主虚线列有真实近端多行观测，沿它通过岔口，不追随返回的弯支线。没有该观测则按现有基础巡线/有界丢线保护，不造线、不倒车、不通过冷却命令停车，也不固定向左盲转。

合格虚线锁存后先沿当前路线前进 `ring_entry_advance_m=0.12`；当前帧确认左侧向前发散的弯曲虚线后才左探，累计到 `ring_entry_approach_max_m=0.90` 仍无证据就退回普通巡线。2026-11-25 19:31/19:33 两份采集显示：第一帧虚线出现的位置离真正岔口约 0.88m，旧值0.60会让APPROACH在左支刚进画面时就被距离上限释放、退回SEEK全速（0.30→0.61 m/s）冲过岔口；因此放大到0.90，让APPROACH有机会等到左支。5号向前汇聚的返回支线即使再次出现也不能入环。方向由弯支线相对主虚线的近/中间距变化判断，不能仅看绝对屏幕倾斜方向。

失败重找冷却（默认2秒/0.50米）内仍持续测量虚线：若当前帧已确认发散左支，`roundabout_entry_branch_rearm_enabled=1`允许立即重新锁存APPROACH并继续0.12m左探，不再等冷却走完。只豁免重找冷却，不绕过最初的6秒/1米启用门，也不把汇聚返回支线当成入口；置0可现场对照旧行为。2026-11-15三份实车采集显示左支恰好在冷却期出现，冷却结束再锁存时左支已消失，因此旧行为会连续错过入口。

ENTRY 固定左探 0.50s（约34°）后，只有在**当前帧仍确认发散左支或已看到可用环内路线**时才继续探左，最长到 `ring_entry_probe_max_seconds=0.90`（约62°）或累计转角 `ring_entry_probe_max_turn_deg=60°`，先到即停。2026-11-25 起再加两条“已承诺左转”规则，解决两份采集里ENTRY必然死在 `entry_probe_no_route`/`entry_confirm_timeout` 的问题（探左1.2 rad/s下分叉左支只存在约0.2s，之后当前帧一定无证据）：

1. **探左保持**：ENTRY开始时已由节点确认过VV虚线（`entry_dash_proven`）且累计转角还小于 `roundabout_entry_acquire_yaw_deg=55°` 时，即使当前帧已看不到左支也继续左探（仍受0.90s/60°上限约束）。这是转角证据，不是定时盲转。
2. **提交后交给巡线**：探左到时限时若已满足“VV虚线已证明 + 实际左转≥`roundabout_entry_min_yaw_rad=0.35`”，不再释放环岛任务回SEEK，而是置 `entry_visual_tracking` 由普通巡线接管（`route_status=entry_commit_follow`），交给原有有界ENTRY/FOLLOW合同（实测转角、行程、稳定路线、`roundabout_entry_acquire_max_seconds`）决定是否确认；丢线超过 `roundabout_path_dropout_seconds` 仍按 `entry_tracking_lost` 退出。

`ring_entry_probe_extend_enabled=0` 回到固定0.50s+0.25s。上面两条“已承诺左转”规则受 `roundabout_entry_commit_enabled` 控制：置 0 即完全回到旧行为（探左0.50s到期就释放回SEEK），便于单变量对照。达到上限且转角不足时仍以 `entry_probe_turn_limit`/`entry_confirm_timeout`/`entry_probe_no_route` 退回普通巡线。

3. **已经在弯里就不再补一次左探**（2026-11-25 20:13–20:16、20:38 采集）：有的入环位姿在VV虚线锁存时车**已经在沿弯走**——环岛左弯（上一帧指令 `w≥0.75*roundabout_entry_turn_rad_s≈0.9 rad/s`），或者已经在环岛自己的顺时针虚线圆上绕行（20:38 第三人称视频：车沿线自动进入环岛并一圈圈绕行，每圈经过岔口都会重新触发VV）。判定用“最近1秒内累计指令转角 `|Δyaw|≥0.35 rad`（约20°）”或上一帧强左转，任一满足即置 `entry_follow_only`：ENTRY 期间**不叠加盲转**、由普通巡线沿现有弯走，`route_status=entry_follow_existing_left_curve`，仍受 `roundabout_entry_acquire_max_seconds=3s` 超时约束；路线确认仍走原 ENTRY/FOLLOW 合同（实测转角/行程/稳定路线），只是不再靠盲转。直道居中的入口（最近1秒几乎没转、上一帧 `w` 很小）仍按上面1/2两条做探左。**注意**：20:38 证明“绕环时被VV岔口误触发”是摇头的直接来源；这个判定只阻止二次盲转，不解决“任务层始终没确认FOLLOW”的根本问题（绕环时路径在 `black_outside_locked_route`/丢帧之间闪，3帧连续路线难以成立）。

有新鲜odom时按实测位移/当前正向指令折算运动秒数，避免10Hz里程计与30Hz图像导致冷却被拉长；无odom沿用原命令估计，日志route_progress会区别来源。停车/倒车不累计运动时间，暂停不吃掉冷却，重复时间戳不计新帧；重试或第二圈清空首次标记。

ENTRY以后沿用V15：建立左转、实际取到环内路线后FOLLOW、确认真实出口后才发放本圈横道许可。当前真实弯支点仅初始化路线记忆，不等同已取线，更不能直接形成完成凭据。5号返回支线可作为FOLLOW的出口候选，但仍受原时间/行程/转角和真实EXIT路线确认约束。

真实 `visual_exit` 之后，`roundabout_post_exit_guard_enabled=1` 会启动 3.0 s 航向保护：保护期内普通巡线的大左转被限到 `roundabout_post_exit_left_limit_rad_s=0.20`，右修不受限，防止出口左支/回环分支把车再卷回环岛。只有真实 visual_exit 会启动；失败入环、时间释放、未确认出口都不启动，也不延长环岛任务或横道许可。暂停会顺延保护时间，开关置0可关闭。

单帧检测不等于任意一个黑点。仍要求同列小黑块、白路邻域、纵向白色间隙、近端弯支方向。二维几何不能绝对证明物理4/5位置；首次标记顺序与现场行驶方向必须正确，从入口中途开机可能先跳过它。

## 普通巡线丢线后的找线方向与弯前降速

SEEK 丢线后，控制器原来冻结丢线前的弯道转向。若丢线前最后一帧已明显看到线在另一侧（S弯反向、发卡弯），车会朝旧方向找线而越走越远。现在 `SearchSideMemory` 记录最近 0.25 s 内 `|near_error|>=0.30`、近/中同侧且非救线/交接帧的观测；丢线时把该侧作为**找线方向**（0.80 rad/s），当前帧再看到强侧观测也会即时纠正方向。它只影响找线方向，不确认路径、不刷新恢复里程/时间、不改速度；环岛任务仍使用自身的显式方向。

SEEK 中若线已到画面边缘（`max(|near_error|,|mid_error|)>=search_edge_cap_error=0.40`），或上一帧转向已接近饱和（`|w|>=search_edge_cap_prev_ratio=0.60*max_turn_rad_s`）且误差`>=0.30`，目标速度先降到 `search_recovery_speed_m_s=0.30`。另外换向帧（上一帧转向与当前误差方向相反、|prev_w|>=`reversal_angular_ratio=0.50*max_turn`、|error|>=0.20）进一步降到 `reversal_speed_m_s=0.22`（0=关闭）。只降速：不确认路径、不改转向增益/任务、不改变固定速度参数；速度按 `linear_accel_m_s2` 平滑下降（0.65→0.30 约0.54s，仍受此限制）。

急弯权限不再只看误差大小，还看当前路径置信度（参考 V2.3 的 `_steer`）：`corner_min_confidence=0.45` 以下不给急弯满权限，`corner_full_confidence=0.75` 才给满；急弯时误差滤波上限 `corner_filter_alpha_ceiling=0.88`，不再升到 1.0。低置信度的大误差多是画面在两块黑结构之间跳，旧写法会按满权限甩 2.0 rad/s，出弯后表现为持续抖动。三个参数都可热调，关闭门控可把 min/full 都设很低（不建议）。

2026-11-25 19:20/19:22 采集：第二个左弯 w 顶到 +2.0（0.30 m/s 下半径约0.15m），线被转到右侧后控制器猛反向，反向过程中丢线（两次都是同一形态，延续50/32帧右找）。先做单变量 `max_turn_rad_s: 2.0 → 1.5`（0.30 m/s 半径约0.20m）试验；若第一个弯转不够或第二个仍过冲，下一个单变量是 `steering_kd` 0.018→0.04 增加反向阻尼。日志 process 仅 11.8~18fps、algo 70~110ms，控制环延迟会放大过冲；必要时先关 `contour_takeover_enabled` 排除轮廓耗时。

找线转向强度默认从 0.80 提高到 1.10（`search_turn_rad_s`、`roundabout_search_turn_rad_s`）：0.30 m/s 下 0.80 对应半径 0.375 m，两段 11-16 采集的重获时间普遍 1.5~2.5s。左探失败释放后 0.8s 内（`roundabout_entry_search_seconds`）直接以左方向进入普通找线，不再先 `(0,0)` 直行/停车约 0.5s；窗口只改变找线方向，不确认路线、不延长环岛任务、不改速度任务。2026-11-25 起这个窗口内**也保持** `search_recovery_speed_m_s=0.30` 速度上限（即使当前路径有效）：19:33 采集在 `entry_approach_distance_limit` 释放后0.7s内从0.30加速到0.61 m/s，正好冲过环岛岔口。

同一环岛连续左探失败达到 `roundabout_max_failed_entries=2`（0=不限）时，报 `ring_entry_unconfirmed` 停车等人工，不再在环内无限绕圈、每圈重试；真实出环、重置课程或人工恢复会清零。approach 距离上限、时间释放等非探针失败不计入。2026-11-25 起 `time_fallback`（整圈时限释放，说明车已经绕出去很久）视为**新的尝试纪元**并把计数清零：19:31 采集就是在第一圈左探失败一次后、第二圈又失败一次时被旧“跨整段累计”逻辑停车并要求操作台输1；清零后仍能在同一纪元内两次连续探针失败时停车，但不会把上一次绕圈/上一圈的一次失败累计成冻结。

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

## 2026-11-25 19:31/19:33 环岛入口复盘

两份采集（`20261125_193112_6712` 1472帧/109.4s、`20261125_193329_6836` 1253帧/72.2s，代码 `6d0d6f7…`/`3419a8…`）的现象与结论：

- **19:31 第一次“第二圈停车要输1”**：第一圈13.51s ENTRY 0.5s后 `entry_probe_no_route`（第1次探针失败）；之后车沿环内线走了一圈，56.11s `time_fallback`，58.13s 再 APPROACH，60.1s ENTRY、60.76s 又失败（`entry_confirm_timeout`，第2次），累计达到 `roundabout_max_failed_entries=2` → `ring_entry_unconfirmed` 停车等人工；CSV 在60.76~85.39s 无帧，正是等待操作台输入1的24.6s。
- **19:33 第二次“看着能进但加速没进”**：47.2s 锁存APPROACH后48.9s距离已到旧上限0.60m，此时左支要到49.96s（advance≈0.88m）才出现；旧逻辑在49.26s按 `entry_approach_distance_limit` 释放回SEEK，随后0.4s内速度0.30→0.61 m/s冲过岔口，再锁存已晚。
- **ENTRY 永远确认不了的直接原因**：探左1.2 rad/s时，发散左支只在前约0.2s可见；黑线逐帧在“主虚线”和“左支虚线”之间跳（例如f238主虚线在黑估计里变成near +0.49，而记忆的入口支线是near -0.57），节点用“与转弯前的记忆引导一致”作路线门，于是逐帧 `black_outside_locked_route`；探左0.5s到期即释放，环岛任务被丢弃。之后普通巡线其实会沿环内线进环（f250/f280 可见蓝岛+环线），但任务层从未进入FOLLOW。

本轮改动（对应上面“唯一入口流程”与“找线方向”小节）：APPROACH上限0.90m、探左按“虚线已证明+未到55°”保持、到时限且已实际左转≥0.35rad时交给巡线（`entry_commit_follow`）、节点在已提交左转后接受新看到且稳定的黑线作为环内路线、左探失败后的0.8s找线窗口保持0.30m/s、`time_fallback` 清零跨纪元失败计数。

实车验证重点：观察日志中是否出现 `entry_commit_follow`、`route_status=entry_confirming_route/entry_confirm_extension` 连续帧、以及能否进入 `FOLLOW`；若已在环内却仍不确认，下一个单变量是 `roundabout_entry_acquire_max_seconds`（3.0s，提交后交给巡线到确认的总预算）与 `roundabout_entry_path_delta`（0.18，逐帧路线连续性；虚线跳变会打断3帧确认），不要同时改。若仍停车等人工，先看 `failed_entries` 是否同一纪元内累计两次，而不是跨圈累计。

## 2026-11-25 20:13/20:15/20:16 三次采集（修正后第一次实车）

三份采集：`20261125_201321_8221`（1620帧/93.3s，丢线3.9%）、`20261125_201517_8343`（429帧/24.6s，丢线41.3%）、`20261125_201623_8444`（1750帧/99.4s，丢线16.2%）。用户反馈：第一次能两次进入，之后几次“偏转进去的角度不对，而且会摇头”。CSV/视频核对：

- 三次采集都没有进入过 FOLLOW，ENTRY 都是盲探后释放；2026-11-25 的提交/交接改动确实生效（出现 `waiting_continuous_near_mid_route`、`entry_tracking_lost`）。
- 20:16 的第二次 ENTRY（f1045）：ENTRY 前 5 帧已经在沿左弯打满左舵（w +1.00~+1.49、near/mid −0.35~−0.87），ENTRY 再叠 1.2 rad/s 盲左探 → 实测累计转角 52°~80°，视频 f1058 已是“横看环内/蓝岛在左下”的过度入环位姿；随后丢线 40 帧、左找 1.1 rad/s 继续加转角，就是“角度不对”。
- 20:13 的 ENTRY（f1054）前车在向右修正（w −0.77、e +0.42），探左后 f1070 下一帧就拿到线（e −0.45），进入顺滑——说明直道/居中位姿的探左策略本身可以工作。
- 因此本轮只改“入环位姿分类”，不动探左参数：见“唯一入口流程”第3条 `entry_follow_only`。实车验证重点：日志出现 `entry_reason=vv_follow_existing_left_curve` / `route_status=entry_follow_existing_left_curve` 时，车应沿现有左弯平滑进环、不再多转一段；若这种位姿转不够（进不去），再把阈值 `0.75*roundabout_entry_turn_rad_s`（≈0.9 rad/s）调高，而不是改 `roundabout_entry_turn_rad_s`。

## 2026-11-25 20:38 采集 + 第三人称视频（entry_follow_only 第一次实车）

采集 `20261125_203818_9696`（1294帧/73.9s，丢线29.8%，代码 `5cd81cd4…`/`881ef56d…`），另有第三人称视频 72.1s（`3be84fee…mp4`）。用户反馈：**“第二次的转弯过早导致小车之后要摇头巡线”**。

视频+CSV对齐核对（视频 t≈CSV t+2s）：

- 视频 6–7s 车**自己沿线拐进环岛虚线圆**（没有VV触发、没有ENTRY），然后顺时针一圈圈绕行（8–18s 一圈、18–60s 继续绕），说明“车其实已经在环里”。
- 第一处 ENTRY（CSV t=18.1 / 视频≈16.5s）：车正在绕环，CSV 前5帧 w 从 +0.38 到 +1.20（左）又翻到 −0.85（右）——**绕环本身已经在左右摇头**；VV岔口一锁存，ENTRY 再强加 1.2 rad/s 左转（与当前 −0.85 右转相反），0.86s 后释放，随后左找 1.1 rad/s 丢线 4s+，车才重新抓线。这就是用户看到的“转弯过早→之后摇头”。
- 第二处 ENTRY（CSV t=60.6）：前几帧同样是绕环的 ±0.5 rad/s 左右摆动，ENTRY 又强加左转，丢线 39 帧。
- 两次ENRTY都没有确认进入 FOLLOW：ENTRY期间路径一直是 `black_outside_locked_route`/`line_too_few_bands`，3帧连续路线凑不出来；`heading_ok=0` 只是 SEEK 期 measured_turn 不再更新（不是IMU故障），但 ENTRY 内实测转角只有 +11.7°，离确认角55°很远，主因是轮足在蓝/白交界打滑、0.86s只实际转了约12°。

本轮改动：把 `entry_follow_only` 的判定从“上一帧强左转”扩展为“**最近1秒累计指令转角 |Δyaw|≥0.35 rad（约20°）**（或上一帧强左转）”——绕环、左弯这两种“已经在弯里”的位姿都不再叠加盲左探，由巡线沿现有曲线走；直道/居中位姿仍然探左。实测验证重点：日志出现 `entry_reason=vv_follow_existing_curve` 时车应保持原曲线、不再有多余左转和随后的大幅左找。**仍要说明**：这不解决“绕环时任务始终不确认FOLLOW”的根本问题——要确认需要路线连续+实测转角，而绕环时两者都难满足，下一步要么降低确认门槛/容忍闪帧，要么换“入环后按已绕行角度确认”的策略（需另立版本）。
