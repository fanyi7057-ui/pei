# 当前 V14：恢复普通巡航、独立搜索防自转（不录像）

当前改动详见 `records/2026-10-22-V14-巡航过慢与丢线快速自转.md`。普通巡航目标恢复0.65m/s；起步仅短时0.40，不再因0.45~0.64置信度重置。真正急弯短时0.45；VV虚线触发APPROACH后0.40，环内最高0.30，丢线最高0.12。

搜索角速度在最终输出端硬限制0.35rad/s，单带/弱路径/未确认重获限制0.45rad/s。防自转开关独立于自动线速度规划，普通可信急弯仍保留2.0最大角速度。V13的VV检测、1帧入口确认和环内2/28秒预算没有改变。

配置仍为 `ros2_ws/src/wheel_robot/config/patrol_test.yaml`，保存后约0.5秒校验整套加载。新增参数需先同步整套文件并重启一次。运行仍用下方命令。以下V13及更早说明为历史记录，速度数值以本段和当前配置为准。

## V13历史说明

当前唯一生效配置是 `ros2_ws/src/wheel_robot/config/patrol_test.yaml`。正确绕行为4号进入、5号出去；沿底部主路接近时先经过5号出口岔口，不能从那里反向进入。

本次完整参数表、测试结果、实物风险和运行命令见 `records/2026-10-22-V13-VV单帧入环与巡线限速.md`，以下V11/V10为历史说明，不得按历史的0.65m/s或30/45秒任务释放规则操作当前版本。

- 巡航上限0.40m/s，起步/重新咬线0.20，弯道0.25~0.28，环岛最高0.30，丢线最高0.10。
- 虚线检测移植用户提供的VV小黑块/两侧白色/竖向断开判据，确认1帧；旧入口评分、跨帧虚线链与严格分叉分类退出实际入口运行链。
- 首次标记先沿主路通过至少1秒且0.20m，之后一帧真实近处朝前左弯支线触发4号入口左探。入口路径稳定确认仍是独立保护。
- 环内最短2秒、最长28秒及原出环视觉条件保留。时间诊断不再释放环岛，严格横道视觉提前释放默认关闭。
- 陀螺仪仅在FOLLOW阶段积分净转角；默认360度+0.80m里程计实测行程+最短环内时间后，触发短时直行寻找出口，视觉确认后才完成。

```bash
bash /home/orangepi/Two_patrol/tools/patrol_test_menu.sh
```

先选1视觉干跑再选2实车，站立完成后输入1。复制时必须包含新增 `patrol_dash_entry.py` 及整套YAML，不要只复制旧的两个Python。

## 历史 V11（已被V13替代）

本次以替换后的 V10 为基准，只把 ENTRY 固定左转目标从0.8改为1.2rad/s；线速度目标仍0.65m/s，环内丢线搜索目标仍0.8rad/s。
入口需先连续识别汇聚出口标记5，按可见主虚线前进至少1秒且0.20m，旧标记离开后再连续识别左侧发散入口4。冷却不主动停车。
参数位置：`ros2_ws/src/wheel_robot/config/patrol_test.yaml` 的 `roundabout_entry_turn_rad_s`、`roundabout_first_marker_cooldown_seconds`、`roundabout_first_marker_pass_distance_m`、`roundabout_fork_change_ratio`、`roundabout_fork_confirm_frames`。
完整变更、风险与实车清单见 `records/2026-10-22-V11-入口方向确认与角速度隔离.md`。下面保留 V10 的原有基础功能说明，旧“仅虚线链即可进入”描述以本段为准。

## V10 基础说明

## 本次解决什么，不改变什么

本轮V9日志有两种不同情况：一次已经ENTRY→FOLLOW，随后丢线右找并由30秒兜底释放；另一次没有进入ENTRY，约1Hz可见采样最高入口分数0.74。不能把两次都叫“没有识别”，也不能把场外视频当车载二值图或与ROS时间精确同步。

V10只调整环岛导航。普通巡线、急弯控制器、0.65目标速度、正常姿态0.15m/+25°、横道识别/停车3秒/清线3秒、第二次合法横道后走1m结束、原台阶与底盘/固件保持不变。raw/debug录像仍关闭，不增加编码/视频写盘。真实相机、通信、持续定位故障及人工停止保护保留，不承诺任何异常都不停。

## 四处环岛修改

1. **连续确认入环路线**：原时间0.6秒、行程0.45m、净左转0.35rad、3帧条件保留；还需近/中端真实路径连续稳定0.20秒。单带救线、只有route_locked标志、跳变或陈旧帧不能证明已经抓住环内路线。短时单边允许使用原有新鲜路宽；不要求看到蓝色圆岛。
2. **ENTRY→FOLLOW保留路线身份**：不再清除已选分支和入口已离开视野的记忆。独立环岛几何记忆不被普通0.8秒路径缓存清空；FOLLOW候选先与近期近/中误差比较，没有轮廓也不能让任意右侧黑线接管。沿正确路线连续向右移动/右纠偏仍允许，不是永久强制左转。短时无有效路径时使用环岛左找线方向，不继承普通巡线残留右转；仍受原1.8m/3s边界保护。
3. **低分入口跨帧关联**：只在SEEK/APPROACH对同一组真实纵向虚线链关联。最低0.70、至少5块、纵向跨度22%图高、合理间隔与拟合形状、连续4个新帧且至少0.08秒、最大间隔0.20秒和相邻横向位置限制。通过后提供入口强证据，原始gate分数不篡改；不用于放宽出环/横道。暂停/新场重置不积攒旧候选。实际真假漏检仍需实测，不是任意黑块就左转。
4. **EXIT阶段拐点补线**：先沿原环行进展和返回虚线/左长实线条件确认EXIT，再寻找当前连通左通道在分叉口的内侧右拐点，以及同一通道上方真实左边界点，连接成虚拟右边线；左边界照旧并重新计算中线。连接及中心预引导必须在实际连通白路内，不能穿过已知蓝区，不能拿另一块白区的全局最左点凑线。可靠补线存在时优先跟其道路中线，不被直行黑线覆盖；找不到就沿原验证左通道恢复，不凭空补线。

蓝色可见时用于排除实际不可行区域；看不到蓝岛本身不扣分、不阻止入环，也不自动当作出环。局部视野只剩普通直线时仍有歧义，连续性不能保证绝对定位。V10没有新增依赖蓝色出现的状态门槛。

## 出环触发、引导和完成是三件事

- FOLLOW：仍需最少环内时间2秒/行程0.50m/净左转1.5rad、入口证据离开过，再由返回虚线或连续左长实线触发EXIT；不是见一个拐点立即出环。
- EXIT：只有此阶段尝试上述虚拟右边线，屏幕紫线代表实际采用的补线；普通弯道、ENTRY/FOLLOW不使用出环拐点补线。
- 完成：至少退出0.45m，且分叉/补线消失后重新连续3帧得到近中一致、非救线、无近端缺口的稳定路线，才标为visual_exit。带分叉的旧帧不能预填完成计数。之后按原流程转交横道，不重新触发同一入口。
- 原30/45秒兜底和18秒后严格横道证据提前释放不改；time_fallback/zebra_evidence_fallback不冒充几何出环，不能据此说车已真正出环。

## 覆盖、菜单和实时参数

先人工结束旧程序，备份实车当前可用源码和参数。ZIP顶层仍是 `Two`，把其中的文件合并覆盖到实车实际工程目录，不能多套一层Two/Two；不要只传scripts，也不要保留旧V9 YAML代替新表。

若车上工程叫Two：

```bash
bash /home/orangepi/Two/tools/robot_menu.sh
```

若仍叫Two_patrol，就从该目录内启动同一个tools/robot_menu.sh。菜单按自身位置编译/加载install_patrol，不依赖另一个工程，也不要再运行以前复制到家目录的旧菜单。仍需系统ROS Humble和依赖。

菜单1首次编译/检查本包；2视觉干跑、不启动底盘；3原站立；**4运行V10巡线/环岛/横道测试，不执行台阶**；5保留原含台阶比赛备用，原流程没有自动接入V10；6环境，0退出。完成原上电调零/使能并保持遥控急停可用。子菜单1开始/继续、2人工暂停保留任务、3放回起点重置新场后等待1、0退出。

屏幕保持DISPLAY=:0、XAUTHORITY=/home/orangepi/.Xauthority。菜单/环境助手一起放在本包tools，不复制成外部旧入口。原build/install/log不删除，首次可选1检查自己的独立build_patrol/install_patrol。

只改本工程 `ros2_ws/src/wheel_robot/config/patrol_test.yaml`。新Python和完整新YAML覆盖后需结束旧进程、重启一次；随后保存完整参数表约0.5秒热加载，不需要反复编译或输入ROS调参命令。缺字段/拼错/非法值整套拒绝并报错，不会悄悄替换速度。参数按F1入环确认、F2环内记忆、F3入口关联、F4出环补线分组，逐项有单位、范围和调整风险。改动前先安全停放。

| 重点参数 | 默认值与作用 |
|---|---|
| fixed_speed_m_s | 0.65，两圈相同，没有新增弯道降速档 |
| roundabout_entry_stable_seconds | 0.20，真实近中路径稳定时间，不是额外固定左转脉冲 |
| roundabout_entry_path_delta | 0.18，相邻路径误差允许变化 |
| roundabout_path_max_gap_seconds | 0.20，稳定路径帧最大间隔，陈旧帧不能凑次数 |
| roundabout_route_memory_seconds / roundabout_route_max_delta | 1.20秒 / 0.30，环内目标几何关联；过期不等于任意线可接管 |
| roundabout_gate_tracking_enabled / roundabout_gate_track_min_score | 1 / 0.70，低分同链入口补充；0回原分数门槛 |
| roundabout_gate_track_frames / roundabout_gate_track_max_gap_seconds | 4 / 0.20秒，入口跨帧关联，非未来帧预测 |
| roundabout_gate_track_max_x_ratio | 0.10，相邻拟合链位置差占图宽比例 |
| roundabout_exit_corner_enabled | 1，仅EXIT尝试可验证拐点补线；0回原左通道引导 |
| roundabout_exit_corner_min_span_ratio / roundabout_exit_corner_min_shift_ratio | 0.12 / 0.08，排除过短/过小边界毛刺 |

角速度、转向增益和统一速度未因本轮自动改大，入环仍受真实v/w半径和执行能力影响；没有实车内参、轨迹测量就不能保证某组值最优。

## 日志与离线验证边界

- `version=10`：确认运行的是本包；菜单4不应仍显示version=9。
- `gate`原始分数、`gate_chain`本帧链块数、`gate_tracked=1`跨帧入口确认分开记录。
- `route_owned`、`route_status`：是否持有路线；`waiting_continuous_near_mid_route`等真实近中路径，`following_owned_route`正在跟随，`left_route_recovery`左向恢复。
- `exit_corner=1`且屏幕紫线：本帧实际采用验证后的出环虚拟右边界；0不是程序没运行，可能还没到EXIT或没有可靠拐点。
- `reason`：被拒外侧黑线/道路、出环补线优先、连续性不足等原因；`ring_release`区分视觉完成/时间/横道视觉兜底。
- `target/cmd_v/odom_v/process/age`继续区分目标、输出、底盘反馈与处理频率；不把场外感觉直接等同软件限速。

离线测试包含原无驱动用例、无蓝岛黑线入环、跨阶段记忆、外侧右线/右白区拒绝、真实连续右纠偏、FOLLOW左搜线、新帧关联拒绝随机/陈旧链、EXIT专用补线不跨蓝、分叉消失后重新确认、两圈横道任务与录像关闭。原源代码和冻结参数的字节一致性、ZIP CRC、LF、Python/Bash检查由构建/测试记录给出。

合成图只能验证程序规则，不是本次车载实拍像素，也不是Orange Pi实际FPS或稳定完赛保证；未操控实车、未烧录、未进行Orange Pi ROS2构建。原181文件仅授权替换菜单，另外180个原文件逐字节保留，包含原台阶/恢复/底盘/固件/CMake及原比赛参数。具体测试计数和包摘要见本轮records/UPLOAD_NOTES。

English: V10 adds observed near/mid acquisition, persistent ring-route ownership, task-specific bounded left recovery, temporal association of structurally valid lower-score entry chains, and EXIT-only validated corner/right-edge guidance. Blue-island visibility is never a required entry condition; continuous right corrections remain allowed. Original cruise/corner/zebra/step behavior and recording-off are preserved. Offline validation is not physical validation.
