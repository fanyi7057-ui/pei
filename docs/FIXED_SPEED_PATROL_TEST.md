# V27：两弯后切换姿态，启动即自动电脑第一视角

## 使用：不再手动开终端转发或打开网页

MobaXterm SSH会话需启用X11 forwarding，且本机X server运行（通常默认开启）。这是连接的一次性设置，不是每次另开窗口输入命令。若之前没启用：编辑SSH会话勾选X11 forwarding后重新连接；缺少转发时远程程序无法自行在电脑创建窗口。

仍只运行`bash /home/orangepi/Two_patrol/tools/patrol_test_menu.sh`。程序自动打开电脑第一视角窗口，同时保留车载屏幕。菜单和launch在选择车载DISPLAY=:0前保留SSH转发地址；后台viewer通过转发显示，不额外打开摄像头、无需SSH端口隧道/浏览器。窗口是只读，按Q/关闭只退出观看，不停车，急停用原遥控/终端。转发失败会警告但不影响控制；HTTP预览仍作为可选后备，下面V26手动网页步骤不再是默认操作。

## 分阶段姿态

`posture_stage_enabled: 1`：从起步到第二弯完成使用`posture_start_height_m: 0.15`和`posture_start_pitch_deg: 25.0`。确认两次独立大弯、出弯连续稳定跟线后，沿原RacePostureTransition切换至`posture_height_m: 0.25`、`posture_pitch_deg: 28.0`。不修改巡线速度或环岛/横道许可，第二圈保持后段姿态。设stage=0则直接用后段姿态（V26行为）。

借鉴E:/wheel_robot.pei的航向记录，但不搬其0.95rad(54.4度)就计数逻辑。新计数使用新鲜、同一来源的实测航向净变化（支持跨±180），约90度下界`posture_turn_target_deg: 80`，还需前进行程`posture_turn_min_travel_m: 0.12`、低转速且黑线/道路可靠居中`posture_turn_center_error: 0.12`持续`posture_turn_center_frames: 5`个新航向样本。未回正不计数、超过130度拒绝宽转/自旋、单弯超时`posture_turn_timeout_seconds: 10`丢弃；小幅左右摆动不累计绝对转角。运动有odom用现有实测行程，缺odom仅为指令估计；不是绝对证明位移。

仅首圈普通SEEK/APPROACH阶段记录，最多2次；停止、遥控、航向过期/跳变/来源变化丢弃未完成弯，保留已完成数。按2暂停保留姿态阶段，按3重置全部计数并恢复起始姿态，仍须1开始。参数修改不混用旧门槛证据。姿态切换清理视觉几何缓存，不清除环岛/横道完成凭据。

日志：`posture_turns=0/2`、`posture_turn_deg`、`posture_reason`；每个弯完成打印POSTURE_TURN和净角度，第二次打印POSTURE_SWITCH。航向不可靠则保持起始姿态，不用时间假装完成两弯。高度和角度变化会改视野，先干跑、后实车观察转弯计数及切换时是否短暂丢线；未实测X11电脑弹窗和底盘，不能承诺流畅/完赛。

复制时必须包含新增patrol_turn_posture.py、patrol_pc_viewer.py，以及更新的launch/patrol_test.launch.py和两个菜单。按原校验/菜单指令运行，确认V27，源码启动无需重编译C++。以下V26和更早均为历史说明。

# V26：视觉稳定、短掉帧宽限、电脑第一视角、姿态0.25米/28度

基于完整V23，不沿用V24/V25入环距离试验。入口前通过距离仍0.12米；出口寻找仍0.73米/7秒/8秒。本次不调普通0.65、环岛上限0.30、内环/横道任务参数。

## 视觉与流畅度

- 当前帧暗蓝色保护：H在76..140、S至少48、B-R至少8，不设亮度下限；不会仅因低亮度或高饱和就删除黑胶带。无跨帧二值图叠加。
- 普通黑线图仅过滤宽度超过图宽0.18、且连续至少图高12%行都很宽的大块区域；保留短暂近水平急弯。横道/虚线事件图不应用宽度过滤，只排除明确蓝色证据。
- 黑线已有跳变防抖保留，扩展到单边轮廓突然换侧；锁定环内路线不套普通跳变判断。
- 当前可靠路线刚丢失时，最多0.12秒且指令累计行程小于0.08米，保持不高于丢失前速度及已发转向，不盲加速/加转。任一到限进入原0.20低速搜索；弱路线、补救/交接路线、明确环岛找线不获此宽限。时间/距离保护预算不被宽限刷新；相机超时和遥控保护优先。
- `mask_stability_enabled: 0`关闭新增掩模处理；`loss_grace_seconds: 0`关闭宽限。保存后按原实时参数机制应用。新掩模处理异常回退当前基础掩模，日志`mask_guard_fallback=1`；不伪造路径。

## 电脑第一视角（原始摄像头，不含算法叠加）

车载屏幕保留，电脑预览复用唯一采集源；后台JPEG编码上限8fps、宽640、质量70，只绑定小车127.0.0.1:8080。无人观看不编码；网络线程不能发布速度。页面标注LIVE/STALE/DISCONNECTED；仅供观察，不是急停通道。

1. 在MobaXterm启动小车测试程序，看到V26和电脑预览提示。
2. 电脑另开PowerShell，替换下方`小车IP`：`ssh -N -L 8080:127.0.0.1:8080 orangepi@小车IP`。输入密码后保持窗口打开，无输出是正常现象。
3. 电脑浏览器打开 `http://127.0.0.1:8080/`。

也可使用MobaXterm的Tunneling：New SSH tunnel / Local port forwarding，本地8080，远端127.0.0.1:8080，SSH服务器填小车IP、用户orangepi、端口22；启动隧道后访问同一网页。不要同时启动两个占用本地8080的隧道。

小车8080被占用：启动前`export PATROL_PC_PREVIEW_PORT=8081`，电脑隧道对应`ssh -N -L 8080:127.0.0.1:8081 orangepi@小车IP`。关闭电脑预览：启动前`export PATROL_PC_PREVIEW=0`；恢复为1。端口和启用开关需重启程序。端口失败只警告，不中断巡线。

## 姿态与部署

YAML `posture_height_m: 0.25`、`posture_pitch_deg: 28.0`，单位米/度，正俯仰低头，沿原姿态平滑与站立使能流程。0.15升到0.25会显著改变相机地面视野；务必先干跑确认近远ROI、虚线和横道，再低风险实测，不保证完赛。

将E:/Two更新内容复制到/home/orangepi/Two_patrol，执行`sha256sum -c checksums/PATROL_TEST_ADDITIONS_SHA256SUMS.txt && bash tools/patrol_test_menu.sh`；先1干跑后2实车。新增patrol_stability.py和patrol_preview.py必须一起复制；当前源码启动方式无需重编译C++。下文为历史V23记录。

# V23：出口0.73米，丢线搜索限速与可靠反向消除

当前出口目标0.73米、低速直探仍0.12m/s（理论6.08秒）、寻找7秒、获取总期限8秒。普通有效路线目标仍0.65m/s，环岛上限0.30不变。

`maintain_cruise_during_search: 0` 为当前安全设置；`search_speed_m_s: 0.20` 限制无路线和未连续确认恢复期间的实际输出，不等待减速斜坡。连续3帧有效后平滑恢复，不更改普通有效路线的转向公式。旧参数设1会启用旧的巡航搜索行为，不建议实车使用。退化搜索仍0.12，更严格的任务/遥控/相机保护优先。

丢线恢复时，非补救/非交接、置信度达标且近处方向一致的大偏差观测，可立即将反向旧输出归零，再平滑建立新转向；不把噪点直接变为大角速度。日志增加 entry_branch_reason / exit_branch_reason，原vv_reason/fork_gate记录检测拒绝阶段。

本次日志始终SEEK/L1_RING且exit_seek_m=0，因此该日志的右偏不是0.83米出口分支执行导致，缩距只是用户要求，主要修复的是普通丢线搜索速度。下文V22及更早均为历史记录。

# V22：出口寻找目标0.83米，时间配套8秒/9秒

用户实测0.43与0.63不足后明确要求改0.83米。V22只改出口寻找参数、合法范围及相应测试/提示；寻找仍0.12m/s，理论约6.92秒，寻找硬期限8秒、出口获取总期限9秒（阶段上限10秒不变）。默认及YAML同步0.83，距离合法范围(0,1.0]米，修复0.63触发旧0.50上限无法启动的问题。

任何实际可靠出口路线仍立即接管，不要求走满；图像/航向/遥控/距离/期限保护保留，不能仅走够距离就算出环。较长无可靠路线行驶增加冲出赛道风险，实测清场并保留急停；不是所有停车都强制继续走0.83米。普通巡线、入环、FOLLOW预测、人行道速度与参数均不变。

以下内容是V21及更早历史说明，以V22开头说明为准。

出口低速寻找目标0.43米，保持0.12m/s，寻找硬期限4.5秒、出口获取总期限5秒。找到可靠路线立即接管，不强求走满，航向/图像/距离/遥控保护不变。普通0.65、环内上限0.30、人行道0.25米及停车3秒、4进5出入口均不改。

只在现有FOLLOW当前匹配失败时，根据最近两次真实锁定且有近端观察的路线预测搜索参考，最长0.35秒、最大平移0.10图宽。预测本身不是路径，不延长有效观察时间；仍需匹配当前图像黑块，完全无像素不能伪造路线或出环凭据。暂停/重置/退出FOLLOW清空预测。

新日志version=21，ring_predict/ring_match/ring_delta/ring_ref_age/ring_points分别用于观察是否尝试预测、匹配原因、路线偏差、原参考年龄和过滤前黑线点数。配置ring_prediction_seconds/max_shift_ratio。风险与清单见records/2026-10-24-V21-exit043-search-prediction.md。

## 以下V20及更早内容为历史说明，以V21为准

本轮不改普通0.65、环内上限0.30、虚线预引导0.12m/s；不改入口4进5出、人行道0.25米接近及停车3秒。仅gyro候选出环找线目标改0.22米，较旧1秒×0.12m/s名义距离多0.10米；2.3秒硬期限，航向偏离35度/过期保护仍在。优先用候选起点odom的前向投影，不把侧移/倒车算成向前；没odom时只受时间保护，不声称实测走够0.22米。任何情况下不靠该距离授权完成。

FOLLOW原窄窗失败时可按已选曲线形状平移重定位当前白色两侧黑块；范围最多0.22图宽，拒绝单点、形状冲突和近似双候选，不覆盖已有可靠黑线/道路，不参与入环授权。参数ring_curve_reacquire_enabled/max_shift_ratio。

遥控保护重要：已核对本包固件SelectTelecontrol=1，SWA是sbus[4]，原值>=1792对应上位机档；robot_mode不是这个标志。实车需要新鲜/sbus_channels；unknown/manual_switch/stale均不允许自动起跑。遥控介入时未完成环岛作废，已完成圈凭据保留；切回自动档后输入1继续，不能将人工摆位/转动计成自动完成。新日志authority/remote_raw/exit_seek_m和version=20。话题新鲜度不是射频failsafe，也不是底盘执行ROS的证明；实际固件映射不同须先核实，不能盲目降低门槛。

启动方式与原来相同；先干跑再实车。详细参数、风险及测试见records/2026-10-24-V20-exit-distance-current-dashes-remote.md。

## 以下为V19历史说明，以V20说明为准

依据日志1043b9e6：第一次保护后重复赛前站立；第二次实际入环时被从起点计时的time_fallback打断。旧查找计时不再中断ENTRY/FOLLOW/EXIT，也不再让SEEK周期性触发恢复。入环仍无可靠虚线时维持原3秒获取边界；当前锁定虚线持续存在时最多延长到6秒，满足0.12m/s走0.45米所需时间，6秒仍未接管则保护。

定位保护保持零速、原任务和图像检测，不重新站立倒计时、不退出、不自动开动；连续可靠路线出现会提示输入1。人工2停车和3重置仍保留原确认流程。出弯独立CornerExitGuard只在3帧近/中真实路线均居中时清除旧PD误差，不改变基础控制器/速度/急弯参数，不用于环岛阶段。

新参数：ring_entry_progress_max_seconds=6、corner_exit_confirm_frames=3、corner_exit_error=0.12。继续使用本页启动命令；日志version=19、entry_limit可核对。完整测试风险见records/2026-10-23-V19-phase-timers-and-localization-wait.md。

## 以下为V18及V17历史说明，以本页V19说明为准

V18在V17基础上修复日志34da8771的问题。普通速度仍0.65；可信主虚线候选先限速0.30，消失0.6秒释放，候选/APPROACH角速度限制±0.6。候选不授权入环；5号/4号方向判断不变。已入环但尚未取得完整路线证明时，当前锁定虚线可用0.12低速视觉引导，而不是被固定左转覆盖。近端证明统一使用vv_dash_entry_near_ratio，仍需多块真实观察。最终失线停车保留。

参数位于ros2_ws/src/wheel_robot/config/patrol_test.yaml：ring_candidate_hold_seconds、ring_candidate_turn_limit_rad_s；新日志vv_raw/vv_white/vv_column/vv_branch/candidate及version=18。详见records/2026-10-23-V18-candidate-and-current-curve.md。

以下为保留的V17设计说明；“只有主虚线”在V18会先局部减速，但仍不认定5号。

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
