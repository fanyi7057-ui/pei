# 自适应巡线 / 左分支两次环岛 / 分叉形出口 / 远距人行横道

当前正式启动文件 `icn.launch.py` 启动 `scripts/icn_adaptive.py`。原 `scripts/icn.py` 保留不动，作为固定入口逻辑的回退版本。

## 1. 普通丢线：按最近帧趋势左右找

有效黑线存在时，每一张新画面记录近场、中场、远场位置，默认权重为 `0.55 / 0.30 / 0.15`。黑线消失后根据最近 5 帧的位置和移动趋势外推：线向左移动后消失就继续左找，向右移动后消失就继续右找；没有明显趋势时保持接近当前方向。普通赛道不再无条件左找。

## 2. 环岛入口：真正计数的是“左分支”，中央间断主线允许持续

用户现场观察的结构是：

1. 接近环岛时，正常连续黑线逐渐变成中央连续间断块，同时左侧第一次出现分支间断块。物理上这是先经过的环岛出口/前置标志；代码只记为 marker #1，继续普通逐帧巡线。
2. 之后中央间断主线可以一直存在，不要求它清除。状态机只等待左侧分支离开画面并满足 `roundabout_marker_clear_frames`。
3. 左侧分支第二次出现，同时中央间断主线仍成立，才记 marker #2 并进入 `ROUNDABOUT_FAST_ENTRY`。

这避免“中央虚线一直存在导致第二次永远不成立”，也避免中央链抖动一下就把同一处标志重复计数。当前实车日志显示本圈局部里程长期为 0.00m，因此环岛第二标志不再依赖 /odom 里程门；只保留左分支清除帧数 + 最小时间间隔，避免同一分支闪断重复计数。

HDMI 调试行：

`RING C=0/1 L=0/1 mark=0/2 c=<中央链块数> l=<左链块数>`

其中 C 表示中央间断链，L 表示与中央处在同一纵向场景的左分支，mark 是已经确认的左分支次数。

## 3. 第二次左分支后：不是固定打左，而是逐帧趋势 + 左侧优先

进入 `ROUNDABOUT_FAST_ENTRY` 后，不再调用旧版固定 `roundabout_fast_entry_left_turn_speed`。每一帧重新计算 near/mid/far 加权位置和帧间趋势：

- 趋势向左：增加物理左转；
- 趋势向右：允许趋势把左偏减小；
- 但入环阶段保持 `roundabout_entry_min_left_turn_speed` 的最小左转，防止入口外侧旧线把车直接拉回右侧；
- 转向总量被 `roundabout_entry_max_turn_speed` 限制。

默认入环速度 `0.24m/s`、左偏基线 `0.20rad/s`。左分支离开后，连续 3 帧重新看到稳定内线才切入 `ROUNDABOUT_FOLLOW`。因此“固定左打”只保留在原 `icn.py` 回退路径，不属于当前自适应入口。

## 4. 真正出环：不是左右交替，而是“横向主链 + 斜向分支”

用户补充的实物图把出口结构说明得很清楚：上方是一排近似横向连续的间断黑块，同时从其中一处向车辆方向/画面下方延伸出另一条斜向间断黑块链。它是明显的**分叉形**，不是左右交替的锯齿形。

因此出口 detector 已改为两条链的几何关系：

- 主链至少 3 个块，并且横向跨度足够大、纵向漂移较小；
- 分支至少 3 个块，并且向画面下方延伸足够距离；
- 对两组连通块中心分别拟合直线；
- 两条线夹角至少 20°；
- 两条拟合线的交点必须靠近两段可见黑块，允许连接处本身没有贴黑块而留有一小段空白。

用户照片中约 4 个上方主链块 + 3 个斜向分支块，属于这类结构。普通单列虚线和两排平行黑块都不会满足“横向主链 + 斜向分支 + 交点”条件。

HDMI 调试行改为：

`RING EXIT F=0/1 main=<主链块数> branch=<分支块数> ang=<夹角> drop=<分支纵向跨度>`

入环至少 `roundabout_exit_arm_seconds` 后，分叉出口连续确认 2 个新画面，不再立刻恢复普通巡线。实车日志证明立刻切回 `SEEK_CROSSWALK` 会在分叉处被右侧黑线吸走，因此现在先进入 `ROUNDABOUT_EXIT` 左侧提交段，默认持续 `roundabout_exit_commit_seconds=1.20s`。提交期间正角速度代表左转，`race_tuning.yaml` 的 `roundabout_exit_min_turn_speed=+0.10` 作为最小左转，adaptive 层不会允许负值右转；提交结束后才切回 `SEEK_CROSSWALK` 并开启默认 15 秒环岛冷却。

如果 28 秒仍完全看不到分叉出口，也进入同一个 `ROUNDABOUT_EXIT` 左侧提交段作为安全兜底。

## 5. 当前建议默认参数

入口识别目前比上一版稍宽松：中央 ROI `0.32~0.68`、左分支 ROI `0.03~0.40`、纵向跨度 `0.08/0.04`、两次左分支最小时间间隔 `0.35s`，里程间隔关闭为 `0.0m`（因为当前实车 /odom 局部里程没有可靠增长），每圈前 6 秒不统计环岛。

这些只是下一轮实车起点，不是已经验证的最优值。优先观察 HDMI 的 C/L/mark 与 EXIT X 诊断，再一次只改 1~2 项。

## 6. 人行横道

原近场检测继续保留，同时增加 `12%~60%` 的远距 ROI。出环后默认 0.8 秒开始识别，远距确认后继续循迹 0.18m 再停车。

## 实时调参文件

`/home/orangepi/robot/ros2_ws/src/wheel_robot/config/adaptive_tuning.yaml`

程序每 0.5 秒读取一次，保存即可生效，不需要重新编译或重启。代码文件本身修改后仍需要在 Orange Pi 重新 `colcon build --symlink-install`（若当前部署直接用源码 symlink，也应重新 source 并确认实际运行文件）。

第一次验证先关闭运动，只看识别：

```bash
cd /home/orangepi/robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motor_temperature_pkg wheeled_legged_pkg wheel_robot
source install/setup.bash

ros2 launch wheel_robot icn.launch.py \
  enable_motion:=false \
  start_base:=false \
  start_step_support:=true
```

先确认顺序确实是 marker #1 → 左分支清除 → marker #2 → ENTRY → FOLLOW → FORK EXIT，再在遥控急停可用、现场有人值守的条件下做低速实车测试。


## 2026-10-02 实车回归修复：恢复已验证入口阈值

一次后续完整 ZIP 从 GitHub 重新打包时，意外把此前实车已验证的入口阈值退回到更严格值：中央跨度 0.08、左分支跨度 0.04、ROI 0.32~0.68 / 0.03~0.40、纵向错位 0.25。对应实车日志中 marker #1 在约 4.51m 出现，但 marker #2 延迟到约 13.55m 才成立。

现恢复此前已验证参数：中央 ROI 0.30~0.70、左分支 ROI 0.02~0.46、中央最少 2 块、中央跨度 0.07、左分支跨度 0.03、纵向错位 0.32；保留 1.00s 两事件滞后、0.0m 里程门和每圈起始 0.0s 屏蔽。运行时默认值与 YAML 同步，避免文件缺失时再次退回旧阈值。


## 2026-10-02 · good1 后续：环岛平滑 + 加强咬线 + 横道稍后触发

本次在 `good1` 稳定分支之外继续修改 `main`，`good1` 不移动。

为减少状态切换时的顿挫：
- `roundabout_entry_forward_speed` 从 0.24 提到 0.30m/s；
- `roundabout_entry_min_search_seconds` 从 0.25 提到 0.40s，避免刚进环就立刻交控制权；
- `roundabout_blind_recovery_speed` 从 0.08 提到 0.12m/s，减少环内短暂丢线时明显“刹一下”；
- `roundabout_exit_speed` 从 0.15 提到 0.20m/s；出口左侧提交时长仍保持 1.20s，不破坏已验证的左支选择。

为增强环内“咬线”，新增实时参数 `roundabout_follow_steering_gain=1.15`。FOLLOW 和 EXIT 有线时对原逐帧转向做 1.15 倍温和放大，并继续受 `line_recovery_max_turn_speed` / 出口最大转向限制保护。

人行横道保持原 detector 结构和 2 帧确认，不降低可靠性；只把远距 ROI 从 0.12~0.60 下移到 0.18~0.66，并把检测后继续循迹距离从 0.18m 增到 0.22m，使确认和急停位置整体稍后。


## 2026-10-02 · 第一圈软启动、入环提交加强、EXITDBG

本轮继续只修改 `main`，`good1` 保持固定。

- 第一圈起跑新增实时参数：`drive_start_soft_enabled=1`、`drive_start_soft_seconds=1.50`、`drive_start_soft_speed=0.30`。只在第一圈比赛刚开始生效，1.5 秒后继续由原 `drive_linear_accel` 平滑升到 `drive_forward_speed=0.50`；第二圈不套这个限制。
- adaptive 入环最短保持从 0.40s 增到 0.75s，内线重获从 3 帧增到 5 帧，避免车身尚未真正进入环岛就提前切到 FOLLOW。
- race_tuning 中对应旧入口参数同步为 0.75s / 5 帧，使终端显示与 adaptive 实际逻辑一致。
- ROUNDABOUT_FOLLOW 每秒新增 `EXITDBG`，打印 `armed / F / main / branch / mainspan / flat / angle / drop / junction / age` 及阈值。fork detector 即使没有最终通过，也会保留最接近的一组几何数据用于诊断。
- 本轮暂不盲目放宽出口识别阈值，也不缩短 28s 超时；下一次直接根据 EXITDBG 判断出口失败原因。


## 2026-10-02 · 快速入环：更大左转、更短保持

基于实车反馈，FAST_ENTRY 调整为“更有力、但更短”的切入方式：

- `roundabout_entry_min_search_seconds: 0.75 -> 0.60`
- `roundabout_entry_max_search_seconds: 1.50 -> 1.10`
- `roundabout_entry_min_left_turn_speed: 0.55 -> 0.60`

目标是让车身在第二次 marker 后更快地切进圆环，同时更早结束强制左偏，避免已经进入环内后仍持续较大左转。其余保持不变：`entry_forward_speed=0.30`、`entry_max_turn_speed=0.70`、`reacquire_frames=5`、`follow_steering_gain=1.00`。

IMU yaw 出环逻辑本轮不改：仍以真正入口 yaw 为基准，`270°` 只是“允许开始检查回到入口方向”的解锁门槛，不是目标出口角；yaw 仍只参与状态判断，不参与 steering。

## 2026-10-03：环岛误判抑制 + 持续丢线恢复

本版针对实车日志中“第二个普通转弯被当成环岛入口”和“普通巡线丢线后速度/转向归零”的两个问题做状态机级修复。

### 环岛 marker

- 中央虚线链与左分支链必须使用互不重叠的连通块，并满足最小横向间隔 `roundabout_dual_min_horizontal_gap_ratio`。
- 单个 marker 需要连续 `roundabout_marker_confirm_frames` 帧确认，短暂纹理闪烁不再直接计数。
- marker #1 清除需要 `roundabout_marker_clear_frames` 帧。
- marker #1 与 marker #2 之间必须至少出现 `roundabout_marker_between_center_frames` 帧“中央虚线存在、左分支不存在”的走廊证据；没有这段证据的普通后续转弯不会升级为 marker #2。
- `RINGDBG` 增加 `gap / ind / corr`，分别显示中心-左支横向间距、两链是否独立、中间走廊累计帧数。

### 普通巡线丢线

- `line_recovery_timeout_seconds` 不再代表“到点停车”，只表示从趋势预测阶段切换到持续搜索阶段。
- `line_recovery_bridge_seconds` 内按短时闪断处理，尽量保持最近可信速度和方向。
- 之后继续根据最近 5 帧加权位置和帧间趋势找线。
- 超过预测时长后，如果 `line_recovery_persistent_enabled=true`，车辆降到 `line_recovery_persistent_speed`，沿最后可信方向持续搜索，转向从 `line_recovery_persistent_turn_start` 逐渐增加到 `line_recovery_persistent_turn_max`，不再因为普通视觉丢线直接输出 `(0, 0)`。
- 如果跨状态切换导致趋势历史被主动清空，但比赛中此前已经见过赛道线，则以低速直行/最后可信方向继续寻找，不在原地发愣。
- marker #1 严格保持段、marker #1 后保护段、出环后保护段的丢线，也统一复用该恢复逻辑并额外限速/限转。

所有新增阈值均位于 `config/adaptive_tuning.yaml`，继续支持 0.5 s 热加载。

## 2026-10-03 V5: real-exit -> 2s straight -> direct left entry

V5 removes the second-marker/reappearance entrance gate. The physical sequence is now:

1. the first 10 s after race motion start do not evaluate roundabout entry;
2. confirm a real exit-like shape for consecutive frames using both center-dash and left-branch raw component count/span;
3. immediately command a hard straight segment (`w=0`) for 2.0 s;
4. on the first control frame after the 2.0 s timer expires, switch directly to `ROUNDABOUT_FAST_ENTRY` and use the existing strong-left entry controller;
5. no left-branch clear, center-corridor accumulation, or new-left-branch reappearance is required to start entry;
6. gyro-z integration remains state-only for ring exit, and the 25 s post-exit roundabout cooldown remains active.

The V4 false trigger seen at race age 10.0 s (`c=1`, `l=1`) is rejected because it does not meet the V5 real-exit component/span thresholds.

## 2026-10-03 V9：出环左向恢复与第二环岛横道冷却

- 出环提交结束后，恢复保护窗口内不再允许右转。看见主线也至少保持小幅左弧，防止刚离开环岛口就被右侧分支/瞬时视觉拉回。
- 第一次人行横道确认进入 `CROSSWALK_APPROACH` 的时刻开始计时；默认 15 秒后才允许第二圈环岛识别重新武装。冷却期间仍正常逐帧巡线，只清空环岛 marker 累计。
- `RINGDBG cw2=<剩余>/<配置>` 用于实车确认第二环岛冷却是否仍在生效。

## 2026-10-03 V11：220°提速左找 + 330°出口航向锁

基于 V10 实车日志继续收敛环岛出口行为：

- `turn=215°` 时仍有正常主线，随后约 `234°` 开始持续 `line_too_few_bands`；V10 此时用 `0.18m/s,+0.30rad/s` 左找。V11 将 `roundabout_exit_loss_speed` 提到 `0.24m/s`，220°~330°丢线仍固定向左找出口，但减少“慢慢蹭过去”的时间。
- 日志中 `turn=324°` 重新看到线时普通视觉曾给出约 `+0.52rad/s`，`348°` 附近仍有 `+0.34rad/s`，说明出口左支会重新把视觉重心拉向左。V11 从 `peak>=330°` 开始武装“出口航向锁”，目标不是 `w=0`，而是把从 FAST_ENTRY 起点积分的 gyro 角继续收敛到 `360°`：默认 `kp=0.010`，所以 330°约 +0.30，345°约 +0.15，350°约 +0.10，进入 ±8° 后不再额外加 gyro 转向。
- 330°以后正常视觉仍参与控制，但在尚未回到目标方向前仅保留 35% 权重；如果视觉突然要求超过 `+0.18rad/s` 的大左转，认为可能再次抓到环岛左支，启动一次 1.0s 航向保持。该 1 秒不是固定 `w=0`，而是继续按 gyro 朝 360°收敛。
- 正式 `ROUNDABOUT_EXIT -> SEEK_CROSSWALK` 后继续保留 3.0s gyro 航向参考。普通直线/右修仍正常执行；若刚出环又出现可疑大左转，则再允许一次 1.0s gyro 航向接管。1 秒后重新判断；若仍偏左，后续只把左转限制到阈值附近，不接受日志里 `+0.54/+0.61` 这类会物理卷回环岛的大左转。
- 25s 环岛识别冷却、本圈完成锁、第一次人行横道确认后 15s 的第二环岛延迟均保持不变。它们负责“软件上禁止同圈二次入环”；V11 航向锁负责“物理上不要被出口左支重新吸回去”。
