# 新台阶策略与首帧环岛接管 / Visual step strategy and immediate ring takeover

本版入口与菜单保持原样。日常仍只编辑小车上的
`/home/orangepi/robot/ros2_ws/src/wheel_robot/config/race_tuning.yaml`，保存后约 0.5 秒加载。

## 开关与调试

- `step_enabled: 0`：忽略台阶，不后退、不冲刺、不跳跃。默认保留为 0。
- `step_enabled: 1`、`step_visual_jump_enabled: 1`：使用本版新台阶策略。
- `step_visual_jump_enabled: 0`：保留原深度/固定距离策略，原来的回车保护在 `icn.yaml`。
- `step_reverse_align_enabled: 0`：摆正时不后退，只调整方向。
- `step_heading_hold_enabled: 0`：冲刺与落地找线的角速度固定为 0。
- `step_jump_on_loss_enabled: 0`：后退对线和冲刺仍运行，但丢线条件满足时停车，不发送跳跃 true。

新策略自动起跳，不使用旧策略的 `manual_confirm_before_jump`。需要保护时先把
`step_jump_on_loss_enabled` 设为 0。台阶总开关关闭、新旧策略切换和正常姿态变化仍须在台阶未接管时进行。
其余新策略参数均可保存热加载；计时不因文件保存而重新开始。

先架空确认话题、状态和急停，再在可控场地低速验证方向；架空里程计不能证明实际行驶距离或跳跃成功。
这里只实现软件行为，没有进行橙派编译、实车运动或跳跃验证。

## 状态流程

```text
深度确认台阶 + 原有赛段允许 + 新鲜 odom
  → VISUAL_ALIGN：后退对线，检查近/中场中心和方向
  → 连续居中，或最多 3 秒后结束对线
  → 有黑线：记录此刻 odom 偏航；无黑线：STOPPED
  → VISUAL_SPRINT：平滑加速、保持记录偏航
  → 最短时间/距离均满足，且不同有效画面连续丢线
  → JUMP：发送跳跃 profile 与一次 true 脉冲
  → VISUAL_REACQUIRE：false，恢复正常姿态，保持原偏航前行
  → 保护时间结束，连续看见黑线：本圈台阶完成，普通巡线恢复
```

“3 秒强制退出”结束的是后退摆正：若仍看得见黑线但未完全居中，按当前偏航继续冲刺并明确记录超时原因；
若完全没有黑线，停车。它不能保证超时后的方向已正确。

新策略使用真正画面中心，不沿用普通弯道目标偏移。中心误差为近/中场归一化误差的均值，
方向误差为近/中场误差之差。后退时横向位置纠偏的符号随行驶方向反转；其方向需实车验证。

冲刺和找线阶段的偏航来自同一个 `/odom` 坐标系；`atan2(sin(error), cos(error))` 处理 ±180° 跨界。
这是反馈纠偏，不是把角速度一直设成某个固定值，也不能排除轮胎打滑或里程计漂移。

相机断帧不会被当成黑线消失；重复画面也不增加确认次数。冲刺/找线超过所设时间或距离就停车，
不会因超时直接跳跃。跳跃脉冲已经开始时，相机短暂缺失不会提前截断脉冲；到时 false 并停车。
里程计失效会立即停车并撤销跳跃 true。新策略成功后不会追加旧版固定向右找线动作。

## 实时参数分组

H1 是后退速度、3 秒退出、中心/方向容差和确认帧；H2 是冲刺速度、加速度、时间/距离窗口和偏航控制；
H3 是起跳开关、跳跃腿高/临时姿态/脉冲时长；H4 是落地保护、找线速度与退出条件。
各字段的单位、增减效果和有效范围在 YAML 中逐项注释。

## 环岛 A 方案

原有比赛阶段和起步里程门仍有效。首个 `roundabout_gate=True` 且分数至少 `0.82` 的画面，
立即进入 `ROUNDABOUT_APPROACH`。弱证据不会通过积分触发进环；日志积分只作观测。
接管后取消残留的右转命令，左转仍按原平滑器增加，前进速度仍平滑减速。

保留用户提供的普通巡线和环岛参数：普通 `0.45 m/s`、最低 `0.30 m/s`、普通预转关闭，
环岛接近 `0.20 m/s / +0.20 rad/s / 1.20 m`，固定进环 `0.60 m/s / +0.34 rad/s`。
这些数值是真实目标值，但不能据此宣称实车必定进环；固定进环的速度/转向比仍决定半径。

## Transfer and verification

Transfer the updated `robot` folder, preserving any robot-side logs and locally tuned YAML before replacement.
Compile once using menu 1 after replacing the package. Menu 2 provides vision-only checks; subsequent tuning saves
need no rebuild. The runtime helper is already installed by CMake, and the new behavior tests are registered.
Local tests exercise distinct-frame confirmation, timeout exits, heading wraparound, jump opt-out, sensor loss,
landing recovery, file reload, and immediate ring takeover. They do not establish physical success.
