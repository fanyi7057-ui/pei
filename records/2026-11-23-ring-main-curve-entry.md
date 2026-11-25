# 环岛识别：允许“主虚线链左弯”作为探左证据

数据：`20261123_190209_6047`（741 帧 / 42.4 s）。用户反馈：没有识别到环岛任务；第二圈第一个弯道丢线。

## 环岛为什么没识别

- phase 只有 `SEEK 653 / APPROACH 88`，**没有 ENTRY/FOLLOW**；两次 APPROACH（9.5–12.4 s、13.8–15.9 s）都失败。
- 离线逐帧跑 VV：f164–187 全部 `valid=1 vv_dash_hit`，`main=1`，**`branch=0`（一直没有分叉左支）**；f160/f180/f240 的画面里入环处是一条**向左弯的主虚线链**，不是分叉。
- 旧 ENTRY 条件只认 `entry_branch.valid`，所以永远等不到左支 → APPROACH 到 0.60 m 距离上限释放 → 环岛任务从不触发。

## 修改

`PatrolTuning`/YAML 新增（默认开启）：

| 参数 | 默认 | 作用 |
|---|---:|---|
| ring_entry_main_curve_enabled | 1 | 允许主虚线链连续左弯作为探左证据 |
| ring_entry_main_curve_near | -0.03 | 主链 near 误差上限（负=向左） |
| ring_entry_main_curve_mid | -0.08 | 主链 mid 误差上限 |
| ring_entry_main_curve_frames | 2 | 连续满足帧数 |
| ring_entry_main_curve_min_near_ratio | 0.75 | 虚线链近端至少到画面该比例（太远不算） |

APPROACH 里连续满足上述条件时 `left_ready=True`，后续仍走原有 `advance`、`approach_max_m`、探左 0.5 s/证据延长/失败释放/重试上限，不跳过任何安全约束。`entry_detection_reason` 标记 `vv_main_curve_left_probe`。

## 离线验证（同一段采集重放）

- 第一次 APPROACH 仍按原逻辑释放（9.25→11.35 s，`entry_approach_distance_limit`）；
- **第二次 APPROACH 在 14.39 s 成功进入 ENTRY**（`vv_distance_complete_left_probe`，主链左弯证据），探左 0.5 s 后在 14.95 s `entry_probe_no_route` 释放。
- 即：**环岛任务现在能被识别并触发探左**；剩下的问题是 ENTRY→FOLLOW 的路线确认（探左后没有连续黑线/路线证据），那是下一层问题。

## 第二圈第一个弯道丢线（未改，先记录）

- f660–677：v 0.48→0.30，near 从 -0.44 在约 0.6 s 内翻到 +0.55，w 从 +1.45 翻到 -1.42；f678 `line_low_confidence` 丢线，随后 61 帧右找。
- 这是典型的急 S 换向：车在 0.30 m/s 下已按弯道降速，但换向仍来不及，线在画面里消失；和环岛无关，是急弯控制问题。
- 候选下一步（未实施）：
  1. 换向帧额外限速（如 `reversal_speed_m_s=0.22`，检测 near 符号翻转且上一帧 |w| 较大）；
  2. 给 mid/far 弯向更高权重（提前预判下一个弯），需实车对照。

## 验证

- 新增 2 条单测：主链左弯可触发 ENTRY；虚线链太远（near_ratio 0.40）不触发。
- 完整测试：752 passed / 23 failed / 11 skipped；23 项与修改前历史/合同失败完全相同，无新增功能失败。
