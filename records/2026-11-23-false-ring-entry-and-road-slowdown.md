# 修正主链入环误触发 + 低置信度道路兜底降速

数据：`20261123_191523_7977`（830 帧 / 46.9 s，丢线 38.9%）。用户：环岛任务**错误进入**（随后遥控回正）、S 弯容易丢线。

## 本轮之前改了什么（用户问的）

上一次只改了 `patrol_contour_runtime.py` 的入环 APPROACH 逻辑 + 参数，没有改节点：

- 新增 `ring_entry_main_curve_enabled=1`、`ring_entry_main_curve_near=-0.03`、`ring_entry_main_curve_mid=-0.08`、`ring_entry_main_curve_frames=2`、`ring_entry_main_curve_min_near_ratio=0.75`；
- 位置：`PatrolTuning`（`patrol_contour_runtime.py` 参数区）+ YAML + `RoundaboutRouter._observe_vv_entry()` 的 APPROACH 分支；
- 作用：没有分叉左支、只有向左弯的主虚线链时，也允许进入 ENTRY 探左（19:02 采集验证有效）。

## 误入环岛的原因与修正

19:15 采集 f209–216 发生 ENTRY（t=11.8 s），但当时 APPROACH 段车已经在剧烈摆动（f172–208，near ±0.8、w ±1.9），f208–209 的主链误差是 **near -0.55 / mid -0.64**——这是车自己偏到左边，不是入环线，主链左弯条件被误满足，于是探左 0.5 s。

修正：只允许**接近居中、只是略向左弯**的主链作为证据：

- 新增 `ring_entry_main_curve_max_near=0.25`、`ring_entry_main_curve_max_mid=0.30`（`|near|`/`|mid|` 上限）；
- `_observe_vv_entry` 条件加 `abs(near)<=max_near and abs(mid)<=max_mid`。

离线重放：
- 19:15 采集：APPROACH 在 11.78 s 以 `entry_approach_distance_limit` 释放，**不再误入 ENTRY**；第二次 APPROACH 同样距离上限释放；
- 19:02 采集：真实主链入环仍在 14.39 s 进入 ENTRY（保留）。

## S 弯丢线的原因与修正

19:15 采集 f480–490（t≈27.7 s）丢线前：黑线已丢，走的是**道路兜底**（`road_dual_edge` / `road_single_edge_width`），`conf=0.23~0.34`，但速度仍是 **0.65 m/s**（`fixed_speed`），到 f490 道路路径也无效，进入 61 帧右找。

修正：道路/轮廓兜底置信度低时降速：

- 新增 `road_low_confidence_cap=0.45`（0=关闭）；
- 节点 `_run_control_cycle`：SEEK 中若选中路径 `source=="road"`、`path.valid`、`!rescue` 且 `confidence<0.45`，目标速度降到 `search_recovery_speed_m_s=0.30`；
- 只降速，不改路径/增益/任务。

## 验证

- 新增 3 条单测：主链过于偏左（near -0.55）不触发 ENTRY；低置信度道路兜底降速；既有主链入环单测保留。
- 完整测试：754 passed / 23 failed / 11 skipped（23 项为既有历史失败）。
- 仍未解决：环岛识别后 ENTRY 探左 0.5 s 仍会 `entry_probe_no_route`（环内路线确认），属于下一层；S 弯的问题这次只降低兜底速度，黑线主跟踪的急弯换向仍需实车验证。
