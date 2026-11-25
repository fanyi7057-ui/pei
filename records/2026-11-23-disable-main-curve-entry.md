# 关闭“主链左弯”入环回退（回到只认分叉左支）

数据：`20261123_195347_5922`（383 帧 / 22.0 s，丢线 8.6%）。用户：入环过早且错误进入；问现在环岛逻辑是否还是“1 帧虚线→前进0.12（最高0.6）→左搜索”。

## 现在的环岛逻辑（改前）

仍是“锁存→前进→左探”的结构，但参数和证据有变化：

1. SEEK：一帧有效虚线（`vv_dash_confirm_frames=1`）+ 门控 → APPROACH；有确认左支时可提前重臂。
2. APPROACH：从锁存点前进 `ring_entry_advance_m=0.25`（原 0.12）后才允许进 ENTRY；超过 `ring_entry_approach_max_m=0.60` 释放（`entry_approach_distance_limit`）。
3. 进 ENTRY 的证据：`entry_branch.valid`（分叉左支）；2026-11-23 又加了“主虚线链连续左弯”回退（`ring_entry_main_curve_*`）。
4. ENTRY：左探 `roundabout_entry_turn_rad_s=1.2`，基础 `ring_entry_probe_seconds=0.5`；只有当前帧仍有视觉路线/左支证据才延长到 `ring_entry_probe_max_seconds=0.9` 或 60°；否则 `entry_probe_no_route`/`entry_confirm_timeout` 释放。
5. 失败重试 2 s/0.5 m；成功出环后 12 s/3 m 重臂；连续探左失败达到 `roundabout_max_failed_entries=2` 报 `ring_entry_unconfirmed` 停车。

## 19:53 为什么错误/过早进入

- f159(9.25 s) 进入 APPROACH，车已经在跟一段左弯虚线；f197(11.4 s) 触发 ENTRY，此时主链 near -0.20~-0.47、mid -0.26~-0.51——是**车自己正在走的弯**，被“主链左弯”回退当成入环证据；
- 左探 0.5 s 后 `entry_probe_no_route` 释放，随后找线，f212–224 丢线 13 帧；
- 结论：主链左弯回退在“车已经在弯里”时太容易满足，导致过早/错误进环。

## 修改

`ring_entry_main_curve_enabled: 0`（YAML 关闭，代码保留）。环岛回到**只认分叉左支**的旧行为：

- 离线重放 19:53 采集：APPROACH 在 11.28 s 以 `entry_approach_distance_limit` 释放，**不再误入 ENTRY**；
- 代价：像 19:02 那种“没有分叉、只有主链左弯”的位姿不再进环（宁可不进，不要错进）。

## 验证

- 两份“整表 YAML 等于默认值”的测试同步 `ring_entry_main_curve_enabled=0`；
- 完整测试：756 passed / 23 failed / 11 skipped（23 项为既有历史失败）。
