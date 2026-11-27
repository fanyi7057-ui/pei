# 环岛入口：探左保持 + 提交后交给巡线（19:31/19:33 两份采集）

数据：`20261125_193112_6712`（1472 帧 / 109.4s，丢线 3.6%，最长 6 帧）、`20261125_193329_6836`（1253 帧 / 72.2s，丢线 3.9%，最长 14 帧）。runtime `6d0d6f7…`、node `3419a8…`，YAML 事后统一为 `ring_entry_approach_max_m=0.9`。

## 现象（用户原话）

- 第一次（19:31）第二圈“停止行动，要操作台输1”；
- 第二次（19:33）第一圈“看着可以进入但是给加速了导致没有进入”；
- 两次普通巡线都没有丢线（离线统计丢线率 3.6%/3.9%，最长连续 6/14 帧）。

## 数据结论

### 19:31 的停车是“2 次探针失败上限”跨整段累计

- 13.51s ENTRY → 14.08s `entry_probe_no_route`（第 1 次失败，`failed_entries=1`），随后车沿环内线继续走。
- 56.11s `time_fallback`（整圈时限释放），58.13s 再 APPROACH，60.1s ENTRY → 60.76s `entry_confirm_timeout`（第 2 次失败，累计=2）→ `fault=ring_entry_unconfirmed`，节点按既有 route fault 停车等人工。
- CSV 在 60.76s→85.39s 完全没有帧，24.6s 就是用户在操作台按 1 的时间。**不是控制环卡死，也不是相机/里程计故障。**

### 19:33 的“加速没进”是 APPROACH 距离上限 + 释放后回全速

- 47.2s 锁存 APPROACH；48.9s 已走 0.6m（旧 `ring_entry_approach_max_m=0.60`），此位姿左支直到 49.96s（advance≈0.88m）才第一次进画面。
- 49.26s 按 `entry_approach_distance_limit` 退回 SEEK；此时路径有效，节点不再套 `search_recovery_speed_m_s`，49.6s 速度已从 0.30 到 **0.61 m/s**，正好冲过岔口。
- 49.96s 虽然重新锁存（`vv_rearm_on_confirmed_left_branch`），但车已过岔口，51.95s 再次距离上限释放。

### ENTRY 为什么“必然”确认不了

逐帧回放（`replay_state.py`/`debug_entry.py`，重放真实 vision+router+motion_override，能复现 CSV 的相位与 release）：

- 探左 1.2 rad/s 时，发散左支只在最初约 0.2s 可见（f234–f235 黑线=左支 near -0.57/mid -0.90；f236–f237 置信度 0.21/0.22 判 `line_low_confidence`）。
- 车一转过去，黑估计立刻跳回主虚线：f238 near +0.49、f239 +0.72，而 `ring_memory` 记住的是转弯前的左支（near -0.57）。节点 ENTRY 路线门要求“黑线与记忆引导一致”，于是逐帧 `black_outside_locked_route`，`visual` 永远拼不出 3 帧连续；0.5s 到期 → `entry_probe_no_route`。
- 但**车其实已经物理进环**：f250/f280 画面是蓝岛+环岛内圈黑线，之后普通巡线一直沿环内线走；任务层却已经退回 SEEK，所以每圈都会重新把环岛当“入口”再试，最后累计失败停车。

## 本轮修改

1. `ring_entry_approach_max_m: 0.60 → 0.90`（YAML 单变量）：让 APPROACH 有机会等到 19:33 位姿那种 0.88m 才出现的左支。
2. `patrol_contour_runtime.py` `motion_override` ENTRY：
   - **探左保持**：`entry_dash_proven` 且累计转角 < `roundabout_entry_acquire_yaw_deg=55°` 时，当前帧无证据也继续探左（仍受 0.90s/60° 上限）；不再是“证据一消失立即释放”。
   - **提交后交给巡线**：到时限仍无 visual、但已 `entry_dash_proven` 且实际左转 ≥ `roundabout_entry_min_yaw_rad=0.35` 时，不 `_release_ring`，而是 `entry_visual_tracking=True`、`route_status=entry_commit_follow`，由普通巡线接管，交给原有 ENTRY/FOLLOW 合同（转角/行程/稳定路线/3s acquire）确认。
3. `patrol_contour_test.py` ENTRY 路线门：当 `entry_dash_proven` 且 `router.entry_turn_committed()`（实测/命令转角 ≥ 0.35）时，接受“当前稳定、非救线、route_observed”的黑线作为环内新路线（`entry_committed_route` 直接置 `route_locked`）。转弯前（<0.35）仍保持旧的 `black_outside_locked_route` 拒绝，覆盖“车自己偏左”的误入风险。
4. `patrol_contour_test.py` 速度：`entry_search_active`（左探失败后 0.8s 找线窗口）内即使路径有效也把目标速度压到 `search_recovery_speed_m_s=0.30`；解决 19:33 释放后 0.30→0.61 冲过岔口。
5. `_release_ring("time_fallback")` 清零 `failed_entries`：整圈时限释放视为新尝试纪元，避免“上一圈失败 1 次 + 本圈失败 1 次”直接冻结。同一纪元内连续 2 次探针失败仍会 `ring_entry_unconfirmed` 停车，保留防无限绕圈。

## 验证

- 新增单测：
  - `test_entry_probe_holds_after_evidence_drops_once_dash_is_proven`（保持；无 proof 仍立即释放）
  - `test_committed_entry_hands_to_tracking_instead_of_dropping_the_ring`
  - `test_entry_turn_commit_uses_measured_heading_when_fresh`
  - `test_time_fallback_starts_a_new_failed_entry_epoch`
  - `test_v10_entry_accepts_the_new_line_after_the_committed_left_probe`
  - `test_v10_entry_rejects_the_new_line_before_the_committed_turn`
  - `test_real_node_keeps_recovery_speed_through_the_post_probe_search_window`
- 两份 YAML 整表断言同步 `ring_entry_approach_max_m=.9`。
- 完整测试：见本轮运行记录（23 项既有历史/合同失败为基线，无新增功能失败；本文件写就时同步清单）。

## 边界与下一步

- 仍未实车验证。离线重放使用录制命令，无法模拟“探左保持”后真实多转的30°，所以只能验证状态机不崩、门控不被误放松；真实效果要看实车是否出现 `entry_commit_follow` 并进入 FOLLOW。
- 若实车已进环但仍不确认 FOLLOW：下一个单变量是 `roundabout_entry_acquire_max_seconds`（3.0s，提交后总确认预算）或 `roundabout_entry_path_delta`（0.18，虚线跳变会打断3帧连续性），不要同时改。
- 若仍停车等人工：先确认 `failed_entries` 是否同一纪元内两次；输出 `ring_release_source`/`route_status`。
