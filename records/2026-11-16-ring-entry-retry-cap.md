# 环岛入口重试上限：不再无限绕圈

数据：`20261116_195026_30001`（953 帧，38.6 s，丢线仅 7.9%）。runtime `0a2c99…`，含左探后立即左找与 1.10 找线。现象：小车进环岛后一直在里面转圈，每隔约 10 s 再触发一次 APPROACH/ENTRY。

## 数据与离线重放

- CSV phase：SEEK 838 / APPROACH 83 / ENTRY 32；三次 `APPROACH→ENTRY`（13.4、24.7、35.4 s），没有任何 FOLLOW/EXIT。
- ENTRY 每次约 0.3–0.5 s：左探 1.20 rad/s，`ring_yaw` 只积累约 0.5 rad（30°），随后 `entry_probe_no_route`/`entry_tracking_lost` 释放。
- 释放后 SEEK 沿环岛虚线圆连续跟随（`w≈+0.5`，路径有效），约 8.7 s 绕回同一入环点；`entry_block_until` 的 2 s/0.5 m 重试门早已过期，于是再次 APPROACH/ENTRY。
- 用 `replay_state.py` 重放真实 vision+router+motion_override：ENTRY 的 route_status 是 `entry_left_branch_not_confirmed`，第二次尝试出现 `entry_confirming_route`/`waiting_continuous_near_mid_route` 但路径逐帧闪烁（valid/invalid 交替），`entry_visual_tracking` 后 0.25 s 无有效路径就释放；始终不到 FOLLOW 要求的 55° 确认角。
- 帧 420/700 几乎相同（同一位置、同一蓝区、同一虚线块），确认物理上就是绕环。

## 改动

- 新增 `roundabout_max_failed_entries`（默认2，0=不限）：`_release_ring` 的 `entry_probe_no_route`/`entry_confirm_timeout`/`entry_probe_turn_limit`/`entry_tracking_lost` 计入失败；达到上限设置 `fault="ring_entry_unconfirmed"`，节点按既有 route fault 流程停车等人工，不再继续重试。
- `entry_approach_distance_limit`、时间释放等非探针失败不计入；真实 `visual_exit` 清零；`reset_course()` 重建 router 自然清零；人工 `resume(clear_fault=True)` 清零，允许再次尝试。
- 触发上限时不再 arm 左找窗口。
- YAML/文档同步；新增路由测试（两次探针失败触发 fault、approach 失败不计数、人工恢复清零）。

## 验证

- 完整测试：739 passed / 23 failed / 11 skipped；23 项与修改前历史/合同失败完全相同，无新增功能失败。

## 边界

- 这是**失败保护**，不是出口修复：它让车停下并给出明确 fault，而不是无限绕圈。真正的“能出环”仍取决于 ENTRY→FOLLOW→EXIT。
- 离线重放显示入口路径闪烁是当前确认不了的主因；下一步应拿实车终端日志的 `route_status`/`entry_*` 与 `vv_reason`，评估入口确认是否要容忍短丢帧（`roundabout_path_dropout_seconds`/`roundabout_path_confirm_frames`），不能只靠加左转时间。
- 未改固件/差速/比赛栈；上限按每次环岛计，正常多圈跑法只在真实出环后清零。
