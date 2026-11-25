# 真实出环后的 3 秒航向保护（只压大左转）

## 背景

用户反馈“出环岛口时返回了”。逐条对照代码后确认：

- 出口逻辑（`ExitRouteCommit`、`exit_corner_guide`、`yaw_exit_recovery`、`roundabout_exit_*`）只在 `ENTRY→FOLLOW→EXIT` 之后运行；17:25 的采集只有 `SEEK/APPROACH/ENTRY`，没有 FOLLOW/EXIT，所以这些出口代码一次都没执行。
- 历史记录 V10/V11 已描述过同一现象：出口左支会再次吸引视觉，出环后普通巡线输出较大左转，车体物理卷回环岛。旧版用 330°航向锁 + 出环后 1.5s 接管/限幅处理；V16 只保留 `yaw_exit_recovery`（0.8s，且仅路径无效时）和 `ExitRouteCommit`（只锁路线，不限制朝向）。

## 改动

- 新增 `roundabout_post_exit_guard_enabled`（默认1）、`roundabout_post_exit_guard_seconds`（默认3.0）、`roundabout_post_exit_left_limit_rad_s`（默认0.20）。
- `RoundaboutRouter._release_ring` 只有 `source=="visual_exit"` 才设置 `post_exit_until`；失败入环、时间释放、未确认出口都清空，不会启动。
- `post_exit_guard_active` 只在 `SEEK_CROSSWALK` 且未过期时为真；节点在普通巡线输出后，若保护期内 `angular > 0.20`，把它限到 0.20，并同步控制器的 `last_command`，右转（负角速度）不受限。
- 暂停时 `post_exit_until` 随暂停时长顺延；开关置0完全关闭。
- 不改固定速度、路径选择、FOLLOW/EXIT 触发条件、横道/终点凭据；不新增路径源。

## 边界

- 这段保护**只对真实出环有效**。17:25 采集没有 FOLLOW/EXIT，所以本次改动不会改变那次的“出口返回”；它不能替代“入口→FOLLOW”的修复。
- 如果真实出口本身需要大于 0.20 rad/s 的左弯，保护期会压制这一小段左转；可提高 `roundabout_post_exit_left_limit_rad_s`（例如 0.30–0.40）或置0关闭，先实车对照。
- 保护只限角速度，不强制直行；右修、速度、定位保护和急停都不变。

## 验证

- 新增路由测试：真实 `visual_exit` 启动 3s 保护、到期失效、开关关闭、失败入环不启动；新参数非法整表拒绝。
- 新增真实节点测试：保护期内路径要求大左转时，发布角速度被限到 0.20；右转不受限（由控制器原逻辑保证）。
- 完整测试：734 passed / 23 failed / 11 skipped；23 项与修改前历史/合同失败完全相同，无新增功能失败。

## 实车

1. 先确认能稳定进入 FOLLOW/EXIT，再看保护是否生效；日志 `route=SEEK_CROSSWALK`、`exit_commit`、`yaw_exit_*` 可对照。
2. 若出环后仍被左支带回，先看 `angular` 是否被限到 0.20；如果 0.20 太小/太大，只改这一项，不同时改 `exit_commit` 和 `roundabout_exit_left_max_ratio`。
3. 需要保留右修，不要把左限幅写成 0；本改动没有改底盘差速或固件。
