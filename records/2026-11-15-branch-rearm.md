# 重找冷却内已确认左支提前重新锁存

基准：2026-11-15 车上的三个采集目录（`20261115_135319_11270`、`20261115_135435_11812`、`20261115_135545_12330`）。三份 `provenance.json` 的源码 SHA256 与当时 `patrol_contour_runtime.py / patrol_contour_test.py / icn.py / patrol_dash_entry.py / patrol_recorder.py` 完全一致，因此这些录像就是当前入口代码的实车行为，不是旧版本素材。

## 现象

三份采集合计 3006 帧；离线逐帧重放当前检测器和路由器，只有后两份出现虚线（c1 全程 0 帧合格虚线）：

| 采集 | APPROACH | 路程上限退回 | 冷却期内出现左支 | 冷却结束再锁存 | 结果 |
|---|---|---|---|---|---|
| 135545_12330 | 第235帧 9.52s | 第282帧 11.58s | 316–322帧（12.9–13.1s） | 337帧 13.71s | 再走0.60m，无左支，SEEK |
| 135435_11812 | 第248帧 9.98s | 第293帧 12.02s | 326–343帧（13.2–13.9s） | 345帧 14.11s | 再走0.60m，无左支，SEEK |
| 135319_11270 | 无 | 无 | 无 | 无 | 与本次入口无关 |

两条车都重复出现同一顺序：先正常触发APPROACH，0.12–0.60m内看不到发散左支，按设计退回；真正能测到左支的只有失败后冷却期那几帧；等2s/0.50m冷却结束、重新锁存时，左支已经不在画面里，于是永远进不了ENTRY，更到不了FOLLOW。

## 原因

`PatrolVision.estimate` 原来只在 `gate_detection_allowed()` 为真时调用虚线检测：

```python
if self.router.gate_detection_allowed(now, tuning) or self.router.phase == "ENTRY":
    self.dash_observation = detect_vv_dashes_compatible(...)
```

失败重找冷却期 `gate_detection_allowed()` 为假，所以冷却期的左支不仅不能锁存，连“测量”都没有发生（`dash_observation` 是空的）。重放中把检测器强制每帧运行，冷却期就能稳定测到 `vv_entry_diverging_left`；这说明瓶颈是入口调度，不是几何门限，也不是0.60m上限本身。

## 改动

- 新增 `RoundaboutRouter.detection_allowed()`：只放宽“测量窗口”。SEEK 满足最初6秒/1米启用门后即持续检测虚线，冷却期也测；它本身不授权任何任务。
- `gate_detection_allowed(branch_confirmed=True)`：当前帧已确认**发散左支**时，只豁免重找冷却（`roundabout_retry_seconds / roundabout_retry_distance_m`），不豁免最初的6秒/1米启用门。
- `_observe_vv_entry` 的SEEK锁存：普通合格虚线仍必须等冷却；带已确认左支的合格虚线可以立即锁存APPROACH，`entry_detection_reason=vv_rearm_on_confirmed_left_branch`。汇聚返回支（`exit_branch`）不算入口，不触发该路径。
- 新开关 `roundabout_entry_branch_rearm_enabled`，默认1；置0完全恢复旧行为。
- 不改：`ring_entry_advance_m=0.12`、`ring_entry_approach_max_m=0.60`、入环1.20rad/s、55°建立角、FOLLOW/EXIT/横道/两圈凭据，以及PathSelector/FixedSpeedController源码。

## 离线验证

用录制的逐帧 `cmd_v/cmd_w` 作为行程、用当前检测器输出作为 `dash_cue`，重放 `RoundaboutRouter`。开关置0时重放出与实车相同的两次APPROACH且无ENTRY；置1时：

| 采集 | 旧：第二次锁存 | 新：左支锁存 | 新：ENTRY | ENTRY时累计路程 |
|---|---|---|---|---|
| 135545_12330 | 337帧 13.71s | 316帧 12.88s | 322帧 13.11s | 5.36m |
| 135435_11812 | 345帧 14.11s | 327帧 13.41s（329帧被判为汇聚支短暂退回）→338帧 13.84s | 349帧 14.24s | 5.86m |
| 135319_11270 | 无 | 无 | 无 | 不变 |

135435_11812 的327→329帧说明同一岔口会被检测器短暂判成汇聚返回支：`exit_only`按原规则退回SEEK，9帧后再次确认发散左支才重新锁存。这个来回不产生重找冷却，也不改变转向，重放中车内位置几乎没动；保留该判定比强行忽略汇聚证据更安全。

重放只证明入口状态机与检测器在真实画面上的判定顺序，不模拟后续转向闭环；ENTRY之后的重放会因为没有真实路径而按时限退出，不能当作FOLLOW成功。

## 测试

- 新增5项：冷却内左支可重新锁存并进入ENTRY；开关置0保持旧冷却；带左支也不能绕过最初6秒/1米启用门；汇聚返回支不触发重新锁存；实际`PatrolVision.estimate`在冷却期仍测量虚线、并对左支锁存。
- 修改后的完整测试与修改前对照：修改前 719 passed / 23 failed / 11 skipped；修改后 724 passed / 23 failed / 11 skipped，失败名称与修改前完全相同（旧adaptive参数/旧源文本合同、历史SHA清单、旧菜单合同、原台阶测试桩），没有新增失败。同步更新 `PATROL_TEST_ADDITIONS_SHA256SUMS.txt` 的6项摘要并新增本记录。

## 风险与实车

1. 冷却期检测会增加少量每帧计算（80x56小图），不影响30Hz控制。
2. 若把远处平行虚线误判为发散左支，会提前左探：看日志 `vv_rearm_on_confirmed_left_branch` 与 `vv_blocks`，必要时把开关置0对照，而不是同时改检测门限和0.60m上限。
3. 仍需实车确认：新锁存后能否连续接线、达到55°建立角并进入FOLLOW；本次没有ROS构建、Orange Pi运行或电机验证。
4. 三份采集的c1没有合格虚线，说明该段路程与入口无关，不能拿它判断入口改善。
