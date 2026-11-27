# 入环位姿分类：已经在左弯里就不再补一次盲左探（entry_follow_only）

数据：`20261125_201321_8221`（1620帧/93.3s，丢线3.9%）、`20261125_201517_8343`（429帧/24.6s，丢线41.3%）、`20261125_201623_8444`（1750帧/99.4s，丢线16.2%）。runtime `4edbc7c…`/node `10ec069…`，YAML `ring_entry_approach_max_m=0.9`（上一轮改动）。

## 用户反馈

“第一次可以两次都进入；但是之后几次偏转进去的角度不对，而且也会摇头。”

## 数据核对

- 三次采集**都没有进入 FOLLOW**，ENTRY 全部是盲探→释放；上一轮的提交/交接改动确实在跑（日志出现 `waiting_continuous_near_mid_route`、`entry_tracking_lost`、`ring_aborts` 递增）。
- 20:16 第二次 ENTRY（f1045，t≈59.7s）：ENTRY 前 5 帧车**已经在沿环岛左弯打满左舵**：f1040–f1044 `w=+1.10/+1.50/+1.46/+1.49/+1.00`，near/mid 约 −0.7~−0.9。ENTRY 再叠加 1.2 rad/s 左探 0.73s（f1045–1066），实测累计转角到 +52°~+80°；视频 f1058 已经是“横看环内、蓝岛在左下”的过度入环位姿。释放后又是 40 帧丢线 + 左找 1.1 rad/s，继续把转角推大——这就是“角度不对”。
- 20:13 的 ENTRY（f1054，t≈61.2s）：ENTRY 前车正在向右修正（f1050–1053 `w=−0.45~−1.03`，e=+0.37~+0.58），探左后 f1070 第一帧就拿到线（e=−0.45），进入顺滑。**直道/居中位姿的探左本身可以工作。**
- 20:15 的 ENTRY（f287，t≈16.6s）：ENTRY 前 near/mid≈0、w≈+0.1（居中直道），探左 20 帧后释放、左找 20 帧才重新拿到线（e=−0.93），转角约 80°。属于“居中位姿 + 盲探”，本轮不动。

## 判断

入环触发时**车的转向状态**分两类：一类还在直道/修正（探左正确），一类已经跟着环岛左弯打满左舵（再补盲探=重复加转角 → 过度入环 + 后续摇头）。把这两类分开，而不是再调探左速率/时长。

## 本轮修改（只改位姿分类）

- `RoundaboutRouter._observe_vv_entry`：ENTRY 触发时读上一帧实际指令 `command[1]`；若 `w >= 0.75*roundabout_entry_turn_rad_s`（≈0.9 rad/s）→ `entry_follow_only=True`、`entry_dash_proven=True`、`entry_reason=vv_follow_existing_left_curve`。
- `motion_override` ENTRY：`entry_follow_only` 时**不叠加任何盲转**，返回 None 交给普通巡线沿现有左弯走（`route_status=entry_follow_existing_left_curve`）；仍受 `roundabout_entry_acquire_max_seconds=3s` 约束，未确认就 `entry_no_route_abort` 释放（不计入探针失败）。
- `patrol_contour_test.py` ENTRY 路线门：`entry_follow_only` 时即使还没有“已提交转角”也接受当前稳定、非救线、`route_observed` 的黑线作为环内路线（否则没有路线、巡线又会去盲找）。转弯前的 `black_outside_locked_route` 拒绝逻辑不变。
- 不新增 YAML 字段；阈值固定为 `0.75*roundabout_entry_turn_rad_s`，文档里注明若这种位姿转不够应调高该比值，而不是改探左参数。

## 验证

- 新增单测：
  - `test_entry_follows_an_existing_left_curve_without_a_second_blind_ramp`（w=1.2 → follow_only、override=None、3s 超时释放）
  - `test_entry_still_probes_when_approach_was_not_already_turning_left`（w=0.1 → 仍探左）
  - `test_v10_entry_follow_existing_curve_accepts_the_tracked_line`（follow_only 时节点门接受新线）
- 完整测试：见本轮运行记录（23 项既有历史/合同失败为基线）。

## 边界与下一步

- 未实车验证。20:15 那种“居中直道 + 盲探后转角到 80°”的进入本轮未改；若实车仍摇头，下一次单变量是 `roundabout_entry_turn_rad_s 1.2→0.9`（更慢的左探让线更早进画面）或 `roundabout_entry_search_seconds 0.8→0.4`（少一段释放后的盲左找），不要和位姿分类同时改。
- 三次采集都未进 FOLLOW，说明 ENTRY→FOLLOW 的确认仍是下一个主战场；本轮只解决可见的“入环角度/摇头”。
- 若 `entry_follow_only` 位姿出现“转不够、进不去”，先调高 `0.75` 这个比值，不要动探左参数。

## 追加：2026-11-25 20:38 采集 + 第三人称视频

数据：`20261125_203818_9696`（1294帧/73.9s，丢线29.8%）+ 第三人称视频 72.1s。用户反馈：“第二次的转弯过早导致小车之后要摇头巡线”。

- 视频 6–7s 车**自己沿线拐进环岛虚线圆**（无VV触发），随后顺时针一圈圈绕行；CSV 里对应 SEEK 沿环线走，每圈经过岔口重新触发 VV：第一处 ENTRY CSV t=18.1、第二处 t=60.6，第三次 t=73.0。
- ENTRY 前车正在绕环：CSV 前几帧 w 在 +1.20（左）与 −0.85（右）之间摆动——**绕环本身就在左右摇头**（环线虚线密、线在画面边缘，控制器反复打满/反向）。VV岔口一锁存，ENTRY 再强加 +1.2 rad/s 左转（与当前 −0.85 右转相反）→ 0.86s 后释放 → 左找 1.1 rad/s 丢线 4s+ → 重新抓线。视频上就是“转弯过早 + 之后摇头巡线”。
- 两次 ENTRY 仍未进 FOLLOW：ENTRY 期间路径始终 `black_outside_locked_route`/`line_too_few_bands`，3帧连续路线凑不出；SETK 期 `heading_ok=0` 只是 measured_turn 不更新（非IMU故障），ENTRY 内实测只转了 +11.7°（0.86s），离确认角 55° 差很远——轮足在蓝/白交界打滑，实际转角远小于指令。

本轮把 `entry_follow_only` 判定扩展为：上一帧强左转（w≥0.75*entry_turn）**或最近1秒累计指令转角 |Δyaw|≥0.35 rad（≈20°）**。这样“绕环时被岔口误触发”也不会再叠加盲左转，车沿现有曲线走。直道/居中入口仍探左。

验证：新增 `test_entry_follows_an_existing_curve_from_cumulative_recent_turn`；完整测试 23 项历史失败不变。

边界：这只能消除“二次盲转/随后大左找”的摇头来源，**不解决绕环时 FOLLOW 永远不确认**（路线闪、实测转角不足）。下一步二选一：容忍闪帧/降确认门槛，或改成“入环后按累计绕行角度确认 FOLLOW”的状态机版本（需另立版本，影响 exit/crosswalk 凭据，不能只调参数）。
