# 轮廓巡线 + 拐点进/出环（第一版，拐点默认关闭）

需求（用户 2026-11-17）：普通巡线不再只找黑线，改为“左右边线轮廓法为主、中心黑线为辅”：

- 连通域掩膜 + 左右边线 + 中线；
- 有黑线时正常按黑线走，但状态机持续判断左右拐点；
- 进环：检测左上拐点，把右侧中线部分与拐点相连作为右边线，车向左拉；
- 出环：看不到黑线时，检测“Y 基本不变、X 突然减小”的右侧折角，把线往左拉；
- 无黑线时用轮廓中线做短时接管；
- 裁剪更激进；
- 视频验证能否跑通。

## 实现

新增 `scripts/patrol_contour_line.py`：

- `_component_mask`：白 + 不确定灰（`contour_corner_gray_v_min`）− 蓝，形态学闭运算，连通域只保留触底且面积达标的那个，填孔；
- `_boundaries`：逐行取连通域左右端点，带断行上限和跳变限制；
- `observe_corners`：按 near/mid/far 行算中线误差/置信度；
- `_corner_candidates`：左边界上最高曲率点（左上、向右下转）作为入环拐点；右边界上“dy 小、dx 负跳变”作为出环拐点；
- `CornerRingState`：入环/出环拉线窗口、冷却、`in_ring` 标志，`pull()` 返回横向拉线量；
- `debug_draw`：画左右边线/中线/拐点/状态（实车菜单 4 预览可见）。

接入 `patrol_contour_test.py`：

- `PatrolVision.estimate` 每帧按开关计算 `observe_corners`，更新 `corner_state`（仅 `contour_corner_enabled`）；
- 黑线和道路都无效时，若 `contour_takeover_enabled` 且轮廓有效，用轮廓中线做短时接管路径；
- 拉线窗口内把选中路径的 error/near/mid/far 减去 `pull`（向右拉线→车向左），reason 标 `corner_entry_pull`/`corner_exit_pull`；
- 拉线窗口内忽略旧 VV 环岛 `roundabout_left_acquire` 覆盖，避免两套逻辑互相打架；
- VISION 日志新增 `corner=`、`cobs=`。

参数（`PatrolTuning` + YAML）：`contour_corner_enabled`、`contour_takeover_enabled`、`contour_corner_gray_v_min`、`contour_corner_min_area_ratio`、`contour_corner_min_angle_deg`、`contour_corner_entry_pull/seconds`、`contour_corner_exit_pull/seconds`、`contour_corner_cooldown_seconds`。

## 视频验证（用户提供的 006cd06…mp4，2433 帧）

离线跑 `observe_corners`：

- 连通域道路掩膜/中线：**2425/2433 帧有效**，左右边线/中线正常，说明掩膜+边界/中线这条主链可用；
- 入环拐点：93 次检测（含真实左上角和噪声），**出环右折角 0 次**；`pull` 会在这些帧频繁触发，直接开上车会一直向左拉。
- 因此最初默认：`contour_takeover_enabled=1`、`contour_corner_enabled=0`（拐点拉线关闭，待实车标定）。
- 用户 2026-11-17 要求打开拉线做实车验证，已把 YAML 改为 `contour_corner_enabled=1`。如果实车出现“频繁左拉”，先提高 `contour_corner_min_angle_deg`（55→60~70）或增大 `contour_corner_cooldown_seconds`，不要先动拉线量。

## 验证

- 新增 `test/test_contour_line.py` 5 条：连通域边界、入环/出环拐点几何、状态机入环→出环拉线、冷却、参数校验。
- 完整测试：744 passed / 23 failed / 11 skipped；23 项为修改前已有历史/合同失败（含旧的全局 SHA256SUMS 过期项），无新增功能失败。

## 下一步标定

1. 实车把 `contour_corner_enabled` 热加载为 1，菜单 4 预览会画出左右边线/中线/拐点；
2. 看真实进环/出环帧的拐点位置，调 `contour_corner_min_angle_deg`（先 60~70）、拐点区域过滤（当前 x 8%~62%、y 12%~58%）；
3. 出环右折角在当前视频里没出现，需要用户录一段“出环看不到黑线”的实车视频再调 `exit` 判定；
4. 标定完再从 0.20~0.28 的拉线量开始实车，别超过 0.3，避免直接甩出赛道。
