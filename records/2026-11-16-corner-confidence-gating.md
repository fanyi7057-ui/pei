# 参考 V2.3：急弯权限的置信度门控 + 滤波上限

来源：用户提供的 `04_V2.3_环岛入口距离1.00.zip`（基线 `0fa165e`，含上游 `93aca87` 与 V2.2 本地改动）。解压后逐类对比 `FixedSpeedController`：**我们的版本与 V2.3 的该类唯一差异就是 `_steer` 里的急弯置信度门控和 alpha 上限**；其余（bend_slow、straight、命令平滑）已经一致。

## 问题对应

17:05 采集里 S 弯 0.65 m/s 段，线误差在 ±0.8 间来回，转向指令被顶到 ±2.0；日志里出现 `corner=1.00` 而 `conf=0.42~0.58` 的帧。旧 `_steer`：

- 只看 `|error|`，`>=corner_full_error` 就给满权限 2.0；
- 同时 alpha 升到 1.0，等于“越急弯越不滤波”。

结果是画面在两块黑结构之间跳一次，车就满舵甩一次，出弯后继续来回，形成抖动/长丢线。

## 改动

| 参数 | 默认 | 作用 |
|---|---:|---|
| corner_min_confidence | 0.45 | 低于此置信度，急弯权限按 0 处理 |
| corner_full_confidence | 0.75 | 达到此置信度才给满急弯权限 |
| corner_filter_alpha_ceiling | 0.88 | 急弯时滤波 alpha 上限（旧为 1.0） |

`FixedSpeedController._steer` 与 V2.3 逐行一致：

```python
self.corner_strength *= clip((path.confidence-corner_min_confidence)/
                             (corner_full_confidence-corner_min_confidence),0,1)
alpha = error_filter_alpha + (alpha_ceiling-error_filter_alpha)*corner_strength
```

单带救线（`path.rescue`）仍用原来的 `rescue_corner_confidence` 缩放，不进入这条置信度门控；环岛、找线、速度参数都不改。

## 验证

- 移植 V2.3 的两条测试到 `test_patrol_contour.py`：
  - 低置信度 0.30 / 大误差 0.80 → `corner_strength≈0`；0.60 → 介于 0..1；0.95 → 1.0。
  - 满权限时 `filtered_error=0.88*0.80` 而不是 `0.80`，锁住 alpha 上限。
- `FixedSpeedController` 源码摘要从 `f897bd17…` 变为 `55b2b8c1…`，已更新 `test_reviewed_ordinary_algorithms_and_numeric_speed_defaults` 的 pin；PathSelector 摘要未变。
- 完整测试：736 passed / 23 failed / 11 skipped；23 项与修改前历史/合同失败完全相同，无新增功能失败。

## 边界/未验证

- V2.3 本身未实车验证；置信度门控是按其记录中的“conf 0.42~0.58 的大误差是跳变”推断的。若实车真急弯被压得太软，先降 `corner_min_confidence` 到 0.35，仍摆再提 `corner_full_confidence` 到 0.85；每次只改一个。
- 本改动只处理“转向权限/滤波”，不替代已加的弯前降速、找线方向、出环后保护；这些仍按各自开关工作。
- 未改固件、底盘差速或比赛栈（`icn.py`/`icn_adaptive.py`）。
