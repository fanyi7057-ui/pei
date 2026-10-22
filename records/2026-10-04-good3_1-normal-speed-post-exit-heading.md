# good3_1：普通巡线平滑加速 + 出环后 gyro 航向保持

基线：`Two_good3_1_core_fast_exit_post_main_filter_trend_recovery.zip`。

本次只处理最新实车日志暴露出的两个问题，不改环岛识别、2.00s 主线窗口、FAST_ENTRY、ROUNDABOUT_FOLLOW、gyro 360°退出判定、POST MAIN 左支过滤和出环后重获主线的趋势恢复。

## 1. 普通巡线一顿一顿

实车视觉有效频率约 14~16Hz。good3_1 的线速度目标会随弯道曲率及时下降、出弯后再升高；原自适应输出层加速斜率为 0.48m/s²，因此每个新视觉帧会形成较明显的速度台阶。

本版仅在普通巡线阶段 `SEEK_ROUNDABOUT / SEEK_CROSSWALK / SEEK_FINISH` 把“重新加速”斜率限制到 0.22m/s²。减速仍使用原值，转向、弯道速度目标、视觉误差和环岛 FOLLOW 输出均不变。

## 2. 第二圈 gyro 出环后又物理卷回环岛

日志确认状态机没有第三次进入 `ROUNDABOUT_FAST_ENTRY`。第二圈 gyro 在约 349° 已正确切换到 `SEEK_CROSSWALK`，但刚出环时视觉无线，旧的 post-exit 快速左找继续输出 +0.41→+0.55rad/s，导致车体物理上重新向环岛弯回。

本版在“正式出环、尚未重新看到主路、gyro 航向参考仍有效”期间：

- 前进速度仍保持 0.24m/s；
- 转向只使用从当前累计角收敛到 360° 的 gyro heading command；
- 不再叠加固定 +0.35→+0.55 左找；
- 普通连续线或 POST MAIN 一出现，立即由视觉接管；
- 主路已经重获后再次丢线，继续使用上一版新增的主路新趋势恢复（0.10~0.16m/s、±0.30rad/s）。

这样 gyro 仍只在“出口无主路线索”的短窗口保护方向，不压制有效视觉巡线。
