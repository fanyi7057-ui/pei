# good3_1 出环后再次丢线：沿新主路趋势低速找回

基线：`Two_good3_1_core_fast_exit_post_main_filter.zip`。

本次只修改正式出环后的 `SEEK_CROSSWALK` 丢线恢复：

- 初次刚出环、尚未重新看到普通主线或 `POST MAIN` 时，保持原快速出环找线逻辑不变。
- 一旦普通连续主线或 `POST MAIN` 虚拟主线真正接管过一次，记录“出环主路已重获”。
- 此后若普通线和 `POST MAIN` 同时暂时丢失，不再重新固定向左找；改为使用刚才主路的 near/mid/far 趋势方向恢复。
- 趋势向左消失就向左找，向右消失就向右找，趋势近似居中就接近直行。
- 该局部恢复前进速度限制在 0.10~0.16 m/s，角速度限制在 ±0.30 rad/s，避免再次丢帧时高速前冲或大角度转圈。
- 普通巡线、环岛 marker、2.00s 入环窗口、FAST_ENTRY、ROUNDABOUT_FOLLOW、gyro 出环、快速出环、POST MAIN 左支过滤及 `adaptive_tuning.yaml` 均保持不变。
