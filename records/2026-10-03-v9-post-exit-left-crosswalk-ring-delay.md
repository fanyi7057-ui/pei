# V9：出环保持左向 + 首次人行横道后延迟第二环岛

基于 V8 实车日志修正两处状态边界。

1. 第一圈出环后，`ROUNDABOUT_EXIT -> SEEK_CROSSWALK` 的恢复保护阶段不再允许负角速度。即使视觉主线短暂偏向右侧，也至少保持 `roundabout_post_exit_min_left_turn_speed` 的左弧，直到最短保护时间完成且主线连续稳定重获，或达到最大保护时间。
2. 第一次人行横道在 `SEEK_CROSSWALK -> CROSSWALK_APPROACH` 确认时启动第二环岛专用冷却。默认 15 秒；第二圈即使已经开始，只正常巡线，不累计环岛 marker。终端 `RINGDBG cw2=` 显示剩余秒数。

关键参数：
- `roundabout_post_exit_min_left_turn_speed: 0.12`
- `roundabout_post_exit_max_left_turn_speed: 0.22`
- `roundabout_post_exit_max_right_turn_speed: 0.00`
- `second_roundabout_arm_delay_after_first_crosswalk_seconds: 15.0`
