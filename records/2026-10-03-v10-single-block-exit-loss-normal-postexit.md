# V10 — 环岛单块追踪 + 220°后丢线左找 + 出环立即正常巡线

基于 V9 实车日志修正三点：

1. ROUNDABOUT_FOLLOW 中若基础 scan-band 巡线判定 `line_valid=false`，但事件图仅剩一个有效黑色方块，
   使用该方块中心作为临时局部目标，按左右位置转向靠近，不再因为“连不成线”完全忽略。
2. gyro 累计历史峰值一旦达到 220°，之后环内如果丢线，直接按出口方向向左找；
   不再让旧趋势把转向改成右侧。使用峰值锁存，避免短暂右修导致净积分从 251°降到230°后规则失效。
3. gyro 确认出环后取消“固定速度 + w=0”的 blind straight。线有效时立即逐帧正常巡线；
   线无效时向左找出口。ROUNDABOUT_EXIT 完成切到 SEEK_CROSSWALK 后不再有 post-exit 运动覆盖，
   防二次入环只依赖 25s 环岛冷却、本圈完成锁、以及第一次人行横道确认后的15s第二环岛门。

关键热调参数：
- `roundabout_single_block_rescue_enabled: true`
- `roundabout_single_block_speed: 0.18`
- `roundabout_single_block_kp: 0.78`
- `roundabout_single_block_max_turn_speed: 0.45`
- `roundabout_exit_loss_left_arm_turn_deg: 220.0`
- `roundabout_exit_loss_speed: 0.18`
- `roundabout_exit_loss_left_turn: 0.30`
