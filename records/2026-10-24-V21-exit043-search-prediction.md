# V21 0.43米出口目标与FOLLOW短时搜索预测

依据日志06fa2a2a：最后一次ENTRY->FOLLOW成功；约154/204度出现black_outside_locked_route，之后连续丢线；EXIT在exit_seek_m约0.18后触及0.22距离保护，未确认出环。中途manual_switch与其后重试不算连续自动完赛。

- 出口寻找目标0.43米、4.5秒硬期限、5秒出口获取总期限；保持0.12m/s，理论0.43米需3.58秒。先遇可靠出口则接管，不走满；没odom不能声称走够世界坐标距离；航向/相机/遥控保护不变。
- RingSearchPredictor仅用最近两次真实锁定且route_observed、非rescue/handoff的至少3点路线，估计像素运动趋势；最多0.35秒，位移限0.10图宽。只供FOLLOW原匹配失败后的当前图像搜索，不能输出PathEstimate、不能更新观察期限、不能生成完成凭据；每次仍匹配当前真实黑块。
- 暂停/重置、非FOLLOW清空历史。普通巡线、ENTRY和人行道不启用预测，无新增状态机。原始PathSelector/FixedSpeedController及速度不变。
- 日志version21；新增ring_predict、ring_match、ring_delta、ring_ref_age、ring_points，用于区分没有线和路线归属拒绝，不直接放松归属阈值。
- 历史测试更新出口距离/时间预期，并保留旧场景安全断言，新增预测/无像素/时间期限回归。

实测清单：1普通直道单弯S弯无变化；2入口4进5出；3FOLLOW偏离参考小范围搜索，空白/单点/歧义不接管；4手动/暂停再开始清掉预测；5出口0.43米前遇线即接管，侧移倒退不给行程；6停车/航向偏离/相机故障仍保护；7两圈横道/终点不越级。离线测试不能保证实车完赛；更长盲搜增加偏出赛道风险，需遥控急停。

离线结果：关联635通过，全套802通过、11跳过、23失败，失败名称与V20逐项相同。Python3.10语法、同作用域重复定义、LF和git diff --check通过，PathSelector/FixedSpeedController源码不变。菜单仅提示文字变化；本机Git Bash受沙箱限制，不声称已执行本轮bash -n。未做实车验证。

部署到/home/orangepi/Two_patrol后：`sha256sum -c checksums/PATROL_TEST_ADDITIONS_SHA256SUMS.txt && bash tools/patrol_test_menu.sh`，先1干跑再2实车；SWA自动档输入1。无需重编译C++。
