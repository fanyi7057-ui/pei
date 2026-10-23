# V15完整Two覆盖包：候选区路线稳定与环岛完成衔接

## 保留与测试范围

普通PathSelector和FixedSpeedController整类与V11提交50db743一致，原0.65m/s、姿态、急弯/救线、黑线/轮廓策略不改。原台阶动作/跳后恢复/固件/CMake源码不改，菜单4仍不执行跳跃。保留V12正确的共享IMU发布、V13获取/倍率/出口策略和V14严格两圈凭据。

当前流程：台阶①占位跳过→环岛①→横道①停车后正常巡线→台阶②占位跳过→环岛②→横道②→设定距离→终点停车。台阶槽位SKIPPED_TEST不表示已跳过实际台阶，也没有开关可直接集成原跳跃；以后需在槽位接回原动作完成事件。菜单5仍是独立原比赛入口。

## 1. 这次针对什么问题

上一终端一直L1_RING、无软件ENTRY/FOLLOW/EXIT，但场外视频首圈确实物理绕行；横道有诊断高分却因无环岛凭据被禁止。入口要求“每帧可靠主路＋同链靠近”，主路在缺线/救线/交接时不断失效；候选区仍普通抢线，大误差又放大转向。V15修参考来源和短失线衔接，不恢复18秒或30/45秒越级，不用时间伪造出环。

这是基于源代码/日志与场外轨迹的改进，不是已经看到车载二值图或重放同一相机图像的结论。视频无法与ROS时钟精确对齐，不能保证全部抖动都由这一机制造成。

## 2. 候选区独立参考与同路选择（只在APPROACH改变选路）

新增ApproachRouteMemory：

- 只记真实多行、非单点救线/非交接的黑线或道路参考；候选链与路线在共同行高太接近时，不能用候选链自己刷新主路。部分链尚未攒满跨帧确认也参与排除，防先污染参考。
- 独立参考默认最多保留0.80秒，不能由缺帧、救线或候选链刷新。超时/人工暂停/重置/几何修改/任务退让清除。
- APPROACH每帧计算当前道路观测，优先与参考近/中误差一致的当前黑线或轮廓；不能凭旧记忆造一条看不见的路线。
- 突然只有另一支/没有匹配路线时，默认最多0.30秒拒绝该跳支，交给**原V11**短失线恢复；超窗解除额外拒绝，恢复普通救线，不无限硬锁旧路。
- 尚未进入APPROACH的SEEK与普通弯道不添加这个控制覆盖。两个普通类保持原字节逻辑；不是全程削弱急弯能力或加固定左转。

实际道路弯曲过快可能暂时超出同路容差，窗口过长可能滞后；这是需实车校准的取舍。不能声称候选区绝不抖动、每条错误分支都能拒绝。

## 3. 入环证明允许短缺帧，但仍需当前真实链

保留V14同一链靠近、左侧分开/张开、近端位置和最少观察时间。改为使用独立参考，不再只看上一帧选中路径是否处于route_observed。

链或参考短缺默认0.30秒内保留已积累证明，**缺失帧不增加次数、不刷新最后观测、不触发ENTRY**；新鲜右支/平行几何仍清空，跳链/过期重新确认。普通路径的短失线模板重置不再连带清空有界入口证据；人工暂停和显式重置仍清空。

仍需当前链到近场0.72图高、向近端推进0.03图高、连续观察0.35秒、独立主路比较、0.05米接近/2帧确认才执行ENTRY。蓝色岛心不作硬条件。55°仍是ENTRY后的获取门，不是启动位置门。入环后真实路线短缺0.25秒内不立即清空确认；缺失不攒次数，跳支/错误几何仍清零。

若真实入口前没有任何可用独立参考，或入射几何不满足，仍可能漏入；没有无证据自动左转。需现场查看entry_missing_chain、entry_missing_independent_reference、entry_not_left_opening_branch和参考年龄，不要一味降低所有阈值。

## 4. 完成判断：实测转角与可见离环路线共同佐证

原虚线/实线出口、满圈偏航候选仍保留。新增一条FOLLOW→EXIT候选：

- 软件已真实获取环内路线，虚线曾离开，FOLLOW满足原最少时间/行程；
- 偏航新鲜可靠，累计净转角达到原roundabout_visual_exit_min_turn_deg（默认240°），不能拿命令积分冒充；
- 当前真实路线无救线/交接、无分叉/近端缺口/虚拟补线，近端回中且近/中一致（默认差<=0.12）；
- 连续新帧保持默认0.35秒，缺失/错误形态/长间隔清空离环候选。

这**只提名EXIT**，不直接计完成，也不直接开放横道；EXIT仍需真实无分叉路线、原出口行程0.45米和多帧确认才形成该圈环岛凭据。普通急弯、原地转动、时间到或横道出现不能代替。直段圆弧也可能满足局部形态，因此必须结合实测环行进展，仍需实车核验，不是物理出口万能分类器。

ENTRY/EXIT路线证明对短失效保留默认0.25秒，不续期/不增加新帧；当前有效且同路的真实观测回来后才可转换任务。保留短时同路出口轮廓优先，不新增全程强制左偏。

## 5. 横道严格顺序，不拿漏识别换自动跳任务

本圈确认出环后，原1.5秒延迟及条纹连续帧满足才停车/计横道。第一次仍前行0.25米、停3秒，随后正常黑线/轮廓跟随；3秒CW_CLEAR收尾、停车结束起5秒仅防重复横道，绝不屏蔽巡线盲直冲。

第一横道收尾后第二圈必须重新取得环岛凭据；第二次合法横道默认走1米再FINISHED。finish_after_second_crosswalk=0时第二次也先停留收尾再距离冲线。累计数字不能替代两圈凭据，终点必须finish_authorized。

入环失败、候选超时、控制预算到期回SEEK重找本圈，默认2秒/0.50米后可重试；30/45秒是控制预算，不发完成凭据；18秒视觉捷径仍禁用。若漏掉环岛，车可能继续找当前环岛而不能完成，这比误授权终点更严格。人工急停、相机/通信/定位最终保护保留，不承诺只在终点才停，不无限盲走。

## 6. 唯一实时参数表与日志

本工程ros2_ws/src/wheel_robot/config/patrol_test.yaml。新增6项有中文用途、单位、边界及调节影响：

- roundabout_approach_memory_seconds=0.80：独立参考最长年龄；
- roundabout_approach_gap_seconds=0.30：链缺帧/当前路线短跳支窗口，不得大于参考年龄；
- roundabout_approach_route_delta=0.30：近/中同路误差容差，归一化；
- roundabout_path_dropout_seconds=0.25：ENTRY/EXIT缺观测保留窗口；
- roundabout_observed_exit_seconds=0.35：离环形态连续确认秒数；
- roundabout_observed_exit_agreement=0.12：离环近/中一致性。

以上只是调试起值，未板端标定。原V14所有已有参数数值不改；台阶未新增执行开关。退役visual_release字段不授权，resume_tracking必须1。完整匹配Python/YAML先覆盖重启，此后保存表约0.5秒热加载，不需要重新编译。非法整套保留上一有效值；修改几何/姿态按原规则清路径，任务期间不能随意关闭任务开关。

日志应为version=15、patrol_baseline=V11；关注entry_proof、entry_chain_n、approach_route、approach_reference_age（-1无参考）、last_entry_proof、route_status、实测转角、task/step_slots/ring_receipts/zebra_receipts/cw_block/finish_authorized。候选成立不是完成，原始横道分数不是已获停车许可。

## 7. 覆盖、显示与运行

ZIP顶层仍Two，不改名。先人工结束旧程序，备份当前可用工程和调参文件，将Two内部内容合并到实际工程，不能套Two/Two，也不删除原项目。实际目录叫Two_patrol就运行它自己的菜单：

```bash
bash /home/orangepi/Two_patrol/tools/robot_menu.sh
```

先2视觉干跑、3站立检查，再4V15实测；未编译正确共享底盘时先1。已编译V12/V13/V14正确底盘，本次两个Python/YAML覆盖重启即可，C++不再变化。必须看到IMU_QUATERNION_MAPPING_V12驱动标记，V15仍有V12字样正常。不要运行家目录旧菜单或继续传旧Two副本。

ROS Humble/colcon及原依赖必需。车载DISPLAY=:0/XAUTHORITY、录像关闭均保留，没有新增视频写盘。候选区增加道路观测与少量几何计算，未在Orange Pi验证FPS。原台阶源码不变，但共享IMU语义影响原订阅者，仍需核验原比赛流程；不烧录固件、不自动传车/驱车。

## 验证与边界

离线测试覆盖参考独立性/候选自刷新拒绝/短缺帧不续期/同路优先/过期放回救线、实际合成视觉可入环、真实环行进展与离环形态联合候选、两次真实出口→横道→距离终点、错误分支/小转角/陈旧偏航不得绕任务、参数校验以及普通两类V11源码摘要一致。

本次没有车载第一视角录像可重放；场外视频不能提供相机掩膜，合成测试不等于真实地图通过。无Orange Pi ROS2构建、烧录、驱车或帧率/稳定完赛保证。完整包由冻结li-5.39生成，181原文件179字节相同，仅批准菜单与共享IMU发布替换；总192文件顶层Two，CRC/摘要/LF/权限/语法检查，结果和ZIP摘要见交付记录。23项既有完整包失败不掩盖。旧包/Release保留，不自动替换Latest。

English: V15 keeps exact V11 ordinary controllers and frozen step slots. It adds bounded independent route memory and candidate-only same-route selection, retains approach proof across short non-renewing gaps, and nominates EXIT from measured ring progress plus sustained observed departure geometry. Each lap still needs an actual exit receipt before zebra permission. Offline synthetic/package validation is not hardware validation.
