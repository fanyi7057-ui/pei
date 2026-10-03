# 代码版本快照

Git 的提交记录保留完整历史；本目录额外保存近期关键修改涉及的源码快照，便于在 GitHub 网页中直接打开、比较或下载，而不需要切换提交。

每个版本文件夹只包含当次修改的 ROS 巡线控制文件：

- `scripts/race_controller.py`：状态机、巡线、环岛与速度控制。
- `config/race_controller.yaml`：ROS 默认参数。
- `launch/race_settings.py`：比赛启动时覆盖 YAML 的参数。

## 当前快照

| 文件夹 | 对应提交 | 内容 |
| --- | --- | --- |
| `2026-10-03_99ca0a3_roundabout-fork-detection` | `99ca0a3` | 环岛入口虚线簇与岔口方向判断。 |
| `2026-10-03_4ff697e_roundabout-entry-stability` | `4ff697e` | 出口等待上限、入口前保护、无台阶测试保护。 |
| `2026-10-03_3a3eb16_roundabout-tracking-response` | `3a3eb16` | 环岛内低速、高角速度、虚线丢失时逆时针找回。 |

之后每次修改上述比赛代码时，都会创建一个新的 `YYYY-MM-DD_<commit短号>_<说明>` 文件夹；已有版本快照不会覆盖或删除。
