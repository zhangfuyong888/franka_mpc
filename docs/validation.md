# 实验效果与复现验证

先按 [实验路线](lessons.md) 运行；算法和约束对应 [控制数学](control_math.md)。
本页记录最新修复后的结果，原始历史证据另行归档。

## 1. 最新验证结果

主要验证环境：Ubuntu 22.04、Python 3.10.20、MuJoCo 3.3.7、mink 0.0.13、OSQP 1.0.4。

| 验证 | 实际结果 | 证据 |
|---|---|---|
| 原工作环境完整测试 | **49 passed，54.28 s**，警告按错误处理 | [测试输出](evidence/finalization/pytest-final.txt) |
| 独立 venv 构建、安装、pip check、自检 | 通过，未继承原 site-packages | [隔离流程](evidence/finalization/isolated-verification.json) |
| 独立 venv 非视频测试 | **48 passed，1 deselected，48.55 s** | [测试输出](evidence/finalization/isolated-tests.log) |
| 独立 venv 第一课 | 运行满 10 s，末目标达到 | [第一课摘要](evidence/finalization/isolated-first-lesson-summary.json) |
| 完整场景验收 | 七场景、无扰动对照和对照分析，共 9 条命令均通过各自判据 | [命令记录](evidence/finalization/acceptance-commands.json) |
| 原先失败的单关节平滑权重 3 | 连续 400 步通过，最终位置误差约 1.01e-8 rad | [摘要](evidence/finalization/single-joint-smooth3.json) |
| 单关节回归范围 | 权重 0.3/3/10，预测步数 15/25/40 的选定组合全部通过 | tests/test_integration.py |

独立测试验证的是 **wheel 内的代码**，不是源码目录的 editable 安装。
[模块来源记录](evidence/finalization/isolated-package-locations.log) 确认核心模块都在独立环境中；
[依赖检查](evidence/finalization/isolated-pip-check.log) 没有损坏依赖。

## 2. 修复了什么数值问题

旧版在速度/加速度的原边界上直接优化，OSQP 接受的数值误差有时会让预测点轻微越界：
下一周期实测初值检查很严格，或者参考整段检查更严格，就会产生不必要的拒绝/回退。

现在默认 `mpc_motion_margin=0.001`，只把优化器内部的速度、加速度界收紧 0.1%，
保留原始实测初值及对外操作限制。求解后还独立检查未来 q/v/a 没有超过原操作界，
违反时标记 `solution_rejected:operating_bounds`，不把不合法预测交给下一周期。
没有裁剪真实状态、放宽到达阈值或扩大力矩上限。

最新真实实验：

| 实验 | 时长 | 最终误差 m | MPC 拒绝 | 参考回退 | 饱和步 |
|---|---:|---:|---:|---:|---:|
| [固定目标](evidence/finalization/single_summary.json) | 10 s | 0.000859 | 0 | 0 | 0 |
| [固定切换](evidence/finalization/switch_summary.json) | 12 s | 0.001255 | 0 | 0 | 0 |
| [平滑权重 3 的整机切换](evidence/finalization/smooth_summary.json) | 12 s | 0.001024 | 0 | 0 | 0 |
| [半速大范围快速切换](evidence/finalization/rapid_summary.json) | 30 s | 0.219967 | 0 | 202 | 0 |

四次均运行满计划时长。半速快速切换的输入拒绝由旧版 7 次降为 0，显式制动累计由 0.14 s 降为 0。
保留的 202 次参考回退是约束处理结果，不等于求解失败；裕量不保证完全没有回退或任意工况都可行。
快速切换最后仍可能未到达，最终误差不能作为固定目标的稳态精度。

完整验收中，hold、single_target、disturbance 满足固定目标判据，switch、random、rapid_switch
正常完成，invalid_goal 按预期拒绝并退出 2。扰动与无扰动对照最大末端偏移约 0.006780 m，
撤去后按原位置/速度/驻留条件恢复。恢复由 MIT 反馈和 MPC 等共同影响，不能只归因于 MPC。

## 3. 安装验证的来源与边界

在线 PyPI 请求在当前工具环境返回 `Operation not permitted`，实际探测见
[网络记录](evidence/finalization/online-probe.txt)。不能声称本轮完成了从 PyPI 下载的全新安装，
也没有把远端 GitHub Actions 标为已运行。

为验证代码不依赖现有 Python 环境，本轮从本机已安装分发包构建了**仅用于本机测试的离线 wheelhouse**，
包含明确记录的 Conda BLAS/Fortran 运行库副本，再在 `include-system-site-packages=false` 的新 venv 中
安装构建工具、构建项目 wheel、安装全部依赖，运行 pip check、测试和第一课。
[38 项离线依赖来源与哈希](evidence/finalization/offline-dependency-provenance.json) 已保存。
这些重打包二进制不随 GitHub 仓库发布，不应冒充官方 PyPI wheel 或跨机器通用离线包。
这证明本机隔离运行成功，不证明在线索引、Python 3.11/3.12 或另一操作系统都已验收。

可在有网络的机器执行同一流程：

```bash
python scripts/verify_install.py
```

脚本会新建输出目录及独立环境，构建 wheel、安装依赖、检查模块来源、执行非视频测试和第一课，
保存 `verification.json` 及每一步日志。已有输出目录会拒绝覆盖。
已有同平台依赖 wheelhouse 时可显式指定 `--offline-wheelhouse /path/to/wheels`；来源会写入记录。

[GitHub 工作流](../.github/workflows/tutorial.yml) 已配置 Python 3.10/3.11 的测试和第二个干净环境验收，
视频 EGL 独立测试；上传后自动触发，尚未在远端执行。Python 3.10 的
[版本约束](../requirements/constraints-py310-linux.txt) 固定依赖版本，但不是带官方 PyPI 文件哈希的锁文件。
安装脚本拒绝覆盖已有环境，基础教程不要求 FFmpeg、窗口或视频录制。

项目原创源码和文档使用根目录 [MIT 许可证](../LICENSE)，模型及网格继续保留原 Apache-2.0 许可，
范围见 [第三方声明](../THIRD_PARTY_NOTICES.md)。

## 4. 怎样判断过程状态

`execution_completed` 表示没有异常或监测停止，`duration_completed` 表示运行满计划时长。
退出 0 不等于到达。`reached` 要求末端位置误差 <0.01 m、速度 <0.02 m/s，连续保持 0.2 s。
`superseded` 是未到达前被新目标替换；`rejected` 是本次局部 IK 未通过，不证明全局不可达。
快速切换中被替换不一律算失败，已达到的历史状态也不代表之后永远没有扰动。

先打开 `report.md`，再按 joints → timing_and_fallback → error → acceleration → torques → events 看图。
实际与参考是否贴合用于判断执行跟踪；参考本身减速时，不要误判为电机跟不上。
alpha 是参考候选向制动候选调整的比例，不是新旧目标权重；显式制动时 alpha=1 也可能在制动。

| 摘要字段 | 发生了什么 |
|---|---|
| `mpc_input_rejections` | 输入或实测状态无效，尚未调用求解器 |
| `qp_solver_failures` | 求解器返回不可行、迭代耗尽等 |
| `qp_solution_rejections` | 解没有通过有限性、残差、原操作界等复核 |
| `mpc_rejections` | 上面三项总和 |
| `predicted_contact_rejections` | 求解后预测接触检查拒绝，独立统计 |

历史 `qp_failures` 保留原意，即所有 MPC 拒绝数。旧日志可据 status 推断分类，不会修改原始证据。
`reference_backtracks` 是 alpha<1 次数，`braking_seconds` 是显式制动时间，不混为一个计数。
定位某时刻和参数对照的完整命令见 [README](../README.md)。同任务比较允许两次相同的首目标前 NaN 空白期，
RMS 只统计有有限目标的样本；不同任务会拒绝直接比较。

## 5. 仍需区分的算法边界

这里是关节空间线性运动模型，不是完整动力学 NMPC。IK 是局部求解；预测接触检查不是连续全局避障。
MPC 从实测状态预测，参考从上一接受参考接续，回退还可能修改输入，二者不完全一致。
没有硬 jerk/力矩预测约束、递归可行性证明、参数自适应或真机安全保证。

IK 有同步墙钟预算，目标激活可能超过 20 ms。QP 包装时间、整个控制 tick、物理积分、渲染和总墙钟
是不同统计范围；录制增加墙钟时间，不代表物理仿真时间调度改变。固定 seed 也不保证跨机器逐位相同，
正式参数对照优先使用固定事件表及数值容差。

## 6. 历史资料

旧记录和原始实验数据全部保留，不拿历史结果充当当前功能已验证：

- [教程初次整理、数值修复前](validation_tutorial_initial.md)。
- [此前随机切换、视频与兼容性记录](validation_20261024.md)。
- [2026-10-21 闭环验收](validation_20261021.md)、[初次缺包记录](validation_initial.md)。
- [最初七场景命令](evidence/closed_loop/commands.json)：旧 rapid_switch 是三个固定点。
- [原生 OSQP 理想模型结果](evidence/native_osqp/summary.json)：不是 MuJoCo 物理实验。

GUI 窗口未在工具环境中完成自动验收；离屏渲染与双格式视频已有真实测试。
