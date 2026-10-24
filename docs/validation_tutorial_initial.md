> 历史归档：这是教程整理后、数值裕量修复前的记录。当前入口见 [validation.md](validation.md)。

# 怎样判断实验效果与复现范围

实验命令与练习集中在 [实验路线](lessons.md)，数学解释见 [控制数学](control_math.md)。
本页说明判据和本轮实际结果；历史记录保留但不作为当前功能的自动验收结论。

## 1. 先区分正常结束、到达和被替换

`execution_completed` 表示没有异常或监测停止；`duration_completed` 另外检查是否运行满计划时长。
退出 0 不保证到达。`reached` 要求末端误差 <0.01 m、末端速度 <0.02 m/s，连续保持 0.2 s。
`superseded` 表示到达前被更新目标替换，`rejected` 表示本次局部 IK 未通过，不证明全局不可达。
`active` 表示仍在追踪，`pending` 表示尚未激活。已 reached 的历史状态保留，之后仍可能有扰动。

`fixed_goal_acceptance` 仅用于保持、固定目标和扰动实验，要求末目标历史到达且最后位置/速度仍满足阈值。
快速切换实验不要求每个目标到达，最终误差也不能当成稳态误差。

## 2. 先看报告，再查原因

每次运行自动生成 `report.md`、`summary.json` 和六张图。
推荐顺序是 joints → timing_and_fallback → error → acceleration → torques → events。

| 内容 | 判断方法 |
|---|---|
| joints | actual 与 reference 是否贴合；若参考本身减速，不能直接判断为执行跟不上 |
| timing_and_fallback | 上图观察控制耗时与 20 ms 周期，下图区分 alpha 回退和显式制动 |
| error | 与当前目标的末端距离；竖线是目标切换，换目标导致的误差跳变很正常 |
| acceleration | MPC 理想加速度、下发参考加速度、真实速度后向差分是不同的量 |
| torques | raw 是合成力矩，applied 是总限幅后力矩；两者分开才能判断饱和 |
| events | 目标编号与注入外力，便于和其他图按时间对齐 |

`alpha=1` 接受原候选；减小时更接近制动候选。但显式制动时原候选已是制动段，alpha=1 也可能在制动。
`reference_backtracks` 统计 alpha<1，`braking_seconds` 统计显式制动时间，不能混为同一个指标。

新摘要和 MPCResult 区分：

| 字段 | 发生了什么 |
|---|---|
| `mpc_input_rejections` | 实测状态或输入无效，在调用求解器前拒绝 |
| `qp_solver_failures` | 求解器返回不可行、迭代耗尽等未接受状态 |
| `qp_solution_rejections` | 有求解结果，但有限性、残差或约束复核未通过 |
| `mpc_rejections` | 上面三项的总和 |
| `predicted_contact_rejections` | 求解后预测构型的接触检查拒绝，独立统计 |

旧字段 `qp_failures` 保留历史含义，即全部 MPC 拒绝数。分析器可根据旧日志的 status 推断分类；
旧证据文件本身不被改写。到达、求解质量与性能都需要结合原因判断，不能只看一个计数。

定位命令及双实验比较见 [README](../README.md)。`--at` 选择最近的真实样本，附上该样本之前最近一次控制 tick；
`--against` 要求采样时间和目标序列相同，也允许两次运行有相同的“首个目标之前无目标”空白期。
参数比较的末端 RMS 只统计有有限目标的样本；目标不同的两次运行会明确拒绝比较。
`--compare` 则专门检查只有外力开关不同的扰动对照。

## 3. 本轮教程整理实际运行的结果

本轮使用现有 Python 3.10.20、MuJoCo 3.3.7、mink 0.0.13、OSQP 1.0.4 环境。
没有更改控制代价、运动限制、参考制动公式或到达阈值；控制层新增失败分类，主循环提取目标激活函数。

| 实验与证据 | 时长 | 最终误差 m | 输入拒绝 / 求解器失败 / 解复核拒绝 | 参考回退 |
|---|---:|---:|---|---:|
| [固定目标](evidence/tutorial_revision/single_summary.json) | 10 s | 0.000859 | 0 / 0 / 0 | 0 |
| [固定切换](evidence/tutorial_revision/switch_summary.json) | 12 s | 0.001259 | 0 / 0 / 0 | 0 |
| [平滑权重 3 的七轴切换](evidence/tutorial_revision/smooth_summary.json) | 12 s | 0.001025 | 0 / 0 / 0 | 0 |
| [半速随机快速切换](evidence/tutorial_revision/rapid_summary.json) | 30 s | 0.216964 | 7 / 0 / 0 | 270 |

四次均运行满计划时长，饱和步数为 0；前面三个末目标 reached，快速切换末目标仍 active。
[局部诊断](evidence/tutorial_revision/rapid_inspection.json) 中 0.86 s 的 J2 速度约 0.350027 rad/s，
略超配置上限 0.35，属于输入状态拒绝，不是 OSQP 求解失败。

[参数对照](evidence/tutorial_revision/parameter_comparison.json) 使用同一固定事件表：
平滑权重 3 与默认 0.3 的末端误差 RMS 分别约 0.076986 / 0.075808 m。
平滑配置的关节跟踪 RMS 略小，但末端 RMS 没有因此更小，不能宣称增大权重无条件更好。

教学脚本也已实际执行并自动生成图片：

- [单关节默认](evidence/tutorial_revision/mpc-base.json)：最终理想位置误差约 3.45e-9 rad。
- [单关节平滑权重 1](evidence/tutorial_revision/mpc-smooth1.json)：最终理想位置误差约 5.18e-9 rad。
- [MIT 逆动力学前馈](evidence/tutorial_revision/mit-inverse.json)：5 s、无饱和；每轴 RMS 见摘要。

这些是分别验证理想运动模型和执行层，不替代整机实验。单关节脚本的权重 3 在本机出现约
6.9e-7 rad/s 的预测点速度超界并被严格初值检查拒绝，已写入进阶故障说明；没有裁剪状态规避问题。

本轮检查结果：

| 检查 | 实际结果 |
|---|---|
| `.venv/bin/python -m pytest -q -W error` | **43 passed，49.96 s**，包含真实闭环和可选视频测试 |
| 拒绝分类、旧日志兼容、时间定位、参数比较 | 新增测试通过，含首个目标前 NaN 空白期 |
| 30 s 快速切换与此前实验对照 | t、q、dq、q_des、dq_des、tau_applied、goal_id 数组逐项完全相同 |
| 环境自检与 EGL 真实渲染 | doctor、doctor --video --render 均通过；明确提示外部环境继承 |
| 38 项版本约束审计 | 与现有分发包版本及 Python 3.10 要求一致 |
| 安装脚本已有环境保护 | 返回 2，原 .venv 不被覆盖 |
| 文档链接、Python 编译、Shell 语法、差异格式 | 通过 |

完整 [检查记录](evidence/tutorial_revision/checks.txt)、[测试输出](evidence/tutorial_revision/pytest.txt)
和 [单关节数值边界案例](evidence/tutorial_revision/known-bound-case.json) 已保留。
本轮没有重跑三种前馈消融、全部七场景验收、多 seed 系统回归或 GUI 自动验收。

## 4. 安装、打包与自动验证的实际范围

主要依赖版本固定在 [pyproject.toml](../pyproject.toml)。
[Python 3.10 约束](../requirements/constraints-py310-linux.txt) 固定了当前环境的活跃依赖闭包，
包括测试依赖；它是本机元数据审计后的约束快照，不等于已在全新机器验证过的锁环境。

本轮在 `/tmp` 创建了 `include-system-site-packages=false` 的独立 venv，
尝试从 PyPI 安装 NumPy，但索引未提供可取得的发行包，安装失败，实际输出见
[干净安装日志](evidence/tutorial_revision/clean_install.txt)。不能据此宣称端到端干净安装通过。
现有工作环境仍继承 Conda 包，doctor 会明确提示。

本轮成功构建 wheel，并将项目 wheel **不带依赖**安装到独立 venv，从 `/tmp` 验证了
67 个网格、XML、默认配置和第三方许可随包存在。该检查只证明打包和资源定位，不证明该独立环境能运行仿真。

[GitHub 工作流](../.github/workflows/tutorial.yml) 配置了 Ubuntu 22.04、Python 3.10/3.11
的安装、pip check、基础测试与首课验收，视频 EGL 另设任务。它尚未上传执行，不能算作已通过的 CI。
Python 3.12 尚未在本轮验证。安装脚本拒绝覆盖已有环境，基础安装不要求视频依赖。

## 5. 性能与算法边界

`qp_timing` 是 MPC 包装函数时间，历史上也包含求解前拒绝的快速返回；`control_tick_timing`
还包含目标处理、同步 IK、预测查询和参考更新等。渲染/录制不包含在这些控制计时里，
但会影响整个运行的墙钟时间。P50/P95、最大值和超预算次数都要看，不能只报平均值宣称硬实时。

IK 是局部数值求解；随机采样经过 home IK 预筛，且有墙钟预算，所以固定 seed 不保证
不同机器的接受序列逐项相同。正式参数对照优先采用固定事件文件，数值结果用合理容差比较。
MPC 预测与下发参考的起点不同，回退还会修改优化输入；这一限制在数学文档中明确说明，尚未在本轮更换算法。
没有硬 jerk/力矩预测约束、递归可行性保证、连续全局避障、参数自适应或真机安全控制。

## 6. 历史证据

原始实验数据、图片、安装/打包日志与模型许可均保留：

- [2026-10-24 之前的完整迭代记录](validation_20261024.md)：随机快速切换、视频与播放兼容性。
- [2026-10-21 修复与真实闭环验收](validation_20261021.md)。
- [最初缺包阶段](validation_initial.md)，不代表当前依赖状态。
- [原有七场景命令](evidence/closed_loop/commands.json) 及相邻摘要；旧 rapid_switch 是固定三点轮换。
- [原生 OSQP 理想模型摘要](evidence/native_osqp/summary.json)：使用 CasADi 插件，非生产 Python 接口或 MuJoCo 实验。

GUI 窗口尚未在工具环境中完成自动验收，离屏渲染和双格式视频路径已有真实测试。
