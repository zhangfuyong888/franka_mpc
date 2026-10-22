> 历史归档：这是最初缺依赖阶段的记录，不代表当前环境状态。后续记录见 [2026-10-21 验证](validation_20261021.md)，当前入口见 [验证文档](validation.md)。下文的 `test_results.txt` 在后续修复时已更新为 22 项通过，不能作为最初 14 项通过/7 项跳过的原始输出；这里保留当时的文字记录。

# 实际验证记录与未完成验收

日期：2026-10-21。本项目按任务说明创建，但**尚未达到第一版完整完成标准**。主要阻塞是当前执行环境禁止终端网络，且本机未安装 MuJoCo、mink、OSQP Python。没有把未运行的物理指标填写成结果。

## 环境与安装

初始工作区只有 CODEX_TASK_MPC_MIT_V2.md，没有现有项目、references 或适用 AGENTS.md。系统 Python3.10.12 的 venv 因缺 ensurepip 失败；随后使用已有 `/home/zfy/miniconda3/envs/tv/bin/python` 3.10.20 在项目内创建 `.venv`，为离线验证读取既有 tv 的 scientific Python 包。不修改全局 Python、Git身份、系统或任何参考仓库。

执行安装：

```bash
.venv/bin/python -m pip install --disable-pip-version-check --no-cache-dir --retries 1 --timeout 5 mujoco osqp mink
```

实际失败：`ProxyError ... Failed to establish a new connection: [Errno 1] 不允许的操作`。去掉代理的 curl 也无法解析 pypi.org。`git ls-remote` 失败。见 [完整安装错误](install_attempt.log)。浏览/GitHub连接器可读取资料与仓库文件，因此模型资产取得并核对原 Git SHA，但该渠道不能完成当前 Python 依赖安装。

成功执行：

```bash
.venv/bin/python -m pip install --no-deps --no-build-isolation -e .
.venv/bin/python -m pip wheel --no-deps --no-build-isolation --wheel-dir dist .
```

项目本身安装、wheel构建成功，67个网格、XML、默认配置和许可进入wheel。另将wheel装入 `/tmp/panda-wheel-check`，从 `/tmp` 读取包资源和配置，验证与当前工作目录、旁边 references 无关。**这只证明打包与资源加载，不证明MuJoCo已编译模型。**

实际科学包版本见 [环境记录](environment_observed.txt)：NumPy1.26.4/SciPy1.15.2/PyYAML6.0.3/Matplotlib3.7.5/pytest9.1.1/CasADi3.6.7；原生OSQP动态库0.6.3。生产候选主要依赖已精确指定，完整传递锁未生成且干净安装未验证，不能称可复现安装验收完成。

## 测试与原生求解实验

```bash
.venv/bin/python -m pytest -q -ra
.venv/bin/python scripts/verify_native_osqp.py
.venv/bin/python -m compileall -q src scripts examples tests
.venv/bin/python scripts/check_docs.py
```

单元结果：**14 passed，7 skipped**，见 [测试输出](test_results.txt)。通过内容包括资产校验、力矩合成/限幅/过期、事件合并与历史、到达驻留逻辑、解析参考值/导数/内部极值/回退/停止、原QP代价等价性、故障注入拒解、失败计数及离线日志绘图。分析流水线测试的数据是明确标记的临时合成单元输入，不作机器人性能证据。

跳过的七项是真实引擎/求解器集成：motor映射与保持、mink独立状态、Python OSQP收敛/不可行、真实单目标到达、途中切换与回放、外力对照与恢复、不可达目标制动。未使用mock替代它们宣称通过。

另运行了真实 OSQP C0.6.3，经已有CasADi插件，使用生产 `MPCProblem` 构造的矩阵做四组理想双积分实验，每组400次实际QP。它不是生产 Python OSQP 接口验收，也不是机器人实验。七轴默认维度539、最终理想关节误差3.45e-9 rad、速度峰0.7 rad/s、加速度峰2rad/s²，最大约束违反3.43e-16；七轴求解包装时间P50≈1.01ms/P95≈12.29ms/最大≈23.11ms。原始数组、曲线、其他参数结果见 [实验](validation.md) 和 [原始摘要](evidence/native_osqp/summary.json)。

## 演示和分析入口

已实际执行：

```bash
.venv/bin/python -m panda_mpc.demo --scenario hold --headless --duration 10 --seed 42 --output outputs/blocked-hold
.venv/bin/python -m panda_mpc.analyze --run-dir outputs/blocked-hold
```

演示在导入 MuJoCo 时 `ModuleNotFoundError: No module named 'mujoco'`，退出1，保存metadata、解析配置、execution错误、空事件/ticks。分析正常读取失败日志并输出 `execution_completed:false`、`acceptance_passed:false`，不生成虚构物理曲线。其错误副本保存于 [blocked execution](evidence/blocked_execution.json)。

其他场景、四个真实教学脚本、GUI演示和文档中的全套安装命令均未成功运行；同一个核心依赖阻塞已明确定位，没有将它们标为通过。GUI显示/关闭/资源释放也未实际验证。

## 当前不能给出的指标

- 真实末端到达误差、保持误差、稳态偏差、外力恢复时间：未测。
- 真实速度/力矩峰值、饱和、QP失败、桥接/降级次数：未测，不填写0。
- 生产Python后端及整个机器人tick的P50/P95/最大/超预算、仿真/墙钟比：未测。
- 参数不匹配、负载偏差、多个固定seed的真实回归：未做。

## 补齐环境后的验收

```bash
PYTHON=python3.11 bash scripts/install.sh
.venv/bin/python scripts/acceptance.py
.venv/bin/python -m pytest -q -ra
```

install脚本依赖可用Python/ensurepip和包下载，成功后才生成完整 requirements.lock。acceptance脚本运行七场景和外力对照，既查退出码，也检查固定目标 reached/原阈值，以及显著外力偏移和恢复。任何失败都保留日志，不能通过调整验收阈值、缩小任务或重置状态规避。

先确认模型/API兼容与home稳定，再单目标到达，随后抢占/随机/快速切换/外力对照/GUI。重跑后应以真实新指标更新本记录和教学图，不保留当前“未测”作为永久结论。
