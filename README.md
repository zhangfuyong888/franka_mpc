# Panda 在线目标跟踪：IK + 关节空间线性 MPC + MIT 关节阻抗仿真

**当前状态：真实 headless 闭环已验证。** 当前环境已安装 MuJoCo3.3.7、mink0.0.13、OSQP1.0.4。`pytest -q -W error` 的22项测试全部通过，七种场景和扰动对照已运行。单目标最终误差约0.86mm，默认运行没有QP失败、饱和或参考回退。GUI自动验证仍受工具环境无法连接X11限制；完整证据见 [validation](docs/validation.md)。

控制路线为IK → 关节空间加速度MPC → 从上一参考边界连续积分第一优化输入 → MIT执行适配器。保留解析整段检查与制动，不重置实际状态。模型、67个网格和许可随包安装。警告来源及连续参考修复见 [连续参考](docs/03_online_trajectory.md)。

## 安装

优先 Python 3.11；本机只有可用的 3.10，本次实际运行采用 3.10.20。请在项目目录执行：

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pip freeze --all > requirements.lock
python -m pip check
```

`pyproject.toml` 固定了主要依赖版本；**完整传递依赖锁尚未生成和验证**。不要把 `docs/environment_observed.txt` 当作安装锁，它记录本机借用环境的已安装包。本机已完成真实集成测试；干净环境安装与传递锁仍需独立验证。

本机为继续离线工作，已通过 `/home/zfy/miniconda3/envs/tv/bin/python` 创建允许读取该环境包的项目 `.venv`，只在项目内写入：

```bash
.venv/bin/python -m pip install --no-deps --no-build-isolation -e .
.venv/bin/python -m pytest -q -ra
.venv/bin/python scripts/verify_native_osqp.py
```

第一条只安装项目本身，不能补齐仿真依赖。模型已包含在包中；启动不下载模型，不依赖邻近参考目录。派生流程见 `scripts/prepare_model.py`。

## 运行

```bash
python -m panda_mpc.demo --scenario hold --headless --duration 10 --seed 42
python -m panda_mpc.demo --scenario single_target --headless --duration 10 --seed 42
python -m panda_mpc.demo --scenario switch --headless --duration 12 --seed 42
python -m panda_mpc.demo --scenario random --headless --duration 30 --seed 42
python -m panda_mpc.demo --scenario rapid_switch --headless --duration 10 --seed 42
python -m panda_mpc.demo --scenario invalid_goal --headless --duration 8 --seed 42
python -m panda_mpc.demo --scenario disturbance --headless --duration 12 --seed 42 --output outputs/push
python -m panda_mpc.demo --scenario disturbance --headless --duration 12 --seed 42 --no-disturbance --output outputs/control
python -m panda_mpc.analyze --run-dir outputs/push --compare outputs/control
python -m panda_mpc.demo --scenario random --seed 42
python -m panda_mpc.demo --config configs/default.yaml --headless --output outputs/manual-run
python -m panda_mpc.analyze --run-dir outputs/manual-run
python -m panda_mpc.demo --scenario switch --events configs/switch_events.json --headless --output outputs/replay
python -m pytest -q
```

输出目录必须尚不存在，避免覆盖已有实验。默认自动创建带时间戳的目录。退出码：0 正常结束，2 出现明确拒绝目标但运行正常，1 非预期失败或监测停止，130 手动中断。GUI 显示最新目标、实际轨迹与逐点 FK 计算的预测曲线，终端显示编号、误差和耗时。headless 不导入 viewer、不创建 Renderer。

`--duration 10` 表示仿真10秒后自动结束并关闭窗口；希望看更久可改为30或60。退出0表示程序正常结束，是否达到目标请同时查看终端的Final goal和summary中的fixed_goal_acceptance。

结果包含解析配置、版本和资源哈希、完整目标生命周期/扰动/监测事件、逐步 NPZ、MPC ticks、summary 和 plots。缺依赖的启动同样保存错误；没有物理数据时不生成伪造曲线。

## 阅读顺序

[总览](docs/00_overview.md) → [模型](docs/01_mujoco_model.md) → [IK](docs/02_kinematics_ik.md) → [在线轨迹](docs/03_online_trajectory.md) → [MPC](docs/04_mpc_theory.md) → [稀疏 QP](docs/05_osqp_implementation.md) → [MIT](docs/06_mit_control.md) → [闭环时序](docs/07_closed_loop.md) → [实验](docs/08_experiments.md) → [源码导读](docs/09_code_walkthrough.md)。

符号见 [notation](docs/notation.md)，实际证据见 [validation](docs/validation.md)，第三方来源见 [references](docs/references.md) 和 [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES.md)。

## 故障排查

- `No module named mujoco/mink/osqp`：核心依赖没有安装，不是控制算法失败。查看 `docs/install_attempt.log`。
- 系统 Python 无 `ensurepip`：使用已有含 ensurepip 的 Python，或者在用户环境安装 Python；本项目不修改全局环境。
- 已有输出目录：使用新的 `--output`；项目不会覆盖。
- `No legal C1 segment`：查看上一参考边界、桥接系数、整个区间加速度；不能逐元素裁剪参考掩盖问题。
- QP 不可行：检查真实初值和制动距离；不能裁剪实测状态。
- 图形库失败：先用 `--headless`。GUI 验证仍是未完成项。

本项目只做仿真，不提供可直接上真机的通信或安全保证。
