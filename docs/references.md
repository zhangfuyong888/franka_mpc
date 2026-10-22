# 参考来源、实际版本和使用范围

工作区初始只有 CODEX_TASK_MPC_MIT_V2.md，没有 references 或已有项目，也没有适用 AGENTS.md。本实现新增唯一项目目录 panda-mpc-tracking，未修改任何参考仓库或全局 Git/Python 设置。终端访问 GitHub 失败；模型和参考文本经 GitHub 连接器读取，网页资料经浏览工具核对。

| 来源 | 实际引用版本 | 使用 |
|---|---|---|
| [Menagerie Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/cda5236f98383f6fbaa4820c6b18dba892cd087f/franka_emika_panda) | cda5236f98383f6fbaa4820c6b18dba892cd087f | README/panda.xml/scene.xml/LICENSE 已读，复制必要67网格与许可 |
| [mjctrl](https://github.com/kevinzakka/mjctrl/tree/f1c82c257248e2e52b0e6c1beaf23c6a2026e4f2) | f1c82c257248e2e52b0e6c1beaf23c6a2026e4f2 | diffik_nullspace/opspace 已读，借鉴Jacobian/控制边界，不照搬执行器 |
| [mink](https://github.com/kevinzakka/mink/tree/47088abce59efcd54ffe69423c1df3748775f785) | v0.0.13 / 47088abce59efcd54ffe69423c1df3748775f785 | Configuration/FrameTask/PostureTask/两类Limit/solve_ik/pyproject已读 |
| [mink Panda example](https://github.com/kevinzakka/mink/blob/main/examples/arm_panda.py) | main，在线阅读快照 | 选定旧tag没有该路径，改读main对应例子；不把main行为当安装版API |
| [MuJoCo仿真流程](https://mujoco.readthedocs.io/en/3.3.7/programming/simulation.html) | 3.3.7 | split step、状态与回调 |
| [MuJoCo引擎源码](https://github.com/google-deepmind/mujoco/blob/3.3.7/src/engine/engine_forward.c) | 3.3.7 | step2积分和warning/reset |
| [MuJoCo动力学](https://mujoco.readthedocs.io/en/stable/computation/index.html) | 文档入口 | bias/passive/外力符号 |
| [OSQP Python](https://osqp.org/docs/interfaces/python.html) | 固定依赖 1.0.4，已有真实求解记录 | CSC/update/warm start/残差字段 |
| [OSQP MPC](https://osqp.org/docs/examples/mpc.html) | 官方示例入口 | 稀疏状态输入堆叠思路 |
| [libfranka Robot](https://frankarobotics.github.io/libfranka/latest/classfranka_1_1Robot.html) | 阅读时0.21.3 | 确认位置/速度/力矩接口，未安装硬件依赖 |

MIT历史 Mbed URL（任务给定）读取时重定向至 ArmMbed GitHub 主页，未获得对应版本源文件，因此不声称已核验其具体实现。本项目五参数语义按用户任务给定公式实现和单元测试，不能用不可访问的历史链接作硬件协议来源证明。

## 版本状态

固定依赖：Python3.10~3.12（优先3.11）、mujoco3.3.7、mink0.0.13、osqp1.0.4、qpsolvers4.8.1、numpy1.26.4、scipy1.15.2、PyYAML6.0.3、matplotlib3.10.7、pytest8.4.2。版本号固定在 pyproject，当前API和真实闭环已验证，干净环境完整传递依赖锁仍未验证。

最初离线验证环境（历史）：Python3.10.20、NumPy1.26.4、SciPy1.15.2、PyYAML6.0.3、Matplotlib3.7.5、pytest9.1.1、CasADi3.6.7，其动态链接 OSQP C0.6.3。初次验证时生产所需mujoco/mink/osqp Python未安装；当前均已补齐并完成真实测试，版本见 evidence/closed_loop/metadata.json。`environment_observed.txt` 是借用环境记录，非锁文件。

## mink推导约定

Configuration.get_frame_jacobian 将 MuJoCo world 对齐 Jacobian 旋转至 body 系。FrameTask.compute_error 使用目标相对于当前的 SE(3) log，compute_jacobian 为负号加 jlog 修正。solve_ik 的最终实际返回是 delta_q/dt；旧版 build_ik 文档注释存在反向换算笔误。讲解必须跟实际源代码，见 [IK章节](control_math.md)。
