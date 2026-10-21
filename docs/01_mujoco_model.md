# 01：Panda 模型、直接力矩与物理状态

原始 Menagerie 的 general actuator 含位置伺服偏置，不能直接把 ctrl 当力矩。项目保留 upstream.xml，用 `scripts/prepare_model.py` 派生 panda.xml；来源 commit 和网格 Git SHA 在随包 provenance.json。

转换删除 actuator/tendon/equality，清除 default 中的 general。七个 motor gear=1、无 activation、无反馈 bias。模型 ctrlrange/forcerange 为前四轴 ±87 Nm、后三轴 ±12 Nm，来自原模型，不是硬件认证声明；配置进一步限为 ±70/10 Nm。

手指保留视觉、碰撞、质量、惯量，去掉滑动 DOF。左/右 body 平移 y=±0.03 m，右侧原旋转保留，开度共 0.06 m。ee_site 在 hand 局部 `[0,0,0.1034]`，IK/FK、判定、日志、显示统一使用它。

Robot 按 joint1..7 查 qpos 地址、DOF 地址和 motor transmission，并检查 nq=nv=nu=7、坐标对齐。不能在一般 MuJoCo 模型上假设 joint ID 等于 DOF 地址。home 为 `[0,-0.6,0,-2,0,1.5,0.785398]`；只有初始化 reset。独立 MjData 用于 IK、预测 FK 与名义前馈。

时序为 `mj_step1 → 读取派生状态/控制 → motor ctrl → 外力 → mj_step2`。模型选择 implicitfast，禁用 RK4 拆分步假设；已检查 MuJoCo3.3.7 engine_forward.c：mj_step2 对 implicit/implicitfast 调用 mj_implicit，其他设置走 Euler。该版本文档页仍有旧 Euler 描述，以固定版本源码为准；实际MuJoCo3.3.7运行已通过保持和闭环测试。没有旧 position servo 或 body_gravcomp 隐含补偿。

运行 `python -m pytest -q tests/test_integration.py::test_torque_model_and_home_hold` 会检查零输入 qfrc_actuator=0、已知输入直接力矩映射，再做真实保持。目前上述测试已实际通过，模型编译、直接力矩映射和两秒保持已验证；静态检查同时保留67个网格SHA与固定手指位姿验证。

错误定位：重力补偿后漂移，查顺序、符号、遗漏惯量及旧 servo。练习：解释修改前后最后轴 ctrlrange 的 rad/Nm 含义差异。
