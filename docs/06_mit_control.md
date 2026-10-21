# 06：MIT 五参数、前馈和力矩执行

JointCommand 的 q_des/dq_des 来自同一解析参考曲线，kp/kd 为固定可配置七元素向量，tau_ff 来自独立名义模型，另含 stamp/valid_until/goal_id。接口表达关节阻抗语义，不是 Franka 原生五元组硬件协议。

每轴

$$\tau^{raw}=K_p(q_{ref}-q)+K_d(\dot q_{ref}-\dot q)+\tau_{ff},\quad\tau=\operatorname{clip}(\tau^{raw},\pm\tau_{max}).$$

kp 单位 Nm/rad，kd 单位 Nm·s/rad。反馈不放进质量矩阵。MITAdapter 是唯一 actuator ctrl 写入者，没有隐藏的第二套位置 PD。已测例子：误差 0.1 rad、速度差 0.2 rad/s、kp100、kd10、ff3，反馈分别10/2 Nm、raw15、限幅后12 Nm；这是接口单元测试，非机器人保持实验。

名义自由空间模型为 $M(q)\ddot q+h(q,\dot q)=\tau+\tau_{ext}$。本实现 inverse_dynamics 前馈取

$$\tau_{ff}=\hat M(q)\ddot q_{ref}+\hat qfrc_{bias}-\hat qfrc_{passive}.$$

通过独立 nominal MjModel/MjData 做 forward，以 mj_mulM 直接算矩阵向量乘，避免 mj_fullM 不同版本签名。没有补偿 plant 的约束力、外力或执行器力；无关节 frictionloss 时被动力主要为默认黏性阻尼，接触不是这里的 h。

none 模式 ff=0，gravity 模式在 nominal 零速度状态读取 qfrc_bias，inverse_dynamics 为默认惯性+偏置-被动项。修改 nominal 数据不会将真实速度清零。未来负载偏差实验应只改 plant，不得偷读改变后的质量获得完美前馈。

理想名义模型精确、无饱和、无外力且连续执行时，令 $e=q_{ref}-q$，代入力矩律并消去 h：

$$M(q)(\ddot q-\ddot q_{ref})=K_pe+K_d\dot e,$$
$$M(q)\ddot e+K_d\dot e+K_pe=0.$$

这只是在理想假设下的误差关系，不能据此声称变惯量、采样、桥接加速度跳变、接触和任意模型误差下全局稳定。局部单轴调参近似 $k_d\approx2\zeta\sqrt{k_pJ_{eff}}$ 依赖局部等效惯量，并非七轴耦合系统严格设计。

默认 kp=[180,180,160,140,60,50,30]、kd=[30,30,25,22,10,8,6] 是保守的待验证起点：近端较大惯量使用较大刚度和阻尼，远端降低增益。当前未完成真实保持和饱和验证，不能称已调好。不可靠扩大力矩界掩盖不稳定。

例子：`python examples/03_mit_step.py --feedforward inverse_dynamics`，分别改 gravity/none，比较相同解析小幅参考。当前因缺依赖未运行。常见错误是把 `M@(kp*e+kd*de)` 当本项目 MIT、把接触力加入 h 精确抵消实验外力。练习：关闭前馈并记录稳态偏差和饱和，勿预先填结果。

Franka libfranka 提供关节位置、速度、力矩控制接口，本项目只是 Panda 动力学 + MIT 语义适配器，无 CAN、硬件驱动或可直接上真机的安全保证。
