# ZERO 机械臂项目全流程说明 v3

本文档面向“准备接手并修改这个项目”的开发者，目标不是简单告诉你怎么运行，而是把这个仓库从机械结构、运动学、嵌入式控制、MuJoCo 仿真到强化学习训练的链路讲清楚，让你能够安全地修改模型、环境和训练参数。

## 1. 项目整体定位

这个仓库不是一个单纯的 RL Demo，而是一条完整的机械臂开发链路：

1. `1. Model`：SolidWorks 结构模型、STEP 文件、装配参考图。
2. `2. Software/robot`：STM32F407 嵌入式控制代码，包含逆运动学、串口/MQTT 命令、关节控制。
3. `3. Simulink`：D-H 参数、符号推导、URDF、Simscape/Simulink 验证。
4. `5. Deep_LR`：MuJoCo 仿真环境和基于 Stable-Baselines3 的 TD3 强化学习训练代码。

也就是说，这个项目的核心不是“从零设计一个 RL 算法”，而是“基于真实机械臂结构，构建一个可仿真、可控制、可训练的完整系统”。

## 2. 仓库结构与作用

### 2.1 根目录

- `README.md`
  原作者说明，包含项目总览和基本训练/测试命令。
- `BOM.xlsx`
  物料清单。
- `LICENSE`
  开源许可证。

### 2.2 机械结构相关

- `1. Model/ZERO_ARM.SLDASM`
  SolidWorks 总装配模型。
- `1. Model/STEP/`
  STEP 导出文件，便于无 SolidWorks 环境查看或二次转换。
- `1. Model/3d_printing_all.3mf`
  3D 打印整包文件。
- `1. Model/installation_guide.md`
  装配图片说明。

### 2.3 嵌入式控制相关

- `2. Software/robot/Core/Src/robot.c`
  实际机器人控制框架，包含 D-H 参数、复位姿态、关节初始化参数、路径插补、PID 等。
- `2. Software/robot/Core/Src/robot_kinematics.c`
  逆运动学实现。
- `2. Software/robot/Core/Src/robot_cmd.c`
  串口与 MQTT 命令入口。

### 2.4 运动学/仿真相关

- `3. Simulink/robot_kinematics_sym_v3_0.m`
  根据 D-H 参数做符号推导，得到逆解公式。
- `3. Simulink/robot_run.m`
  启动 Simulink 模型的入口脚本。
- `3. Simulink/URDF_XG_Robot_Arm_Urdf_V1_1/`
  URDF 和 mesh 资源。
- `3. Simulink/URDF_XG_Robot_Arm_Urdf_Control_V3.slx`
  Simscape/Simulink 模型。

### 2.5 强化学习相关

- `5. Deep_LR/robot_arm_mujoco.xml`
  MuJoCo 模型定义。
- `5. Deep_LR/robot_arm_env.py`
  Gymnasium 环境定义，包含状态、动作、奖励、终止条件、可视化。
- `5. Deep_LR/train_robot_arm.py`
  TD3 训练与测试入口。
- `5. Deep_LR/logs/best_model/`
  已训练好的最佳模型与 `VecNormalize` 参数。

## 3. 从机械模型到强化学习的完整链路

你可以把整个项目理解为 5 个层次，后面的层依赖前面的层。

### 3.1 第 1 层：机械结构建模

起点是 `1. Model` 里的 SolidWorks 模型。这个层负责确定：

- 连杆长度
- 关节数量
- 关节相对安装方向
- 关节运动范围
- 质量、惯量、几何外形

这些参数最终会影响：

- 运动学 D-H 参数
- URDF / MuJoCo 中的连杆和关节定义
- 真实控制器的限位和零位

如果你改了机械结构，例如：

- 连杆长度变了
- 第 5 关节安装角度变了
- 夹具重量变了

那么不能只改一个地方，后面 Simulink、MuJoCo、嵌入式控制都要同步。

### 3.2 第 2 层：运动学建模与验证

这一层主要在 `3. Simulink` 和 MCU 代码中体现。

#### D-H 参数

在 `3. Simulink/robot_run.m` 中定义了 6 关节 D-H 参数：

```matlab
D_H = [
    0,    0,   0,   pi/2;
    0,    pi/2,    0,   pi/2;
    200,  pi,    0,  -pi/2;
    47.63, -pi/2, -184.5,  0;
    0,    pi/2,    0,   pi/2;
    0,    pi/2,  0,    0
];
```

在 `2. Software/robot/Core/Src/robot.c` 中也手工写了一份同样的 D-H 参数。

这说明：

1. Simulink 和 MCU 都基于同一套运动学结构。
2. 参数同步是手工的，不是自动生成。
3. 一旦你修改结构，至少要同步修改这两处。

#### 单位

这里有一个很重要的点：

- `Simulink` / MCU 中的 D-H 参数用的是毫米量级，例如 `200`、`47.63`、`-184.5`
- `MuJoCo XML` 中使用的是米，例如 `0.2`、`0.047631`、`-0.1845`

如果你修改结构参数，必须同时注意单位换算，否则仿真和实际控制会严重偏离。

#### 符号推导

`3. Simulink/robot_kinematics_sym_v3_0.m` 做的是逆运动学公式推导，思路是：

1. 根据 D-H 构造各关节齐次变换矩阵。
2. 通过目标末端位姿 `T_target` 推导 `theta1 ~ theta6`。
3. 再把这些公式移植到 `robot_kinematics.c`。

因此 `robot_kinematics.c` 本质上是 `robot_kinematics_sym_v3_0.m` 的工程化落地版本。

### 3.3 第 3 层：真实机器人控制

真实控制代码在 `2. Software/robot`。

几个关键点：

- `robot.c`
  包含关节初始配置、减速比、限位、零位姿态矩阵 `T_0_6_reset`。
- `robot_kinematics.c`
  提供逆运动学求解。
- `robot_cmd.c`
  提供对外命令接口。

当前支持的关键命令：

- `rel_rotate joint_id angle`
  关节相对旋转。
- `hard_reset`
  硬复位，触发限位开关。
- `soft_reset`
  回到零位。
- `zero`
  当前姿态设为零点。
- `auto x y z`
  自动移动到指定空间位置。

这说明真实机器人的控制接口也是围绕“末端位置控制”来设计的，这与 RL 任务目标一致。

### 3.4 第 4 层：MuJoCo 仿真模型

RL 不直接使用 URDF，而是使用 `5. Deep_LR/robot_arm_mujoco.xml`。

这个 XML 定义了：

- 6 个关节 `joint1 ~ joint6`
- 末端 site：`ee_site`
- 6 个电机 actuator：`joint1_ctrl ~ joint6_ctrl`
- 碰撞排除：相邻连杆之间不碰撞
- 相机：前视角、侧视角、末端相机
- 末端位置和速度相关传感器

关键结构信息：

- 关节 2 有范围 `[-1.5708, 1.5708]`
- 关节 3 有范围 `[0, 3.1416]`
- 关节 5 有范围 `[-1.5708, 0]`
- actuator 目前采用 `motor`，即更接近力矩/驱动力控制

文件中还保留了注释掉的 `position actuator` 版本，这很重要，因为它意味着你后续可以把控制方式从“力矩控制”切换为“位置控制”。

### 3.5 第 5 层：Gym 环境与 TD3 训练

这一层对应 `robot_arm_env.py` 和 `train_robot_arm.py`，是整个 RL 的核心。

## 4. 强化学习环境设计

`RobotArmEnv` 继承自 `gym.Env`，底层调用 MuJoCo。

### 4.1 动作空间

动作空间定义为：

```python
spaces.Box(low=-15.0, high=15.0, shape=(self.nu,), dtype=np.float32)
```

这里 `self.nu = 6`，所以动作是一个 6 维向量，对应 6 个关节控制量。

当前语义：

- 不是目标角度
- 不是角速度
- 更接近关节控制输入/扭矩

如果你改成 XML 中注释掉的 `position actuator`，那么这 6 维动作的语义就会变化，奖励和训练难度也会变化。

### 4.2 状态空间

状态空间是 24 维：

1. 末端到目标点的相对位置 `3`
2. 关节角 `6`
3. 关节速度 `6`
4. 上一时刻控制量 `6`
5. 末端线速度 `3`

拼接顺序在 `robot_arm_env.py` 的 `_get_state()` 中。

这是一个非常典型的“末端追踪”状态设计。它让策略同时看到：

- 我离目标还有多远
- 当前关节姿态是什么
- 正在往哪个方向运动
- 上一步施加了什么控制

如果你后续新增观测，必须同步修改：

1. `_get_state()`
2. `state_dim`
3. 训练好的模型不能直接复用，需要重新训练

### 4.3 目标点采样范围

环境每次 `reset()` 时会随机采样目标点：

- `x` in `[-0.2, 0.2]`
- `y` in `[-0.37, -0.17]`
- `z` in `[0.2, 0.4]`

这定义了训练任务的工作空间。

如果你觉得模型只能在一个很小范围内学会跟踪，可以：

- 先缩小范围，降低任务难度
- 等模型稳定后再逐步放大

这是最常见、也最有效的 curriculum 思路之一。

### 4.4 奖励函数拆解

当前奖励由多部分组成。

#### 1. 步数惩罚

每步固定减 `0.1`，鼓励更快完成任务。

#### 2. 距离改进奖励

- 如果距离比历史最优更小，给正奖励
- 如果距离比前一步更大，给负奖励

这类 shaping 奖励的作用是让策略在早期训练时也能获得连续反馈，而不是只有最终成功时才拿到奖励。

#### 3. 基础距离惩罚

当前使用：

```python
base_distance_penalty = -distance ** 0.5 * 0.8
```

这比线性惩罚更温和，离目标远时梯度不会太夸张。

#### 4. 分阶段接近奖励

阈值：

```python
[0.5, 0.3, 0.1, 0.05, 0.01, 0.005, 0.002]
```

奖励：

```python
[100, 200, 300, 500, 1000, 1500, 2000]
```

每个阈值只奖励一次。

这类设计适合“逐步逼近目标”的任务，但副作用是奖励尺度较大，可能压过其他项。

#### 5. 运动方向奖励

计算当前末端速度方向与“指向目标方向”的夹角余弦：

- 同方向运动会加分
- 反方向不加分

这可以减少乱晃。

#### 6. 速度惩罚

末端速度过大时减 `0.2`，抑制暴力动作。

#### 7. 关节速度变化惩罚

对相邻时刻关节速度变化做惩罚，减少抖动和失稳。

#### 8. 碰撞惩罚

只要检测到接触，就每个 contact 扣 `5000`，并终止 episode。

这是一个非常强的约束。

#### 9. 成功奖励

如果末端距离目标小于 `success_threshold`，则：

- 给 `10000` 成功奖励
- 给剩余步数奖励
- 若成功时速度更低，再加额外奖励

注意一个实际细节：

- 代码里 `success_threshold = 0.01`
- 注释写的是“1mm”
- 但 `0.01 m` 实际上是 `10 mm = 1 cm`

也就是说，目前代码的成功阈值比注释宽 10 倍。

### 4.5 终止条件

episode 终止有三种情况：

1. 到达目标
2. 发生碰撞
3. 步数达到上限 `3000`

如果你发现策略总是通过“撞一下结束”来逃避负奖励，那么应该先检查碰撞惩罚和提前终止逻辑，而不是只调学习率。

### 4.6 可视化

测试模式会创建 MuJoCo passive viewer，并在目标点位置画一个绿色小球。

这部分逻辑在 `render()` 和 `_add_target_visualization()` 中。

测试循环里还有：

```python
time.sleep(0.01)
```

这会故意放慢显示速度，便于观察。如果你只想快速评估，不想看动画，可以删掉或调小。

## 5. TD3 训练架构

训练入口是 `5. Deep_LR/train_robot_arm.py`。

### 5.1 训练框架

当前使用：

- `Gymnasium`
- `MuJoCo`
- `Stable-Baselines3`
- 算法：`TD3`

这是一套标准、稳定的连续控制方案。

### 5.2 训练数据流

训练过程大致如下：

1. 创建 `RobotArmEnv`
2. 用 `make_vec_env(..., n_envs=1)` 包装
3. 用 `VecNormalize` 对 observation 和 reward 做归一化
4. 创建 TD3 模型
5. 周期性评估并保存最佳模型
6. 训练结束后保存最终模型和归一化参数

### 5.3 策略网络结构

Actor / Critic 都使用：

```python
[512, 512, 256]
```

激活函数是 `ReLU`。

这比很多教学例子里的 `[256, 256]` 更大，说明作者倾向于用更强的拟合能力去覆盖复杂奖励和动力学。

### 5.4 探索噪声

动作噪声：

```python
NormalActionNoise(mean=0, sigma=2.5)
```

这是 TD3 在连续动作空间里常用的探索方式。

如果你觉得策略训练初期完全不探索，可以增大 `sigma`。
如果你发现动作抖动太强、训练不稳定，可以适当减小。

### 5.5 关键超参数

当前训练超参数如下：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `learning_rate` | `3e-4` | 优化步长 |
| `buffer_size` | `3000000` | 经验回放容量 |
| `learning_starts` | `10000` | 前 10000 步只收集数据不学习 |
| `batch_size` | `256` | 每次更新采样的 batch 大小 |
| `tau` | `0.005` | 目标网络软更新系数 |
| `gamma` | `0.99` | 折扣因子 |
| `train_freq` | `1` | 每步训练一次 |
| `gradient_steps` | `1` | 每次采样后做一次梯度更新 |
| `policy_delay` | `4` | Actor 更新延迟 |
| `target_policy_noise` | `0.2` | 目标策略平滑噪声 |
| `target_noise_clip` | `0.5` | 目标噪声截断 |
| `total_timesteps` | `5000000` | 总训练步数 |
| `eval_freq` | `5000` | 每 5000 步评估一次 |

### 5.6 归一化与模型保存

训练时使用：

```python
VecNormalize(env, norm_obs=True, norm_reward=True)
```

测试时使用：

```python
VecNormalize.load(...)
env.training = False
env.norm_reward = False
```

这意味着：

1. 测试时必须加载训练对应的 `vec_normalize.pkl`
2. 如果你忘了加载归一化参数，模型表现通常会明显变差

### 5.7 回调函数

当前训练脚本里有 3 个关键 callback：

#### `EvalCallback`

- 定期评估模型
- 把最佳模型保存到 `./logs/best_model`

#### `SaveVecNormalizeCallback`

- 当出现新的最佳模型时
- 同时保存与之匹配的 `VecNormalize` 参数

#### `ManualInterruptCallback`

- 支持 `Ctrl+C` 中断训练
- 中断时保存当前模型到 `./models/interrupted/`

这三个组件构成了一个比较完整的训练闭环。

## 6. 当前项目的“真实默认配置”

为了避免你被 README 和代码不一致的地方误导，这里列几个当前仓库的实际情况。

### 6.1 文件夹名

README 中写的是 `Deep_RL`，但仓库实际目录名是：

```text
5. Deep_LR
```

脚本运行时应以实际目录为准。

### 6.2 已有预训练模型

当前仓库已包含：

- `5. Deep_LR/logs/best_model/best_model.zip`
- `5. Deep_LR/logs/best_model/vec_normalize.pkl`

所以你不需要先重新训练，就可以直接测试效果。

### 6.3 MuJoCo XML 资源

当前 XML 已改为内置 skybox，不再依赖丢失的外部贴图文件。

### 6.4 XML 加载路径

环境代码现在按脚本所在目录加载 `robot_arm_mujoco.xml`，不会因为你从仓库根目录启动还是从 `5. Deep_LR` 启动而找不到文件。

## 7. 如何修改项目：按需求拆分

这部分最重要。你以后改项目时，最好先判断你改的是哪一层。

### 7.1 想改机械臂长度、姿态、关节结构

需要同步检查的文件：

1. `1. Model/` 中的 CAD 模型
2. `3. Simulink/robot_run.m` 中的 D-H 参数
3. `2. Software/robot/Core/Src/robot.c` 中的 D-H 参数和零位姿态
4. `2. Software/robot/Core/Src/robot_kinematics.c` 是否仍与符号推导一致
5. `5. Deep_LR/robot_arm_mujoco.xml` 中 body/joint 的 `pos`、`quat`、`range`

推荐顺序：

1. 先改 CAD
2. 再改 D-H
3. 用 Simulink 验证
4. 再改 MuJoCo
5. 最后重新训练 RL

### 7.2 想改目标工作空间

改 `robot_arm_env.py` 的 `reset()` 即可，重点看 `self.target_pos` 的随机采样范围。

这适合做的事：

- 把训练范围缩小，先让策略学会
- 只训练某个象限
- 固定高度，只做平面跟踪
- 做课程学习

### 7.3 想改控制方式

当前是 motor actuator，动作更接近力矩控制。

如果你想改成位置控制，可以参考 XML 里已经注释掉的 `position actuator` 段。

改完后你必须同步检查：

1. 动作范围是否仍合理
2. 奖励函数是否还适合
3. 探索噪声是否过大
4. 策略是否更容易训练

通常：

- 力矩控制更灵活，但更难训练
- 位置控制更稳，但可能不够真实

### 7.4 想改奖励函数

直接改 `robot_arm_env.py` 的 `step()`。

最常改的项：

- `success_threshold`
- `phase_thresholds`
- `phase_rewards`
- `collision_penalty`
- `joint_velocity_change_penalty`
- `setp_penalty`

建议：

- 一次只改 1 到 2 个核心项
- 每次改完记录结果
- 不要同时大改奖励和网络结构，否则很难定位问题来源

### 7.5 想改观测空间

如果你想加入更多状态，例如：

- 目标点绝对坐标
- 末端姿态
- 每个关节的力矩反馈
- 历史动作序列

需要同步改：

1. `_get_state()`
2. `state_dim`
3. 旧模型作废，重新训练

### 7.6 想改训练效率或收敛速度

优先改这些：

- `learning_rate`
- `batch_size`
- `sigma`
- `total_timesteps`
- `eval_freq`
- 网络结构 `pi` / `qf`

经验上：

- 训练不稳定：先减小探索噪声和奖励尺度
- 收敛太慢：先缩小任务空间，再考虑调网络或学习率
- 成功率不高：优先检查奖励和终止条件，而不是盲目加大模型

### 7.7 想改成更多关节或更少关节

这是中等偏大的修改，因为会影响全链路。

需要同步改：

1. MuJoCo XML 的 joints、actuators、sensor
2. `robot_arm_env.py` 的动作维度和状态拼接
3. Simulink / D-H 参数
4. MCU 端 `ROBOT_MAX_JOINT_NUM` 相关逻辑

不要只改 MuJoCo，不然后面全部会错位。

## 8. 推荐的二次开发方法

如果你的目标是“在不把系统搞乱的前提下，逐步做出自己的版本”，建议按下面顺序。

### 路线 A：先改 RL，不动结构

适合先熟悉项目。

1. 保持 MuJoCo 结构不变
2. 只调目标点范围、奖励函数、训练超参数
3. 验证能否稳定提升跟踪效果

### 路线 B：改控制方式

适合对 RL 控制形式有明确想法的人。

1. 把 actuator 从 motor 改为 position 或 velocity 风格
2. 重新定义 action space
3. 重新调奖励函数
4. 重新训练

### 路线 C：改机械臂结构

这是最完整但也最容易出错的路线。

1. 改 CAD
2. 改 D-H
3. 验证逆运动学
4. 改 MuJoCo
5. 最后再训练 RL

## 9. 快速运行与验证

建议在新的 conda 环境中运行，不要直接污染 `base`。

### 9.1 环境安装

```powershell
conda create -n zeroarm-rl python=3.10 -y
conda activate zeroarm-rl
python -m pip install --upgrade pip
python -m pip install numpy stable-baselines3 mujoco glfw pyopengl
```

### 9.2 进入 RL 目录

```powershell
cd "f:\desktop_data_store\a_XinMa_lab\2026_FD_FlexibleDrill_System\04_Software_Control\OpenResource\zero-robotic-arm-master\5. Deep_LR"
```

### 9.3 最小自检

```powershell
python -c "from robot_arm_env import RobotArmEnv; env=RobotArmEnv(); print(env.action_space.shape, env.observation_space.shape)"
```

### 9.4 测试预训练模型

```powershell
$env:MUJOCO_GL="glfw"
python .\train_robot_arm.py --test --model-path .\logs\best_model\best_model.zip --normalize-path .\logs\best_model\vec_normalize.pkl --episodes 5
```

### 9.5 从头训练

```powershell
python .\train_robot_arm.py
```

## 10. 修改时的同步检查清单

每次大改前，建议先过一遍这个清单。

- 我改的是结构、控制、奖励，还是训练参数？
- 单位是否一致：毫米还是米？
- D-H、MuJoCo、MCU 三处是否同步？
- 状态维度改了没有？
- 动作语义改了没有？
- 是否需要重新训练旧模型？
- `VecNormalize` 是否和模型匹配？
- 测试时是否加载了正确的 `best_model.zip` 和 `vec_normalize.pkl`？

## 11. 一句话理解这个项目

这个项目的本质是：

“先把真实机械臂的结构、运动学和控制链路搭起来，再在 MuJoCo 中把它抽象成一个 6 自由度末端目标跟踪任务，用 TD3 学一个连续控制策略。”

如果你以后要做自己的版本，最重要的不是一上来改算法，而是先分清楚每一层在系统里的职责，以及改动是否跨层传播。
