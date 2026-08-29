# wheel_leg_analysis

这是对
[`Skythinker616/foc-wheel-legged-robot`](https://github.com/Skythinker616/foc-wheel-legged-robot)
五连杆腿部数学模型的独立 Python 复现与机械特性分析。参考版本固定为上游
提交 `e2444395dd3a76c20b0683fbb1e123c21186a502`。工程目前包含第一阶段的
数学/C 对照验证、第二阶段的原结构全局工作空间分析、第三阶段的正常
装配模式连续伸缩研究、真实机械包络验证、EduLite 主动关节最小补丁、执行器
公开能力初筛、一维蹬伸/落地数量级筛选，以及 MuJoCo 真实 CAD 可视运动学
检查点；不涉及杆长优化、完整越阶接触动力学、强化学习或 ROS。

## 验证结论

验证通过。Python 与原 MATLAB 公式、MATLAB Coder 生成的三个单精度 C
函数在合法且远离闭环奇异边界的 406 组测试数据上数值一致。所有 C/Python
输出均为有限值，没有超过回归阈值的失败点。

Python 使用 `float64`，原 C 接口使用 `float32`。所有角度、速度、力和力矩
输入先量化为同一个 `float32` 值，再分别交给 Python 和 C，以下是实际绝对
误差，不是仅判断“能否运行”。

| 输出 | 单位 | 最大绝对误差 | 平均绝对误差 | 测试阈值 |
|---|---:|---:|---:|---:|
| `l0` | m | 1.5483113736e-07 | 6.6593644333e-09 | 2.0e-07 |
| `phi0` | rad | 4.2891631316e-06 | 1.1455068755e-07 | 5.0e-06 |
| `dL` | m/s | 8.7192901828e-05 | 2.6978911863e-07 | 1.0e-04 |
| `dPhi` | rad/s | 2.2216659435e-03 | 6.8067460022e-06 | 3.0e-03 |
| `T1` | N m | 2.3053082900e-03 | 8.2771319044e-06 | 3.0e-03 |
| `T2` | N m | 1.2903793645e-03 | 4.7409576230e-06 | 2.0e-03 |

“明显差异”采用比通过阈值更严格的观察阈值，用于暴露单精度敏感点，不代表
测试失败。共记录 4 点：

| # | `(phi1, phi4)` rad | `(dphi1, dphi4)` rad/s | `(F, Tp)` | 超过观察阈值的误差 |
|---:|---|---|---|---|
| 3 | `(1.086452961, -3.035454750)` | `(0.350000, -0.150000)` | `(33.000000, 0.100000)` | `l0=1.548e-07`, `phi0=4.289e-06`, `dPhi=1.159e-04`, `T1=9.085e-04`, `T2=5.079e-04` |
| 122 | `(1.272320986, 3.083838940)` | `(1.833285, 4.116742)` | `(23.170074, -1.942834)` | `phi0=1.003e-06` |
| 186 | `(1.086452961, -3.035454750)` | `(-2.602058, -4.616449)` | `(-66.085556, -0.948967)` | `l0=1.548e-07`, `phi0=4.289e-06`, `dL=8.719e-05`, `dPhi=2.222e-03`, `T1=2.305e-03`, `T2=1.290e-03` |
| 304 | `(0.549493432, -2.319670439)` | `(7.335450, 0.732832)` | `(-71.891205, -1.160547)` | `dPhi=1.328e-04` |

最差姿态的闭环判别式仍为正，属于数学合法位形；但 MATLAB 半角公式中的
`A0 + C0 = -1.061958e-04` 接近零，导致原 `float32` C 展开式发生明显消减
误差。该点没有被删除。分别从原 C `leg_spd` 和 `leg_conv` 反推 Jacobian，
两者在此点自身最大相差 `2.1624565e-04`，说明差异来自原来分别生成的
单精度表达式，而不是 Python 改换了坐标系或装配分支。

## 数学约定

固定主动关节为 `A=(0,0)`、`E=(l5,0)`，角度从 x 轴正方向逆时针增加，单位
为弧度。`phi1` 驱动 A-B，`phi4` 驱动 E-D。实机调用时后关节传入 `phi1`，
前关节传入 `phi4`，本项目保持这个顺序。

```text
B = A + l1 [cos(phi1), sin(phi1)]
D = E + l4 [cos(phi4), sin(phi4)]
O = (l5/2, 0)
l0 = |C - O|
phi0 = atan2(yc, xc-l5/2)
```

C 点严格使用 `matlab/leg_func_calc.m` 的正平方根分支：

```text
phi2 = 2*atan((B0 + sqrt(A0^2+B0^2-C0^2))/(A0+C0))
```

`src/kinematics.py` 用 SymPy 建立上述 `l0`、`phi0` 表达式，并用 `diff`
得到正式的 2x2 Jacobian：

```text
J = d(l0, phi0) / d(phi1, phi4)
[dL, dPhi] = J @ [dphi1, dphi4]
[T1, T2] = J.T @ [F, Tp]
```

有限差分 Jacobian 只用于测试解析结果，不参与速度或力矩计算。虚功等式也在
测试中验证：`[T1,T2] dot dq = [F,Tp] dot [dL,dPhi]`。

## 已验证函数

- `src.kinematics.forward_kinematics`: `xc`、`yc`、`l0`、`phi0`；其中
  `l0/phi0` 对照原 C `leg_pos`，C 点另外通过四根杆长闭环检查。
- `src.kinematics.analytic_jacobian`: SymPy 解析求导，并对照中心有限差分。
- `src.vmc.leg_velocity`: 对照原 C `leg_spd`。
- `src.vmc.joint_torques`: 对照原 C `leg_conv`。
- 原 C `leg_spd` 与 `leg_conv` 反推的两个 Jacobian 也进行了相互检查。

## 安装与运行

需要 Python 3.10+ 和 GCC。Ubuntu/KDE 环境可执行：

```bash
cd /home/fool/robot/wheel_leg_analysis
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements.txt
```

运行全部测试：

```bash
python3 -m pytest -v
```

重新编译原 C 并输出完整误差统计：

```bash
python3 scripts/validate_against_original.py
```

显示五连杆，或保存自定义角度图片：

```bash
python3 scripts/visualize_leg.py --phi1 -2.2 --phi4 -0.9
python3 scripts/visualize_leg.py --phi1 -2.2 --phi4 -0.9 \
  --output artifacts/five_bar_example.png
```

在 Python 中调用：

```python
from src.kinematics import analytic_jacobian, forward_kinematics
from src.vmc import joint_torques, leg_velocity

pose = forward_kinematics(phi1=-2.2, phi4=-0.9)
jacobian = analytic_jacobian(phi1=-2.2, phi4=-0.9)
speed = leg_velocity(-2.2, -0.9, dphi1=1.0, dphi4=-0.5)
torque = joint_torques(force=20.0, virtual_torque=0.3, phi1=-2.2, phi4=-0.9)
```

## 五连杆可视化

![Five-bar leg example](artifacts/five_bar_example.png)

图中 A、E 是两个主动关节，A-B、B-C、C-D、D-E 是四根运动连杆，C 是
足端，O 是固定关节中点，绿色虚线 O-C 是虚拟腿。

## 分支、奇异点与机械限制

- 只复现原 MATLAB 的正平方根装配分支；另一闭环解不是本阶段模型的一部分。
- 闭环判别式小于零时没有实数构型；等于零时两圆相切，Jacobian 会趋于
  奇异。当前 API 与原 C 一样不做机械限位或自动换分支。
- `2*atan` 的半角参数化在 `A0+C0=0` 附近数值敏感，即使几何本身仍合法；
  这是最差误差点的来源。
- `phi0=atan2(...)` 位于 `[-pi, pi]`，跨越分支切线时数值会跳变 `2*pi`；
  该切线上不能直接把角度差当连续速度。
- `l0=0` 时 `phi0` 无定义，相关 Jacobian 也无定义。
- 上游实机代码未给出 `phi1/phi4` 的硬机械限位。它对 `l0>0.12 m` 使用软
  保护，并对虚拟腿相对机身角及机身俯仰使用约 `+/-pi/4` 保护；这些控制层
  约束不等同于五连杆数学可达域。

## 第二阶段：原结构工作空间与传递能力

### 扫描与判据

`phi1`、`phi4` 均在 `[-pi, pi)` 上以 1 度间隔扫描，共 `360 x 360 =
129600` 个姿态。全部采样点都有闭环实数解；这里的“合法”只表示数学闭环
成立，不表示通过了未知的硬关节限位、杆件碰撞、轴承干涉或机身碰撞检查。

扫描仍取原 MATLAB 的正平方根装配分支，但用等价的两圆交点公式稳定计算 C
点。Jacobian 通过两条定长杆约束微分得到，并在常规姿态与第一阶段 SymPy
Jacobian 交叉验证。

原始 `J=d(l0,phi0)/d(phi1,phi4)` 的两行分别具有 m/rad 和 rad/rad 单位，
直接对它做 SVD 会使 condition number 随 m/mm 单位选择改变。因此 CSV
保留了用户要求的原始 `J` 指标，同时主分析采用物理齐次形式：

```text
J_phys = diag(1, l0) @ J
[dL, l0*dPhi] = J_phys @ [dphi1, dphi4]
```

`J_phys` 两行都是足端线速度，`sigma_min` 单位为 m/rad，2-范数 condition
number 无量纲，且在 `l0>0` 时与原 `J` 的秩亏位置完全相同。

| 指标 | 最小值 | 中位数 | 95 分位 | 最大值 |
|---|---:|---:|---:|---:|
| `l0` | 46.098 mm | 95.586 mm | 141.321 mm | 152.069 mm |
| `J_phys sigma_min` | 1.437e-07 m/rad | 2.266e-02 m/rad | 4.006e-02 m/rad | 4.786e-02 m/rad |
| `J_phys condition` | 1.003 | 3.712 | 30.282 | 3.212e+05 |
| 原始 `J sigma_min` | 1.692e-07 | 2.413e-02 | 4.195e-02 | 4.786e-02 |
| 原始 `J condition` | 10.238 | 34.134 | 310.297 | 4.294e+06 |

原始 `J` 行的数值仅用于复算，不用于机械优劣判断。

### XY 工作空间和虚拟腿姿态

![Workspace pose](artifacts/phase2/workspace_pose.png)

数学 XY 工作空间范围为 `x=-95.00...155.00 mm`、
`y=-144.93...152.07 mm`，虚拟腿长范围为 `46.10...152.07 mm`。中心空洞
表示当前正根装配分支无法把足端收进该区域，并非扫描遗漏。右图说明 `phi0`
完全由 O-C 方向决定，`atan2` 在负 x 轴附近仍存在 `-pi/pi` 色彩跳变。

对机器人实际意味着：机构具备很大的数学覆盖范围，但下半区、横向区域和
极端收腿姿态不等于可用于平衡。加入机身、轮子、杆件碰撞和电机硬限位后，
实际工作空间只会缩小，不能直接用这张图宣称整块区域都可安全运动。

### Jacobian 奇异性

![Jacobian singularity](artifacts/phase2/jacobian_singularity.png)

奇异/病态姿态在关节空间形成连续曲线，并不只位于工作空间最外边界。最差
离散样本是 `phi1=-75 deg, phi4=-72 deg`，此时 `l0=53.90 mm`、
`phi0=76.04 deg`、`sigma_min=1.437e-07 m/rad`、condition 约
`3.21e5`。红色 XY 点显示同一个足端位置可能由不同关节姿态到达，其中某些
姿态病态，因此仅依据足端 XY 或腿长不能判断安全性。

对机器人实际意味着：接近深色 `sigma_min` 曲线时，会失去至少一个方向的
足端运动/受力控制能力，模型误差、关节间隙、编码器噪声和扭矩误差都会被
放大。图中既包含输入杆与连杆趋于共线的串联奇异，也包含 B、D 圆心接近
重合的闭环约束病态；控制器应同时监控 `sigma_min` 或 condition，而不是只
设置最大腿长保护。

### 竖直主工作带

![Upright condition](artifacts/phase2/upright_condition.png)

该图只选取 `|phi0-90 deg|<=5 deg` 的 15589 个姿态。灰点保留同一腿长下
不同内部构型的离散性，蓝线是腿长分箱中位数，阴影是 10–90% 范围。

| `l0` | 样本数 | condition 中位数 | condition 90 分位 | 推荐比例 |
|---|---:|---:|---:|---:|
| 45–60 mm | 7166 | 2.067 | 7.535 | 35.96% |
| 60–80 mm | 1827 | 2.091 | 2.538 | 98.91% |
| 80–100 mm | 1127 | 2.727 | 3.205 | 100.00% |
| 100–120 mm | 1032 | 3.316 | 3.865 | 100.00% |
| 120–140 mm | 1335 | 1.950 | 4.619 | 97.60% |
| 140–153 mm | 3102 | 5.041 | 18.850 | 43.13% |

对机器人实际意味着：对接近平衡竖直的腿，`60–140 mm` 是明显更稳定的
运动学区间，尤其 `80–120 mm` 在本次 1 度扫描中全部满足推荐判据；短于
`60 mm` 或长于 `140 mm` 后，不同内部构型间差异急剧增大。原实机约
`70 mm` 目标腿长位于良好区间，`120 mm` 软伸长保护也恰好处于良好区间
外缘，但这只是运动学上的吻合，不是新的结构限位结论。

### 轴向力和伸缩速度传递

![Force and speed transmission](artifacts/phase2/force_speed_transmission.png)

纯轴向力按 `Tp=0`、两个关节各 `|Ti|<=1 N m` 计算：

```text
Fmax = 1 / max(|J11|, |J12|)
```

最大伸缩速度按两个关节各 `|dphii|<=1 rad/s`，允许腿角同时变化：

```text
max |dL| = |J11| + |J12|
```

这两张传递图使用 `phi1/phi4` 关节空间，而不是 XY：同一个 XY 可能对应多个
内部构型及不同传动比，直接在 XY 上覆盖着色会让颜色依赖扫描绘制顺序。

推荐区内，归一化轴向力范围为 `13.07...138.23 N`，中位数 `34.71 N`；
归一化最大伸缩速度范围为 `0.0138...0.0987 m/s`，中位数 `0.0446 m/s`。
实际相同电机的结果可分别按扭矩上限和速度上限线性缩放。

对机器人实际意味着：力和速度存在典型机械传动权衡。全局最大理论轴向力
达到 `7.47e4 N`，但同一姿态最大伸缩速度只有 `1.75e-05 m/s`；全局最大
伸缩速度 `2.894 m/s` 的姿态仅能产生约 `0.615 N` 轴向力。这些夸张数字
来自奇异放大，伴随极低控制裕度，不是可用性能。跳跃与落地分析应只在良好
condition 区域内比较力和速度，且最终还要加入电机峰值/连续扭矩及功率限制。

### 综合工作区分级

![Workspace classification](artifacts/phase2/workspace_classification.png)

三级判据是：

- 推荐：`condition<=5` 且 `sigma_min>=0.010 m/rad`；
- 可用：`condition<20` 且 `sigma_min>=0.002 m/rad`；
- 接近奇异/不建议：其余合法姿态。

全关节空间中推荐 78188 点（60.33%）、可用 40277 点（31.08%）、不建议
11135 点（8.59%）。竖直带中对应为 9184、4938、1467 点，即推荐
58.91%、可用 31.68%、不建议 9.41%。右侧 XY 图将工作空间划分为
`120 x 120` 栅格，并对落入同一格的全部内部姿态取最佳类别，因此表示该
足端小区域是否至少存在良好内部构型；左侧关节空间图才是选择具体
`phi1/phi4` 时应使用的风险图。右图白格表示离散的 1 度关节扫描没有样本
落入该 XY 小格，不应解释成新的连续不可达边界。

对机器人实际意味着：绿色不是机械安全认证，而是数值传递品质良好；橙色
可以运动但控制裕度下降；红色应避开或降速、限力。真实推荐区必须在此基础
上与关节硬限位、碰撞、自锁/回差、轴承载荷和电机热限制取交集。

### 数据与复现

每个姿态的 `phi1`、`phi4`、`xc`、`yc`、`l0`、`phi0`、原始/物理
Jacobian SVD、condition、`J11...J22`、归一化力/速度和分类码保存在：

- `artifacts/phase2/workspace_scan.csv.gz`
- `artifacts/phase2/summary.json`

重新生成全部第二阶段结果：

```bash
python3 scripts/analyze_workspace.py --resolution 360
```

降低 `--resolution` 可快速预览，但 README 数字均来自 `360 x 360` 正式扫描。

### Phase 2 结论的适用边界

Phase 2 的全局图仍然有效，但其中“竖直主工作带”的样本不是一条机械连续
轨迹：同一个竖直足端位置最多存在四种内部构型，而上游正根正运动学在
`l0=70 mm` 时会把其中三种映射到同一足端。按 `phi0` 筛选会把这些工作模式
混在一起。因此，Phase 2 的 `60--140 mm` 统计只能说明全局数学样本中存在
大量良好构型，不能据此宣布实机拥有 80 mm 连续可用行程。这个问题由下面
的 Phase 3 通过固定装配模式和连续追踪修正。

## 第三阶段：正常构型的连续伸缩

完整工程论证、方法限制和批判性复核见
[`docs/research/2026-08-23-continuous-stroke.md`](docs/research/2026-08-23-continuous-stroke.md)。

### 研究路径和可靠边界

上游总装图显示正常机械模式为两个短主动杆向外、两个长杆汇聚轮轴。对中心
竖直足端 `C=(l5/2,l0)`，解析逆运动学选择这一 `(左+, 右-)` 模式，并保持
连续展开角 `phi1+phi4=pi`。每个点都回代 Phase 1 正运动学；路径导数同时用
解析 Jacobian 逆映射和有限差分验证。Phase 1 的核心公式没有修改。

必须分开理解三种“行程”：

| 层次 | 区间 | 当前能证明什么 |
|---|---:|---|
| 数学非奇异连续行程 | `46.10 < l0 < 152.07 mm` | 同一正常分支可连续求解；两个开区间端点都是串联奇异 |
| 上游正常用户命令 | `70--90 mm` | Android 滑块与控制器代码共同直接支持的命令范围 |
| 上游控制意图扩展区 | `70--120 mm` | 离地/缓冲目标和 120 mm 软保护支持；不是机械硬限位或无碰撞证明 |

上游 Simscape 的转动关节限位开关全部关闭；Scope 保存的
`54.31--125.23 mm` 显示范围和 LQR 的 `40--140 mm` 拟合范围都不能当作
机械行程。固定版本证据链接见 [`reference/README.md`](reference/README.md)。

### 连续机械构型

![Continuous stroke geometry](artifacts/phase3/continuous_stroke_geometry.png)

五个快照都属于同一向外肘装配模式，没有在缩腿和伸腿之间偷偷切换逆解。
虚线轮廓只按上游 26 mm 轮半径提供比例，图中向下为机器人腿的物理伸出
方向，并未绘制机身实体。

对机器人实际意味着：数学上可以从接近 46.10 mm 一直连续到 152.07 mm，
但这张杆系图不能证明无干涉。50 mm 附近短杆已经进入底板/机身邻近区域，
实际最短长度必须由 SolidWorks 碰撞扫描或实物测量决定；同理，长端还需
检查轮、电机、轴承和线束。

### 关节连续运动

![Joint continuation](artifacts/phase3/joint_continuation.png)

两个主动关节全程等幅反向运动。70 mm 时连续数学角约为
`phi1=177.77 deg, phi4=2.23 deg`，120 mm 时约为
`132.75 deg, 47.25 deg`；因此 70--120 mm 伸出时每个关节连续转动约
45.01 deg。这里是模型坐标，不包含实机编码器零偏。

对机器人实际意味着：中段每伸长 1 mm 只需约 `0.85--1.10 deg` 的关节
运动，轨迹平滑；靠近两个红色奇异端点，单位腿长所需的关节转角急剧增加，
关节即使高速转动也几乎不能继续伸缩。任何真实轨迹规划都应在端点之前保留
裕度，而不是把几何相切点设成目标。

### Jacobian 品质和奇异类型

![Continuous-stroke kinematic quality](artifacts/phase3/kinematic_quality.png)

本图同时给出物理齐次 Jacobian 的两个奇异值、2-范数 condition 和两类几何
正弦裕度。最大 `sigma_min` 位于约 `l0=100.67 mm`，为
`0.04786 m/rad`。最小 condition 却位于约 50.96 mm，此时 condition 近乎
1，但 `sigma_min` 仅 `0.01481 m/rad`。

对机器人实际意味着：condition 只说明各方向是否均匀，不能说明绝对运动
能力。两个端点处奇异值一起趋零，所以 condition 仍有限；只监控 condition
会漏掉这种串联奇异。70--120 mm 内 `sigma_min>=约0.0315 m/rad`、
`condition<=约1.302`、平行奇异正弦裕度 `>=约0.966`，运动学品质良好且
均匀。阈值只是模型筛选，不是机械安全认证。

### 轴向力与固定腿角伸缩速度

![Continuous-stroke force speed](artifacts/phase3/force_speed_tradeoff.png)

纯轴向推力仍按两个关节各 `1 N m`、`Tp=0` 归一化；伸缩速度按两个关节各
`1 rad/s` 且保持 `dphi0=0` 计算。在这条对称路径上，两者严格满足
`Fmax*vmax=2 W`：70 mm 约为 `38.28 N / 0.05225 m/s`，约 100 mm 为
`29.55 N / 0.06767 m/s`，120 mm 为 `32.19 N / 0.06213 m/s`。

对机器人实际意味着：靠近奇异端点出现的巨大理想推力，与伸缩速度趋零是
同一个传动比现象，不是额外性能。70--120 mm 则经过约 100.7 mm 的最高
速度裕度点，力/速度变化温和，适合作为蹬伸和缓冲的**运动学候选区**；这些
归一化值没有包含电机扭矩-转速耦合、功率、效率、热限制或冲击，不能证明
真实机器人能够跳跃或安全吸收落地能量。

### 现在真正知道了什么

- 原杆长在 70--120 mm 中心连续路径上没有明显结构性运动学缺陷，也不需要
  为了 Phase 2 的全局工作空间百分比立即改杆长。
- 70--90 mm 是代码证据充分的正常命令行程；70--120 mm 是控制器意图与
  运动学共同支持的优秀候选区，但仍缺 CAD/实机机械证据，不能称作已认证的
  实际行程。
- 数学理论行程 105.97 mm、控制意图候选行程 50 mm 都不能直接等同于
  15 cm 台阶能力。轮半径、机身轨迹、俯仰、接触切换和动力学同样决定结果。
- 这段中部路径为动态蹬伸和落地缓冲提供了良好几何基础，但没有质量、惯量、
  扭矩-转速-热包络、机械柔顺和接触模型时，不能提前得出动力学结论。

### 数据与复现

1201 点的连续路径数据和摘要位于：

- `artifacts/phase3/normal_vertical_stroke.csv.gz`
- `artifacts/phase3/summary.json`

重新生成第三阶段全部图和数据：

```bash
python3 scripts/analyze_stroke.py --resolution 1201
```

## 真实机械包络

完整工程判定见
[`docs/research/2026-08-23-real-mechanical-envelope.md`](docs/research/2026-08-23-real-mechanical-envelope.md)。
用户提供的 AP214 原总装已通过精确实体验收：80 个导入实体全部有效，从圆柱
面恢复的五个轴心反算出 `50/105/105/50/60 mm`，与 Phase 1--3 一致。

![Real mechanical envelope](artifacts/mechanical_envelope/mechanical_envelope.png)

原结构 `70--120 mm` 的 51 个 1 mm 姿态全部无互穿和名义接触，最小间隙约
`0.985 mm`，因此它第一次同时具备“控制意图 + 良好运动学 + 原 CAD 无碰撞”
三层证据。对机器人实际意味着：这段行程可以继续作为执行器匹配和动力分析
输入，但 70 mm 附近不足 1 mm 的 STEP 名义间隙仍需公差和实物台架验证。

短端真实限制已被定位：`47--60.4 mm` 有主动杆/小腿互穿，`60.5--67.70 mm`
仍有 4010 电机/小腿零间隙接触，首个正间隙样本为 `67.71 mm`，但该点只有
约 25 nm 数值间隙，不能作为实用目标。长端到 152 mm 仍几何 CLEAR，实际先
遇到的是约 152.069 mm 的串联奇异。

EduLite 05 也不再是简单的“尺寸未知”：官方商品外形与腿系在 70--120 mm
满足必要的无碰撞条件，但保留原内部布局时会穿入 NanoPi/电池支架和支撑柱，
原 4010 支架及输出接口也不能复用。因此 EduLite 候选方向继续保留，原位直装
方案被否定；在新支架、主动杆接口、内部器件重排和线束 CAD 完成前，不能把
其最终机械行程写成已验证。

重新生成并校验姿态表和全部输入指纹：

重新生成并校验 1 mm CAD 姿态表和输入指纹：

```bash
python3 scripts/prepare_mechanical_envelope.py \
  --upstream-dir .worktrees/upstream-reference \
  --original-step /path/to/full_assembly_AP214.step \
  --original-parasolid /path/to/full_assembly.x_t \
  --edulite-step /path/to/official/el05.stp \
  --edulite-manual /path/to/official/EL05_manual.pdf
```

下一证据门不是继续扫描 Jacobian 或修改杆长，而是建立可制造的 EduLite 安装
CAD并重复同一实体扫描。通过后再进入执行器公开能力匹配、2--2.5 kg 整机和
150 mm 台阶动力学。

## EduLite 主动关节最小补丁

单腿接口、第一版中性 CAD 和逐姿态证据见
[`docs/research/2026-08-25-edulite-active-joint-module.md`](docs/research/2026-08-25-edulite-active-joint-module.md)。

本检查点保持五连杆尺寸、主动轴位置和两层杆件布置不变，只新增共用双电机
支架并重做主动杆轮毂孔系。官方接口实际是 `6×M4/PCD24` 加
`3×Ø4/PCD17.7` 输出销；`Ø17.7` 不是中心定位凸台。修正输出销孔并让输出
转子随主动杆运动后，单侧模块在 `70--120 mm` 的 51 个姿态全部 CLEAR，
最大输出接口公共体积为零；最小间隙仍是 70 mm 的 EduLite/小腿
`0.9151696 mm`。

这说明最小补丁没有损失候选行程，但单腿结果本身仍不是生产图。

## EduLite 左右整机结构连接

完整判定见
[`docs/research/2026-08-25-edulite-vehicle-structural-integration.md`](docs/research/2026-08-25-edulite-vehicle-structural-integration.md)。

整机化过程中没有把“实体贴合”当作连接：底板/支架、支架/电机和电机/主动杆
都加入了明确标准螺钉。审计实际抓出了通孔没有锁紧位置、右电机后孔钟向错误、
支架底脚与电机零间隙相切、M3 攻牙孔边缘过薄等问题，并作了局部修正。

后续批判性复核还推翻了 `e302405` 的一项结论：旧镜像函数没有接住 FreeCAD
返回的新实体，导致旧整机 STEP 的两只支架都在左侧，而只检查 Y/Z 孔心的审计
没有发现。现在左、右支架被强制限制在各自 X 半空间，安装面 X 和孔心都检查；
正式扫描也已从单腿升级为左右两腿、四台官方商品模型和跨侧实体一起扫描。

修正后，70--120 mm 的 51 个整机双侧姿态仍全部
CLEAR，最小间隙保持 `0.9151696 mm @ 70 mm`。所有新增结构件和简化螺钉均为
有效单实体。电池托板和绑带不是当前主动关节结构放行条件；支架加工细节、实物
螺钉长度/牙深和载荷证据留在生产放行前闭合，不阻塞当前公开能力匹配。

## EduLite 公开能力匹配

完整判定见
[`docs/research/2026-08-28-edulite-public-capability-match.md`](docs/research/2026-08-28-edulite-public-capability-match.md)。

厂家手册公开 48 V、1.8 N·m @ 100 rpm 旋转额定点、1.1 N·m 堵转连续值、
6 N·m 峰值、430 rpm 空载和离散过载时间。映射到 70--120 mm 路径后，
2.0/2.3/2.5 kg 两腿均载静态支撑的最差每关节扭矩分别约为
0.332/0.382/0.415 N·m；相对 1.1 N·m 的名义裕度为 3.31/2.88/2.65 倍。

这支持 EL05 继续进入动态需求匹配，但不证明 150 mm 越阶。当前更明显的新风险
是质量：四台 EL05 已达 0.968 kg，占 2.0--2.5 kg 整机目标的 48.4%--38.7%。
峰值力--速度图仅为 48 V 厂家曲线经理想机构映射后的筛选上界，不是跳跃预测。

## 50 mm 蹬伸与落地的一维筛选

完整判定见
[`docs/research/2026-08-28-simplified-dynamics.md`](docs/research/2026-08-28-simplified-dynamics.md)。

这个小模型把整机作为竖直点质量，使用已经通过机械检查的 `70--120 mm` 行程，
计算匀加速蹬伸和匀减速落地。最严苛参考为 `2.5 kg` 整机在 50 mm 行程后具有
足够再竖直上升 `15 cm` 的离地速度：两腿总力约 `98.1 N`，蹬伸约 `58.3 ms`，
峰值每关节约 `1.659 N·m @ 264 rpm`，约占当前厂家 48 V 峰值扭矩--转速近似
包络的 `37.6%`。同高度理想落地需要在 50 mm 内吸收约 `4.90 J`。

这排除了一个明显的扭矩--转速数量级矛盾，但没有模拟轮缘接触、前进速度、俯仰、
摩擦、惯量、损耗和冲击控制，不能写成“已经证明能过 150 mm 台阶”。当前判断是
原杆长继续 `KEEP`，EL05 继续验证，150 mm 任务为“可能可行、证据不足”。下一
有效证据应来自包含台阶接触和整机质量惯量的刚体模型，而不是继续细化一维公式。

## MuJoCo 真实 CAD 可视运动学检查点

完整判定见
[`docs/research/2026-08-28-mujoco-visual-kinematic-checkpoint.md`](docs/research/2026-08-28-mujoco-visual-kinematic-checkpoint.md)。

第一版 MuJoCo 整机已经使用原底板、新 EduLite 支架、四台官方 EduLite、原
五连杆和轮系的 CAD 网格，而不是用方块代替外观。22 个视觉网格从约 163 万
三角面优化为约 36.8 万面；显示网格不参与碰撞，简化胶囊/圆柱作为独立碰撞层。

左右腿各包含两个主动铰链、两个被动铰链和一个轮轴转动副，并用点连接约束闭合
第二条五连杆支链。`70/90/120 mm` 的轮轴坐标与 Phase 3 一致，左右闭环最大
数值残差分别约为 `8.72e-9/1.19e-8/1.63e-8 mm`。这证明当前 MuJoCo 零件是
真实运动学连接，不是视觉上贴合。

![MuJoCo visual kinematic checkpoint](artifacts/mujoco/visual_kinematic_checkpoint.png)

Linux 桌面交互界面：

```bash
python3 scripts/run_mujoco_viewer.py
```

控制面板提供 `70--120 mm` 腿长滑块、70/90/120 mm 预设、自动伸缩、三种相机、
碰撞体和关节轴显示。当前底盘固定，质量惯量是明确标记的临时值，碰撞层默认关闭；
这是外观/连接/连续运动检查点，不是越阶动力学结果。

## MuJoCo 整机动力学检查点

完整判定见
[`docs/research/2026-08-28-mujoco-dynamics-checkpoint.md`](docs/research/2026-08-28-mujoco-dynamics-checkpoint.md)。

动力学模型与上面的固定底盘展示模型相互独立。新模型已加入六自由度底盘、
轮地摩擦、简化底盘/连杆碰撞体、四个 `±6 N·m` EL05 扭矩源和两个
`±0.3 N·m` QD4310 轮端扭矩源。整机总质量可选 2.0/2.3/2.5 kg；EL05 和
QD4310 使用厂家质量，其余未称重部分以集中质量和简化惯量明确参数化，没有伪装
成最终实测 CAD 数据。

2.5 kg 模型从 3° 初始俯仰能够恢复；70/90/120 mm 静态腿长保持误差均小于
0.8 mm；90 mm 腿长从 5 cm 下落后能够重新站稳，本次最大 EL05/轮端命令约为
2.208/0.119 N·m。5/10/15 cm 台阶已经通过真实轮缘接触测试，但尚未加入越阶
动作，因此这些结果不等于机器人已通过台阶。

![MuJoCo dynamics validation](artifacts/mujoco_dynamics/dynamics_validation.png)

生成和验证：

```bash
python3 scripts/build_mujoco_model.py
python3 scripts/validate_mujoco_dynamics.py
```

打开原生 MuJoCo Viewer 和中文动力学控制台：

```bash
python3 scripts/run_mujoco_dynamics.py --mass 2.5
```

界面可以直接改变腿长、暂停/复位、触发 5 cm 落地、切换 5/10/15 cm 台阶、
驱动轮子接触台阶，并实时显示俯仰、位置、接触数和执行器命令。

## 第一版 5 cm 越阶状态机（当前验证门）

现在已经把第一版越阶动作落成一个可复现的、由接触事件驱动的状态机：

```text
APPROACH → CROUCH → PUSH → FLIGHT → LANDING → RECOVER
```

它只控制已有的腿部目标、虚拟腿轴向蹬伸力和轮端扭矩；阶段切换读取真实 MuJoCo
接触、腿长、底盘位置/速度和俯仰状态，不播放预先写死的底盘动画。运行：

```bash
python3 scripts/validate_stair_controller.py --mass 2.5 --height-cm 5
```

结果写入 `artifacts/stair_controller/`。复核发现原先的宽松判定虽然在约 6.206 s
进入 `SUCCESS`，但过程中俯仰曾达到约 120°，不能视为有效越阶。当前判定增加了
过程安全门：俯仰超过 75° 或着地速度超过 1.20 m/s 会立即判为失败。当前 2.5 kg /
5 cm 参数会被安全门拒绝，说明需要先修正动作本身，而不是继续放宽成功条件。

该配置使用轮端执行器限幅 1.0 N·m、蹬伸命令 0.3 N·m、蹬伸腿长 140 mm 和 120 ms
蹬伸时长。140 mm 靠近数学连续范围上端，因此这是 MuJoCo 中的 5 cm 动作验证参数，
不是实物保证，也不是长期推荐姿态。

控制器实现见 [`src/stair_controller.py`](src/stair_controller.py)，对应测试见
[`tests/test_stair_controller.py`](tests/test_stair_controller.py)。

为避免依靠单次手工试错，当前还提供并行动作搜索：它在固定模型和安全判定下，批量改变
蹲腿长度、蹬伸腿长、蹬伸时长、虚拟腿力、轮端扭矩和落地压缩腿长，并按安全裕度排序：

```bash
python3 scripts/search_stair_actions.py --samples 256 --workers 8
```

这仍然不是强化学习；它是可复现的参数搜索，用来先确认“当前动作族里是否存在安全解”。
如果搜索仍无安全成功，下一步应检查动作策略和接触/姿态控制，而不是盲目扩大样本或直接训练神经网络。

当前已增加台阶接触校准：

```bash
python3 scripts/calibrate_stair_contact.py
```

在 2.5 kg / 5 cm 模型中，轮端直接施加约 `0.15 N·m` 以上扭矩时，首次台阶接触发生在
底盘 `y≈-0.063 m`，但此时底盘俯仰已经约 `-47°`；`0.10 N·m` 以下在 1.5 s 内没有碰到台阶。
这说明轮子前沿接触不是一个可以直接用大扭矩解决的“前进距离”问题：高速撞击台阶会先制造
大俯仰。后续动作应在接触前建立抬升/姿态准备，并把轮端扭矩作为受姿态约束的辅助量。

### 对称 5 cm 越阶可行性检查

当前阶段按工程约束只测试左右完全对称的动作：两条腿同步、两个轮子同步，不引入左右先后
接触或侧倾控制。运行：

```bash
python3 scripts/validate_symmetric_stair_actions.py
```

在 2.5 kg / 5 cm 模型中，覆盖初始腿长 `60/70/80/90 mm`、接触后同步伸腿到
`90/100/110/120/130/140 mm` 的 24 组组合，没有一组建立安全的台阶顶面双轮支撑。
短行程通常卡在台阶前，长行程会产生大俯仰、速度过高或冲出台阶。结果记录在
`artifacts/stair_controller/symmetric_action_validation_2p5kg_5cm.json`。

这不是“原五连杆绝对不可能越过任何 5 cm 障碍”的数学证明；它证明的是：在当前
MuJoCo 质量、接触几何、对称同步动作和测试扭矩范围内，现有两类越阶动作没有找到安全解。
因此在继续强化学习之前，应先决定是改进纵向动作/接触策略，还是重新评估轮端与台阶的
机械接触方案；不能把当前结果包装成 5 cm 已经可行。

对 2.0/2.3/2.5 kg 与 5/10/15 cm 的统一扫描：

```bash
python3 scripts/sweep_stair_controller.py
# 多进程批量运行（每个进程拥有独立的 MuJoCo 状态）
python3 scripts/sweep_stair_controller.py --workers 8
```

该矩阵使用通用参数，不能替代经过安全判定的动作验证；当前结果中 5/10/15 cm
均因姿态安全门失败，不能视为已经越阶成功。`--workers` 只改变运行并行度，不改变模型、
控制器或判定条件。
详细结果在
`artifacts/stair_controller/sweep_summary.json`，研究边界记录在
[`docs/research/2026-08-29-stair-controller-checkpoint.md`](docs/research/2026-08-29-stair-controller-checkpoint.md)。

需要特别说明：这里的轮端限幅和蹬伸命令只是仿真测试参数，不是 EL05 的能力上限，也不是
五连杆机构的物理极限。验证脚本支持通过 `--wheel-torque-limit` 改变该测试值。
上游控制代码明确采用“离地关闭轮端、腿长切到 120 mm、触地缓冲”的策略；本仓库
已按这一证据修正状态机。

上游参考文件及来源说明位于 `reference/`，本项目按 GPL-3.0 发布。
