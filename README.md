# wheel_leg_analysis

这是对
[`Skythinker616/foc-wheel-legged-robot`](https://github.com/Skythinker616/foc-wheel-legged-robot)
五连杆腿部数学模型的独立 Python 复现。参考版本固定为上游提交
`e2444395dd3a76c20b0683fbb1e123c21186a502`。本阶段只处理正运动学、
Jacobian、速度映射、VMC 力矩映射及其数值验证，不涉及杆长优化、MuJoCo、
强化学习或 ROS。

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

## 下一步

可以直接基于这套程序继续做工作空间、奇异位形和推力/关节力矩分析。
正式开展时应在当前固定装配分支上加入真实关节限位、杆件碰撞约束、
Jacobian 条件数或最小奇异值、执行器峰值/连续力矩限制。当前阶段没有改变
杆长，也没有把控制层保护误当作机械结构约束。

上游参考文件及来源说明位于 `reference/`，本项目按 GPL-3.0 发布。
