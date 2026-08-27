# EduLite 主动关节最小补丁：单腿机械检查点

## 工程判定

本检查点没有重新设计五连杆。原来的 `50/105/105/50/60 mm`、A/E 主动轴
位置、两层主动杆布置、从动杆和轮系全部保持。修改仅限于：

- 用一个左侧共用支架固定两台 EduLite 05；
- 重做两根主动短杆的圆形输出轮毂，不改变 50 mm 轴心距和杆身；
- 保留原来的 6 mm 错层垫块，只改变其输出孔系；
- 复用原底板已有的八个 M3 支架孔，不在本检查点修改底板。

结果支持这条最小补丁路线继续 **KEEP**：使用正式商品 STEP、修改后的真实
B-Rep 杆件和第一版支架，单侧五连杆在 `70--120 mm` 的 51 个 1 mm 姿态中
全部 `CLEAR`。最小间隙仍为 `0.9151696 mm`，发生在 `l0=70 mm` 的上侧
EduLite 商品外形与原小腿之间。新支架和新轮毂没有缩短 Phase 3/4 选定的
候选行程。

这仍不是生产图纸。支架强度、紧固件载荷、连接器方向、线束 keep-out、右侧
镜像和整机内部布局尚未闭合。

## 被纠正的输出接口认识

官方说明书图和 18-solid STEP 共同表明：图中的 `Ø17.7±0.03` 是三根
`Ø4` 输出销的分布圆，不是中心定位凸台直径。输出端的机械接口为：

| 接口 | 官方尺寸 | 当前零件处理 |
|---|---:|---:|
| 输出螺纹 | `6×M4`，PCD `24 mm`，深 `3 mm` | `6×Ø4.3` 通孔 |
| 输出销 | `3×Ø4`，PCD `17.7 mm`，突出约 `3 mm` | `3×Ø4.2` 通孔 |
| 后部固定 | `4×M3`，PCD `38.5 mm`，深 `11 mm` | 支架 `4×Ø3.4`/电机 |
| 主动杆中心孔 | 商品端没有 Ø17.7 中心凸台要求 | 改为 Ø10 减重/工具孔 |

第一版尝试把 Ø17.7 错当中心避让孔时，三根输出销与主动杆产生约
`49--51 mm³` 穿透。精确定位三根销后，轮毂改为独立销孔；同时扫描程序把
输出盘和输出销随主动杆旋转，而不是错误地把整台电机都当固定刚体。

六孔阵列本身具有 60° 装配歧义。程序同时用三销阵列消除该歧义，否则即使
六个 M4 孔对齐，也可能把三根销装到错误角度。修正后 51 个姿态的两套
电机—主动杆最大公共体积均为 `0 mm³`。

## 第一版机械结构

直接侧主动杆仍位于原 `x=-71.5...-66.5 mm` 层。错层侧仍使用
`x=-72.5...-66.5 mm` 的 6 mm 垫块，主动杆位于
`x=-77.5...-72.5 mm`。因此两层杆件、轴向间距以及二维运动学完全不变。

轮毂外半径由原约 16 mm 局部增加到 19 mm，为 PCD24 的 M4 通孔保留边缘
材料；它仍小于 EduLite 约 23 mm 的主体半径，因此没有扩大已由商品电机
决定的径向包络。

共用支架用 EduLite 后部四孔固定两台电机，输出安装面保持在原全局
`x=-66.5 mm`，后安装面位于 `x=-22.5 mm`。支架底脚复用原底板左侧八个
M3 孔。后续连接审计发现第一版底脚同样使用通孔时没有锁紧位置，因此已经改为
6 mm 厚底脚中的 M3 攻牙孔，使用 M3x10 从底板下方固定，名义啮合 6 mm。
支架用中部和两端三块肋板连接后安装板。当前几何是用于闭合空间和接口的单件
加工概念，不是已经完成强度校核的加工图。

后续整机连接复核又消除了两个容易被理想 CAD 隐藏的问题：底脚上表面从
`y=11 mm` 降到 `y=10 mm`，使它与官方 EduLite 外形从零间隙相切改为
`1.0 mm` 名义间隙；底脚外缘从 `x=-60 mm` 延伸到 `x=-62 mm`，使最外侧
M3 攻牙孔按 Ø3 mm 大径计算的边缘余量从 `1.0 mm` 增至 `3.0 mm`。这两处
只修改支架毛坯，不移动原底板孔和主动轴。

可直接查看的中性 CAD：

- `artifacts/edulite_joint_module/edulite_left_shared_bracket.step`
- `artifacts/edulite_joint_module/edulite_direct_proximal_link.step`
- `artifacts/edulite_joint_module/edulite_offset_spacer.step`
- `artifacts/edulite_joint_module/edulite_offset_proximal_link.step`
- `artifacts/edulite_joint_module/edulite_single_leg_90mm.step`

最后一个单腿参考装配为避免复制厂家完整商品模型，只放入 Ø46×44 mm 名义
电机圆柱用于显示；所有数值检查都使用哈希固定的官方 18-solid STEP，不能用
显示圆柱替代碰撞检查。

## 70--120 mm 扫描结果

| 项目 | 结果 |
|---|---:|
| 姿态 | 51 |
| CLEAR | 51 |
| 最大实体互穿 | `0 mm³` |
| 输出接口最大公共体积 | `0 mm³` |
| 全段最小间隙 | `0.9151696 mm @ 70 mm` |
| 最小间隙实体 | 上侧 EduLite / `小腿004` |

`108--116 mm` 的最小值为 `1.4430107 mm`，限制对仍是原底板和随小腿运动的
原 M4×16 螺钉，与 Phase 4 一致。新支架在轴向上与所有运动杆件至少分离
6.5 mm；路径扫描没有发现新支架限制姿态。

证据层级是“单侧真实 B-Rep 几何强证据”：它证明当前接口概念在理想 CAD 中
装得上并能走完整段行程。它没有证明 0.915 mm 足以覆盖加工公差、轴承游隙和
受力挠曲，也没有证明支架能承受跳跃和落地载荷。

## 后续校准

项目随后将优先级从连接器细化调整为左右整机结构替换，并把真实螺钉连接作为
进入整机的门槛。连接器和线束仍是待办，但不再阻塞结构骨架。左右支架、四台
电机、输出螺钉和整机结构结果见
[`2026-08-25-edulite-vehicle-structural-integration.md`](2026-08-25-edulite-vehicle-structural-integration.md)。

## 复现

```bash
snap run --shell freecad.cmd -c \
  "PYTHONPATH=\"\$SNAP/usr/lib:\$PYTHONPATH\" python3 \
  scripts/build_edulite_single_leg_freecad.py \
  --edulite-step .worktrees/edulite-reference/产品资料/EL05/el05.stp"

snap run --shell freecad.cmd -c \
  "PYTHONPATH=\"\$SNAP/usr/lib:\$PYTHONPATH\" python3 \
  scripts/scan_edulite_single_leg_freecad.py \
  --edulite-step .worktrees/edulite-reference/产品资料/EL05/el05.stp"
```

输入和输出 SHA-256、孔系以及初始装配检查记录在
`artifacts/edulite_joint_module/interface_audit.json`。该阶段的左腿扫描仍保留在
`artifacts/edulite_joint_module/scan_70_120/`；修正右支架后的正式整机双侧结果
位于 `artifacts/edulite_vehicle/scan_70_120/`。
