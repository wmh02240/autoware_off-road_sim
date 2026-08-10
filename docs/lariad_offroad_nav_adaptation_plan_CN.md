# LARIAD Offroad-Nav 方案适配实施方案

版本：V1.0  
日期：2026-08-09  
目标项目：`autoware_off-road_sim`  
上游项目：[LARIAD/Offroad-Nav](https://github.com/LARIAD/Offroad-Nav)  
上游生成器：[Offroad-Nav/terrain.generator](https://github.com/LARIAD/Offroad-Nav/tree/main/terrain.generator)

## 1. 实施目标

将 LARIAD Offroad-Nav 中的自然环境能力适配到当前项目，并逐步把 `terrain.generator` 改造成面向割草机的草坪场景生成工具。

最终需要具备以下能力：

- 在当前 Isaac Sim 6.x + ROS 2 Humble 环境中加载 LARIAD 户外资产；
- 先复用 `easy.usd`、`medium.usd`、`hard.usd` 完成兼容性验证；
- 在 Isaac Sim 编辑器中启用移植后的 `terrain.generator`；
- 参数化生成平地、单向坡地、缓起伏草坪；
- 参数化放置短草、高草、树木、灌木、岩石和业务障碍物；
- 输出可独立加载、引用路径可迁移、结果可复现的 USD 场景包；
- 与当前车辆、ROS 2、传感器、地表摩擦和语义分割功能兼容；
- 后续能够在不修改仿真启动器的情况下持续生成新环境。

## 2. 已确认的基线与差异

### 2.1 上游 LARIAD 基线

LARIAD 当前公开方案面向：

- Isaac Sim 4.5.0；
- ROS 1 Noetic；
- `terrain.generator` 2.0.0；
- 完整越野导航系统，而不是割草机仿真；
- `easy.usd`、`medium.usd`、`hard.usd` 三个保存场景。

其中 `terrain.generator` 是 Joel Ventola 原始森林生成器经 Eddie Groh 重构后的版本。其 2.0.0 changelog 标记为 “Complete Rebuild of the Extension”。

### 2.2 当前项目基线

当前项目使用：

- Isaac Sim 6.0 开发版本/源码构建流程；
- ROS 2 Humble；
- `scripts/launch_sim.py` 统一加载环境、车辆和传感器；
- 环境被引用到 `/World/Environment`；
- YAML 配置环境缩放、旋转、平移和地表摩擦；
- 环境 USD 与 Docker 镜像一起管理。

### 2.3 主要适配差异

| 类别 | LARIAD | 当前项目 | 处理策略 |
|---|---|---|---|
| Isaac Sim | 4.5.0 | 6.x | 对扩展进行 API 冒烟测试和迁移 |
| ROS | ROS 1 Noetic | ROS 2 Humble | 不移植 LARIAD 导航栈，只移植环境能力 |
| 生成器安装 | `isaac/extsUser` 符号链接 | Isaac Sim 源码构建/容器 | 使用独立 extension folder 或项目扩展目录 |
| 场景用途 | 森林越野 | 割草作业 | 增加缓坡、作业区、禁区和边界控制 |
| 场景加载 | 直接打开 USD | 无 prim-path reference | 统一 `defaultPrim` 和包装层 |
| 物理场景 | 生成器可能创建 PhysicsScene | 启动器统一管理 | 导出时移除嵌入的 PhysicsScene |
| 资产路径 | 指向 `extsUser/terrain.generator/data` | 项目路径/容器路径 | 打包时改写为相对路径 |
| 资产规模 | 完整仓库约 1.9 GB | 当前无 Git LFS 管理方案 | 代码与大资产分离管理 |

## 3. 总体技术路线

采用“生成端与运行端分离”的架构：

```text
LARIAD terrain.generator（编辑器/离线生成端）
        │
        ├─ 参数配置 YAML
        ├─ 地形生成
        ├─ 植被与障碍物布置
        └─ USD 打包与校验
                    │
                    ▼
        可迁移的 Lawn Scene Package
        ├─ root.usda
        ├─ terrain.usdc
        ├─ vegetation.usdc
        ├─ obstacles.usdc
        ├─ materials/
        ├─ textures/
        └─ manifest.yaml
                    │
                    ▼
当前 scripts/launch_sim.py（运行端）
        ├─ 引用环境 USD
        ├─ 创建唯一 PhysicsScene
        ├─ 加载割草机器人
        ├─ ROS 2 Bridge
        └─ Camera/LiDAR/GNSS/IMU
```

### 3.1 核心原则

1. **不移植 LARIAD 的 ROS 1 导航栈**：当前任务只需要场景和生成器。
2. **生成器不进入运行时关键路径**：仿真运行只加载导出的 USD。
3. **代码与大型资产分开**：避免把约 1 GB 数据直接写入主 Git 历史。
4. **物理与视觉分层**：地形和树干负责碰撞，草簇只负责视觉/传感器表现。
5. **输出必须可复现**：所有随机生成必须记录 seed 和配置。
6. **输出必须可迁移**：不得保留开发机绝对路径或 `../isaac/extsUser/...` 引用。

## 4. 建议目录结构

```text
autoware_off-road_sim/
├── extensions/
│   └── lawn.terrain.generator/       # 从 terrain.generator 2.0.0 派生的适配代码
├── assets/
│   └── environments/
│       └── lawn_generated/
│           ├── lawn_flat_01/
│           │   ├── root.usda
│           │   ├── terrain.usdc
│           │   ├── vegetation.usdc
│           │   ├── obstacles.usdc
│           │   ├── materials/
│           │   ├── textures/
│           │   └── manifest.yaml
│           └── lawn_slope_01/
├── external_assets/                  # 默认不提交 Git；运行时挂载或同步
│   └── lariad_terrain_assets/
├── scripts/
│   ├── configs/
│   │   ├── lawn_flat_01.yaml
│   │   └── lawn_slope_01.yaml
│   └── tools/
│       ├── install_lawn_generator.sh
│       ├── validate_lawn_usd.py
│       └── package_lawn_scene.py
└── third_party/
    ├── LARIAD_OFFROAD_NAV_LICENSE.txt
    └── LARIAD_ASSET_MANIFEST.md
```

扩展名称建议从 `terrain.generator` 改为 `lawn.terrain.generator`，避免与上游安装版本冲突，并清晰标识这是项目派生版本。源文件需保留原作者、MIT 许可证和修改说明。

## 5. 分阶段实施

### 5.1 阶段 0：版本冻结与资产合规

#### 工作内容

- 记录 LARIAD 上游 commit；当前调研所用版本为 `69640cf19eec1d90214ed4a3615788dd5dc1e3c4`；
- 将 Isaac Sim Docker 构建从浮动分支改为明确 tag 或 commit；
- 修正 README 与 Dockerfile 对 Isaac Sim 分支描述不一致的问题；
- 保存 LARIAD MIT 许可证；
- 对 `terrain.generator/data` 中每类树木、草、石块、纹理和 HDRI 建立来源清单；
- 将来源无法确认的资产标记为“仅内部验证，不可再分发”；
- 决定大型资产的交付方式：独立压缩包、对象存储或 Git LFS。

#### 建议决策

- 生成器代码可以按 MIT 派生；
- 大型视觉资产先作为外部资产包挂载，不直接纳入主仓库；
- 最终需要公开或商业分发时，将来源不明确的内容替换为 CC0 资产。

#### 验收条件

- Isaac Sim 和 LARIAD 均有不可变版本标识；
- `LARIAD_ASSET_MANIFEST.md` 至少记录文件、来源、许可证状态和是否允许再分发；
- 干净环境能够按照文档得到相同版本。

#### 实施状态（2026-08-09）

阶段 0 已完成：

- Isaac Sim 固定为 `v6.0.0` / `40316786340b3f034a229d9e12650df1ac0b68ab`；
- LARIAD 固定为 `69640cf19eec1d90214ed4a3615788dd5dc1e3c4`；
- 机器可读版本锁保存在 `third_party/lariad_versions.yaml`；
- LARIAD MIT 许可证副本保存在 `third_party/LARIAD_OFFROAD_NAV_LICENSE.txt`；
- 资产审计及再分发状态保存在 `third_party/LARIAD_ASSET_MANIFEST.md`；
- 大型资产采用 `external_assets/` 外部稀疏 checkout，并由 `scripts/tools/fetch_lariad_assets.sh` 获取；
- `external_assets/` 已从 Git 和 Docker build context 中排除。

### 5.2 阶段 1：现有 LARIAD USD 场景接入

这一阶段不修改生成器，目标是先证明 LARIAD 环境能在当前运行框架中工作。

#### 工作内容

1. 在外部资产目录部署 LARIAD 的 `assets/` 与 `terrain.generator/data/`；
2. 修复场景中的 `../isaac/extsUser/terrain.generator/data/...` 引用；
3. 查找并处理 `block.usd` 缺失引用；
4. 处理生成器资产注册表中存在、但公开数据目录缺少的 `Switchgrass/Switchgrass.usd` 和 `Container_J01/Container_J01_126x120x133cm_PR_V_NVD_01.usd`；
5. 检查 `OmniPBR.mdl` 和云端 MDL 材质在 Isaac Sim 6.x 中的解析；
6. 给 `medium.usd`、`hard.usd` 增加明确 `defaultPrim`，或创建 `root.usda` 包装层；
7. 将 Stage 单位统一为米、Up Axis 统一为 Z-up；
8. 移除环境内的 PhysicsScene，让 `launch_sim.py` 创建唯一物理场景；
9. 新增只启用一台车辆的测试 YAML 配置；
10. 分别验证 easy、medium、hard 场景。

#### 包装层示意

```usda
#usda 1.0
(
    defaultPrim = "LawnEnvironment"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "LawnEnvironment" (
    kind = "assembly"
)
{
    def Xform "Terrain"
    {
        prepend references = @./source_scene.usd@</World/GeneratedForest>
    }
}
```

实际 prim path 应以场景检查结果为准，不能假定上游 USD 一定以 `/World` 为默认根。

#### 启动配置示意

```yaml
environment:
  asset: "assets/environments/lawn_generated/lariad_hard_compat/root.usda"
  scale: [1.0, 1.0, 1.0]
  rotation_euler: [0.0, 0.0, 0.0]
  translation: [0.0, 0.0, 0.0]
  frictions:
    - name: "GrassPhysicsMaterial"
      dynamic_friction: 0.45
      static_friction: 0.60
```

摩擦数值只是初始值，必须根据目标轮胎、草地干湿状态和真实底盘测试标定。

#### 验收条件

- 三个环境均能在 Isaac Sim 6.x 打开且无 unresolved reference；
- Stage 中只有一个 PhysicsScene；
- 车辆不会出生在地下、悬空或立即发生物理爆炸；
- Camera、LiDAR、GNSS、IMU 和 ROS 2 topic 正常；
- 保存和重新加载后材质、碰撞和植被不丢失。

#### 实施状态（2026-08-09）

阶段 1 的代码与静态 USD 验证已完成：

- `scripts/tools/fetch_lariad_assets.sh` 在固定 checkout 内建立旧扩展路径兼容链接；
- `scripts/tools/prepare_lariad_scenes.py` 从上游 `easy/medium/hard.usd` 生成非破坏性的 `compat/*.usda`，原始二进制 USD 保持不变；
- 适配层移除上游 Barakuda、编辑器测量辅助节点和缺失的 `block.usd` payload，并消除 Barakuda 带入的云端 MDL 依赖；
- 旧的 `../isaac/extsUser/terrain.generator/` 引用被重写为 checkout 内稳定相对路径，`edges.usd` 引用也已修正；
- 三个适配层均设置 `/map` 为 `defaultPrim`、单位为米、坐标轴为 Z-up，且不包含 `PhysicsScene`；
- `Switchgrass` 与 `Container` 的缺失状态写入生成清单并默认禁用；`OmniPBR.mdl` 作为 Isaac Sim 运行时内置模块保留；
- 新增割草机专用启动器副本 `scripts/launch_lawn_mower_sim.py`，LARIAD 适配不修改原有 `launch_sim.py`；
- 三套 LARIAD 配置显式记录各场景的根节点变换，避免依赖或影响原启动流程；
- 新增 `lariad_easy.yaml`、`lariad_medium.yaml`、`lariad_hard.yaml` 三个单车配置。

使用 OpenUSD 完成了三套场景的全量组合、依赖、材质目标、单位、轴向、默认根节点和唯一物理场景静态检查。当前机器的 NVIDIA 驱动不可用，Docker daemon 中也没有可运行的 `autoware-off-road_sim:6.0` 镜像，因此尚未完成 GPU 运行时验收；车辆落地状态、传感器、ROS 2 topic、Isaac Sim 内置 MDL 解析及保存重载仍需按 README 中的三个启动命令实机确认。为遵守版本约束，本阶段没有升级到 6.0.1，也没有触发 Isaac Sim 重编译。

### 5.3 阶段 2：`terrain.generator` 2.0.0 移植到 Isaac Sim 6.x

#### 安装方式

开发期优先使用 Isaac Sim 支持的外部扩展参数：

```bash
isaac-sim.sh \
  --ext-folder /workspace/autoware-off-road_sim/extensions \
  --enable lawn.terrain.generator
```

Isaac Sim 6.0 官方建议使用 CLI Extension Templates 创建新扩展骨架；可先生成一个 Python/UI 扩展，再移入上游逻辑，避免继续沿用旧模板结构。[Isaac Sim CLI Extension Templates](https://docs.isaacsim.omniverse.nvidia.com/latest/utilities/cli_extension_templates.html)

#### Python 依赖

至少检查：

- `perlin_noise`；
- `numpy`；
- `scipy`；
- Isaac Sim 自带 `pxr`、`carb`、`omni.*` 和 `isaacsim.*`。

依赖应安装到 Isaac Sim 自带 Python，而不是系统 Python。版本写入扩展专用 requirements，Docker 构建时安装，禁止每次启动在线安装。

#### API 迁移检查

重点检查上游使用的：

- `isaacsim.core.prims.XFormPrim`；
- `isaacsim.core.utils.prims`；
- `isaacsim.core.utils.stage`；
- `omni.kit.menu.utils`；
- `omni.usd` Stage event；
- `UsdPhysics`、`PhysxSchema`；
- `UsdGeom.PointInstancer`；
- `UsdLux.DomeLight`；
- OmniPBR MDL 创建逻辑。

Isaac Sim 6.0 已将多项功能迁移到 Core Experimental API 和新的 Simulation/Rendering Manager，不能只依赖“import 成功”判断兼容性。[Isaac Sim 6.0 Release Notes](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html)

#### 必须修复的问题

1. 将扩展依赖完整写入 `extension.toml`；
2. 验证 UI 菜单注册和销毁过程，防止重复启用后残留窗口；
3. 所有 Stage 修改仍在 Kit 主线程执行；
4. 后台线程只计算高度场和实例姿态，不直接写 USD；
5. 将资产根目录从“扩展目录内固定 data”改为可配置路径；
6. 资产注册表加载时检查文件是否存在；缺失的 Switchgrass/Container 默认禁用并给出明确诊断；
7. 去除生成器自动创建 PhysicsScene 的职责；
8. 修正重力单位逻辑，直接按 Stage meters-per-unit 计算；
9. 删除测试模板遗留的 `test_hello_world.py`，增加真实启动和生成测试；
10. 对 PointInstancer 原型可见性 workaround 做回归验证；
11. 避免为每棵树复制高精度 triangle mesh collider。

#### 验收条件

- Extension Manager 能发现、启用和禁用扩展；
- `Window -> Lawn Terrain Generator` 可正常打开；
- 连续生成、清空、再次生成至少 10 次无崩溃和 prim 泄漏；
- 固定 seed 产生相同高度场和对象数量；
- Headless 模式可以通过配置文件生成场景，不依赖 UI 点击。

#### 实施状态（2026-08-10）

阶段 2 的适配代码已落地到 `extensions/lawn.terrain.generator/`：

- 扩展及 Python module 均更名为 `lawn.terrain.generator`，保留上游作者、固定 commit、MIT 许可证指引和修改说明；
- `extension.toml` 声明 Kit UI、菜单、USD 和测试依赖，第三方 Python 版本范围保存在扩展专用 `requirements.txt`，Docker 构建时安装到 Isaac Sim Python；
- 菜单注册、Stage 事件订阅、窗口和后台任务均有对应销毁逻辑，重复禁用不会保留窗口或订阅；
- 高度场和实例姿态由纯计算模块生成，UI 路径在线程池中计算，USD writer 强制由创建它的 Kit 主线程调用；
- 高度场采用扩展内置的 NumPy 平滑 fBm，避免现有容器因缺少 `perlin_noise` 而在运行时失败或在线安装依赖；
- 资产根支持扩展 setting、环境变量、YAML 和项目默认路径，注册表会逐项检查；缺失的 `Switchgrass`、`Container` 会诊断并跳过；
- 生成器不创建或配置 `PhysicsScene`，不包含旧重力换算逻辑；地形保留静态网格碰撞，树木和岩石仅使用 capsule/sphere 简化代理；
- 植被和视觉对象使用标准 `UsdGeom.PointInstancer` prototype relationship，不再使用上游 Instance0/可见性 workaround；
- `scripts/tools/generate_lawn_scene.py` 提供 YAML 驱动的 headless 入口，并在 `--cycles 10` 时执行确定性、清空和 prim 泄漏检查；
- 已删除模板 Hello World 测试，新增纯计算与源码契约测试。

运行时验收应在现有 `autoware-off-road_sim` 容器中执行 README 所列命令；生成脚本会输出高度场 SHA-256、对象数量、缺失资产诊断及扩展启停结果。

运行时验收已在现有 `autoware_off-road_sim` 容器（Isaac Sim 6.0.0、RTX 4060 Ti）完成：扩展由 Extension Manager 发现并启用，真实窗口类可构造/销毁，连续 10 轮生成与清空正常退出，禁用后再次启用成功；固定 seed 的高度场 SHA-256 始终为 `de06202e59378766abe3a2bdbe825ec42e80fe4eb66275f9e0c17cc0edbc7e86`，每轮对象计数均为 Grass 10、Birch 1、Rock 1、Switchgrass 0；输出 USD 依赖扫描无 unresolved dependency，包含有效 defaultPrim、米制、Z-up 和标准 PointInstancer，且不包含 PhysicsScene。主机侧 7 项确定性/源码契约测试亦全部通过。

移除外部噪声包后，当前内置 NumPy fBm 在同一阶段 2 配置下的高度场 SHA-256 更新为 `4d9c56b18a82cf0350fc731b464f1228aed06cbd1c41f9302c06580d543be0e3`，高程范围为 -0.5～0.5 m，对象数量保持不变。

### 5.4 阶段 3：面向割草机的生成器改造

原生成器的 Perlin 地形偏森林越野。割草场景需要平缓、连续、有明确作业边界的地形模型。

#### 5.4.1 地形生成模式

增加以下模式：

| 模式 | 参数 | 用途 |
|---|---|---|
| `flat` | 基础高度、微起伏幅值 | 标准割草与覆盖规划 |
| `single_slope` | 坡向、坡度、过渡宽度 | 爬坡能力和横坡稳定性 |
| `rolling_lawn` | 最大坡度、波长、seed | 自然缓起伏草坪 |
| `terraced_lawn` | 平台数、坡间过渡 | 分区草坪和边界测试 |
| `heightmap` | 高度图、缩放、平滑 | 复现真实测绘草地 |

需要增加硬性约束：

- 最大纵坡；
- 最大横坡；
- 最大局部曲率；
- 出生点平整半径；
- 充电站平整区域；
- 边缘安全缓冲区。

生成后计算每个网格单元的坡度。超过阈值时应平滑、重生成或标为禁区，不能只依赖 Perlin 参数间接控制。

#### 5.4.2 草地与植被分区

新增空间 mask：

```text
整个地块
├─ mowable_area       可割短草作业区
├─ no_mow_zone        花坛、水体、树根保护区
├─ boundary_buffer    围栏和边界缓冲区
├─ tall_grass_band    边缘高草带
├─ bare_soil_patch    裸土/稀疏草斑块
└─ spawn_pad          机器人出生/充电平台
```

植被生成必须遵守：

- 短草可覆盖 `mowable_area`，但不创建逐株碰撞；
- 高草只出现在边缘带、禁区或专门测试区域；
- 树木、岩石、花坛与可行驶区域保持最小安全间距；
- 树干和岩石使用简化 collider；
- 灌木和草叶的 LiDAR 可见性可按测试目的开关；
- 密度随区域变化，避免完全均匀随机分布。

#### 5.4.3 割草业务对象

增加可选资产类别：

- 围栏和门；
- 花坛和树池；
- 充电站；
- 喷灌头；
- 长椅、路灯和垃圾桶；
- 小动物或行人预留 spawn point；
- 落枝、玩具等临时障碍物。

业务对象配置采用资产清单，不把文件路径硬编码在 Python 中。

#### 5.4.4 语义与真值输出

建议输出以下标签：

- `terrain_grass`；
- `terrain_soil`；
- `vegetation_short_grass`；
- `vegetation_tall_grass`；
- `tree_trunk`；
- `bush`；
- `rock`；
- `fence`；
- `flower_bed`；
- `charging_station`；
- `mowable_area`；
- `no_mow_zone`。

同时导出：

- 作业区 polygon；
- 禁区 polygon；
- 坡度栅格；
- 高程栅格；
- 2D occupancy map；
- 机器人推荐出生位姿；
- 配置 seed 与资产版本 manifest。

#### 参数文件示意

```yaml
schema_version: 1
scene_name: lawn_slope_01
global_seed: 20260809

terrain:
  mode: rolling_lawn
  size_m: [80.0, 60.0]
  resolution_m: 0.25
  max_elevation_delta_m: 2.0
  max_slope_deg: 12.0
  smoothing_passes: 3
  spawn_pad:
    center_m: [-30.0, -20.0]
    radius_m: 3.0

vegetation:
  short_grass_density_per_100m2: 250
  tall_grass_density_per_100m2: 25
  tree_density_per_100m2: 0.15
  rock_density_per_100m2: 0.2
  work_area_clearance_m: 1.0

physics:
  grass_static_friction: 0.60
  grass_dynamic_friction: 0.45

output:
  directory: assets/environments/lawn_generated/lawn_slope_01
  package_dependencies: true
  remove_physics_scene: true
  set_default_prim: true
```

### 5.5 阶段 4：USD 分层、打包与运行时接入

#### 推荐 USD 分层

```text
root.usda                 # 入口、defaultPrim、组合关系
├─ terrain.usdc           # 地形视觉网格和简化碰撞
├─ vegetation.usdc        # PointInstancer、树木、草簇
├─ obstacles.usdc         # 围栏、花坛、岩石、充电站
├─ semantics.usda         # 语义标签和业务区域
└─ lighting.usda          # DomeLight/太阳，可选择覆盖
```

#### 打包器职责

- 使用 `UsdUtils.ComputeAllDependencies` 或等价方式收集依赖；
- 把资产、材质和纹理引用改成相对路径；
- 检查缺失引用和大小写错误；
- 设置 `defaultPrim`、`metersPerUnit=1` 和 `upAxis=Z`；
- 移除多余 PhysicsScene；
- 禁止保存开发机绝对路径；
- 生成 `manifest.yaml`，记录 seed、参数、上游 commit、资产哈希和许可证状态；
- 可选生成 `.usdz` 供传输，但运行时优先使用普通分层 USD，便于调试和局部替换。

#### 启动器改造建议

为 `environment` 配置增加可选字段：

```yaml
environment:
  asset: "assets/environments/lawn_generated/lawn_slope_01/root.usda"
  prim_path: "/LawnEnvironment"
  validate_dependencies: true
```

加载时优先使用显式 `prim_path`，没有配置时才依赖 `defaultPrim`。这样既兼容 LARIAD 场景，也兼容现有环境。

当前 `strip_embed_physics_scenes()` 会打开并保存源环境 USD。建议后续改为：

- 在生成/打包阶段清理 PhysicsScene；或
- 在组合 Stage 中通过 override 禁用重复 PhysicsScene；
- 不在普通仿真启动过程中修改源资产文件。

### 5.6 阶段 5：物理、传感器与性能优化

#### 碰撞策略

| 对象 | 碰撞策略 |
|---|---|
| 连续地形 | 单个或分块 triangle mesh static collider |
| 短草/高草 | 默认无 collider |
| 树干 | capsule、convex hull 或简化 trunk mesh |
| 树冠/树叶 | 无 collider |
| 小岩石 | convex hull；过小石块可无碰撞 |
| 花坛/围栏 | 简化 box/convex collider |
| 充电平台 | box collider，单独物理材质 |

#### 渲染策略

- 短草使用 3–6 个低面数原型和 Point Instancer；
- 近景显示立体草，中远景依赖地表 PBR；
- 树木和岩石使用实例与 LOD；
- 阴影距离、RTX 反射和透明草叶按目标 GPU 调整；
- 不直接高密度实例化 LARIAD/扫描级高面数草模型。

#### 传感器策略

- RGB：检查 alpha cutout、双面渲染、阴影和摩尔纹；
- LiDAR：决定短草是否可见，高草和树干必须按测试需求返回点；
- 深度相机：检查透明叶片深度是否合理；
- 语义分割：Point Instancer 原型与实例均应获得正确标签；
- GNSS：树木遮挡和多路径模型属于后续功能，不与第一阶段场景适配耦合。

## 6. 测试与验收体系

### 6.1 静态 USD 检查

每个生成场景必须通过：

- Stage 可打开；
- defaultPrim 有效；
- 所有 reference、payload、texture 和 MDL 可解析；
- 无绝对文件路径；
- 米制和 Z-up 正确；
- 只有一个预期的环境根 prim；
- 导出环境不包含 PhysicsScene；
- prim path 命名合法且稳定；
- 生成参数和资产哈希完整。

### 6.2 地形几何检查

- 最大坡度不超过配置阈值；
- 无 NaN、Inf、退化三角形和越界索引；
- 无明显裂缝、重叠面或法线翻转；
- 出生点和充电区满足平整度要求；
- 地形碰撞高度与视觉高度一致；
- 轮胎不会在网格接缝处跳变或穿透。

### 6.3 Isaac Sim 冒烟测试

- GUI 和 Headless 均可加载；
- 连续运行 30 分钟无崩溃；
- 仿真时间能稳定推进；
- ROS 2 Bridge 正常；
- 车辆控制、Camera、LiDAR、GNSS、IMU 和 odometry 正常；
- 环境重复加载无 PhysicsScene 和 DomeLight 累积。

### 6.4 性能测试

固定硬件、分辨率和传感器配置，记录：

- Stage 加载时间；
- GPU 显存峰值；
- 静态 prim 数量；
- PointInstancer 实例数量；
- 渲染 FPS；
- Physics step time；
- Camera + LiDAR 同开时的实时系数；
- Headless 运行实时系数。

性能验收值不应先拍脑袋确定。先以当前 `pumptrack_simple` 作为基线，再规定新草坪相对基线允许的回退范围。

## 7. 里程碑与工作量估计

以下为单人熟悉 Python/USD、但需要验证 Isaac Sim 6.x API 时的工程估计：

| 里程碑 | 主要交付物 | 估计工作量 |
|---|---|---:|
| M0 版本与合规冻结 | commit、许可证、资产清单、交付策略 | 1–2 人日 |
| M1 现有场景兼容 | easy/medium/hard 包装层和测试配置 | 2–4 人日 |
| M2 生成器启动 | Isaac Sim 6.x 可加载扩展、依赖和冒烟测试 | 3–6 人日 |
| M3 割草参数化 | 缓坡、mask、草地/障碍物规则、YAML schema | 6–10 人日 |
| M4 USD 打包接入 | 分层 USD、相对依赖、manifest、启动器适配 | 3–5 人日 |
| M5 联调和优化 | 物理、传感器、性能与三套验收场景 | 5–8 人日 |

总计约 20–35 人日。若 Isaac Sim 6.x 对旧 UI/API 的兼容问题较少，可接近下限；若需要替换大量来源不明资产或重做材质，工作量会明显增加。

## 8. 建议首批交付的三套场景

### 8.1 `lawn_flat_01`

- 60 m × 40 m；
- 最大坡度不超过 3°；
- 短草主区域；
- 少量树木、花坛和围栏；
- 用于基础覆盖规划和传感器联调。

### 8.2 `lawn_slope_01`

- 80 m × 60 m；
- 主坡 8°，局部不超过 12°；
- 平整出生区和充电区；
- 用于纵坡、横坡和动力学测试。

### 8.3 `lawn_mixed_01`

- 平地与缓起伏混合；
- 高草边缘带、树根禁区、裸土斑块；
- 花坛、围栏、岩石和临时障碍物；
- 用于定位、感知和路径规划综合测试。

## 9. 风险与控制措施

| 风险 | 影响 | 控制措施 |
|---|---|---|
| Isaac Sim 浮动版本 | 今天可用、后续构建失败 | 固定 tag/commit，建立扩展启动测试 |
| 上游资产来源不完整 | 无法公开或商业分发 | 独立资产清单；逐步替换为 CC0 |
| USD 引用依赖外部安装路径 | 换机器后材质/模型丢失 | 打包器改写相对路径并做依赖扫描 |
| 注册表引用的 Switchgrass/Container 缺失 | 部分生成任务失败 | 启动时校验；默认禁用；使用许可清晰资产替换 |
| 高草与树木面数过高 | FPS、显存和 LiDAR 性能下降 | LOD、PointInstancer、显示半径和代理资产 |
| 多个 PhysicsScene | 仿真不稳定或参数冲突 | 导出环境不带 PhysicsScene，运行端唯一创建 |
| Perlin 地形过于崎岖 | 不符合割草业务 | 增加专用坡地模式和最大坡度后验约束 |
| 碰撞过细 | Physics step 变慢 | 视觉网格与碰撞代理分离 |
| PointInstancer 语义不正确 | 分割数据不可用 | 原型/实例级语义专项测试 |
| 启动器修改源 USD | 资产被意外覆盖 | 将清理放到显式打包阶段 |

## 10. 推荐实施顺序

建议严格按以下顺序推进：

1. 固定 Isaac Sim 和 LARIAD commit；
2. 只接入 `easy.usd`，验证最小环境引用链；
3. 接入 `medium.usd`、`hard.usd`，解决材质、缺失引用和性能问题；
4. 将 `terrain.generator` 复制为独立派生扩展并保留许可证；
5. 完成 Isaac Sim 6.x 扩展启动与一次原样森林生成；
6. 增加配置文件驱动和 Headless 生成；
7. 增加割草专用地形、作业区 mask 与障碍物规则；
8. 完成 USD 打包器和启动器 `prim_path` 支持；
9. 生成三套首批草坪并进行物理、传感器和性能验收；
10. 最后再开展可割草视觉状态、GNSS 遮挡和动态障碍物等高级功能。

## 11. 第一迭代建议范围

第一迭代不要立刻重写全部生成器，建议范围控制为：

- 固定两个上游版本；
- 建立代码/资产分离目录；
- 让 LARIAD `easy.usd` 与 `hard.usd` 在当前项目中成功加载；
- 让 `terrain.generator` 在 Isaac Sim 6.x Extension Manager 中启动；
- 从 UI 生成一次 40 m × 40 m 地形并保存；
- 输出依赖完整的 `root.usda`；
- 车辆和四类传感器完成冒烟测试。

第一迭代通过后，再投入割草专用地形与植被规则。这样可以尽早暴露 Isaac Sim 版本和上游资产引用问题，避免在不稳定基础上开发业务功能。
