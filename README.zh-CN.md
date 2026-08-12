Autoware 越野模拟器

# Autoware-RoboRacer 越野模拟器

Autoware-RoboRacer 越野模拟器是一个面向 **Autoware** 和 **RoboRacer** 越野赛车的多车高保真仿真框架，基于 [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) 构建，并集成 **ROS 2 Humble**。本模拟器为 **Autoware Off-road** 项目开发，旨在促进越野及赛车场景 ODD 中自动驾驶算法的训练与评估。本研究是 Autoware 越野/赛车工作组与宾夕法尼亚大学 Autoware 卓越中心（CoE）持续合作的一部分。

本模拟器提供一辆配备完整传感器套件的 **1/5 比例 RoboRacer-Max** 模型，包括：**3D LiDAR**、**RGB 相机**、**GNSS**、**IMU** 和**里程计**。其中包含高保真的 **pumptrack_simple** 环境，其非平面地形专为具有挑战性的越野测试而设计。该框架通过通用控制接口支持**多车配置**，能够检测传入的 **Autoware** 和 **RoboRacer** 控制消息类型，同时通过 CycloneDDS 无缝支持**硬件在环（HIL）测试**。其他工具包括用于感知训练的**语义分割记录器**、用于手动控制的内置**键盘遥控**，以及易于使用的优化启动与配置系统，可在仿真保真度和实时性能之间实现良好平衡。

如需查阅完整的 Isaac Sim 文档，请访问 [Isaac Sim 文档](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/index.html)。

---

## 目录

1. [前置条件](#前置条件)
2. [Docker 设置](#docker-设置)
3. [运行仿真](#运行仿真)
4. [键盘控制](#键盘控制)
5. [用户界面](#用户界面)
6. [ROS 2 话题](#ros-2-话题)
7. [控制接口](#控制接口)
8. [多车设置](#多车设置)
9. [分布式模式与硬件在环（HIL）测试](#分布式模式与硬件在环hil测试)
10. [表面摩擦](#表面摩擦)
11. [语义分割数据集录制](#语义分割数据集录制)
12. [性能调优](#性能调优)
13. [实用脚本](#实用脚本)
14. [故障排查](#故障排查)
15. [许可证](#许可证)

---

## 前置条件

| 要求                     | 版本                                                                                                   |
| ------------------------ | ------------------------------------------------------------------------------------------------------ |
| NVIDIA GPU               | 推荐 RTX 4070+                                                                                         |
| NVIDIA 驱动              | 580+                                                                                                   |
| Docker                   | 24+                                                                                                    |
| NVIDIA-Container-Toolkit | [1.14.0+](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) |
| 主机操作系统             | Ubuntu 22.04+（任何支持 X11 和 NVIDIA 驱动的 Linux 发行版）                                            |
| 磁盘空间                 | 45GB+                                                                                                  |

---

## Docker 设置

该容器会从源码构建 [NVIDIA Isaac Sim](https://github.com/isaac-sim/IsaacSim) 和 ROS 2 Humble。首次构建需要 30–60 分钟。

### 1. 克隆仓库

```bash
git clone https://github.com/autowarefoundation/autoware_off-road_sim
cd autoware_off-road_sim
```

### 2. 构建 Docker 镜像

```bash
./docker/build.sh
```

> **版本固定：** `docker/Dockerfile` 会同时验证 `ISAAC_SIM_REF` 和完整的 `ISAAC_SIM_COMMIT`。若要有意选择其他版本，请同时更新这两个构建参数，并在提交更改前验证模拟器。依赖锁定信息记录在 [`third_party/lariad_versions.yaml`](third_party/lariad_versions.yaml) 中。

### LARIAD 地形资产

可选的 LARIAD 地形生成器及其大型环境资产不会存储在本仓库中，也不会复制到 Docker 构建上下文中。请将经过审计的上游版本拉取到已忽略的 `external_assets/` 目录：

```bash
./scripts/tools/fetch_lariad_assets.sh
```

该命令会在固定的 LARIAD 提交上执行稀疏检出，并验证最终版本。由于上游 MIT 许可证涵盖项目代码，但各资产的再分发条款尚不完整，因此这些资产目前仅获准用于**内部评估**。发布或再分发生成的场景前，请参阅 [`third_party/LARIAD_ASSET_MANIFEST.md`](third_party/LARIAD_ASSET_MANIFEST.md)。

进入 Isaac Sim 6.0.0 容器后，生成并验证非破坏性的兼容层：

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh \
  scripts/tools/prepare_lariad_scenes.py
```

生成的 `easy.usda`、`medium.usda` 和 `hard.usda` 文件会写入已忽略的 `external_assets/lariad_offroad_nav/compat/` 目录。该脚本会移除嵌入的 Barakuda 和损坏的 `block.usd` payload，重写旧版资产路径，补充 `defaultPrim`、米制/Z 轴向上的元数据，并验证环境中不含 `PhysicsScene`。它绝不会修改上游二进制 USD 文件。可使用 `--check` 在不重新生成文件的情况下再次运行验证。

### 3. 启动容器

```bash
./docker/run.sh
```

该脚本会自动：

- 将仓库挂载为 `/workspace/autoware_off-road_sim`
- 通过 `--runtime=nvidia` 启用 GPU 访问
- 转发 X11 以显示 GUI
- 将工作目录设置为 `/workspace/autoware_off-road_sim`

---

## 运行仿真

始终使用 Isaac Sim 自带的定制 Python 解释器——标准 `python3` 不包含 `omni.*` 软件包。

要在容器中运行 pumptrack_simple 环境下的 RoboRacer-Max 演示：

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_sim.py 
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_sim.py --headless --/persistent/UJITSO/geometry=false --/UJITSO/enabled=false
```

这将使用默认配置文件 `scripts/configs/pumptrack_simple.yaml` 运行仿真。

要使用其他配置文件，请添加 `--config` 标志：

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_sim.py --config <配置文件路径>
```

LARIAD 第一阶段单车冒烟测试配置如下：

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_lawn_mower_sim.py \
  --config scripts/configs/lariad_easy.yaml
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_lawn_mower_sim.py \
  --config scripts/configs/lariad_medium.yaml
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_lawn_mower_sim.py \
  --config scripts/configs/lariad_hard.yaml
```

`launch_lawn_mower_sim.py` 是一个专用启动器副本。草坪/LARIAD 特有行为被隔离在其中；原始的 `launch_sim.py` 及其现有环境启动行为保持不变。不使用 `--config` 时，该专用启动器默认使用 `lariad_easy.yaml`。

> **注意：** 首次启动时着色器编译需要几分钟。在此期间，Isaac Sim 可能看起来没有响应。

> **注意：** 启动时会出现两条 `[Error] [omni.physicsschema.plugin] Joint body relationship points to a non existent prim, joint will not be created.` 消息。这些错误不影响仿真，目前可以忽略。

启动脚本将会：

1. 加载环境 USD 资产
2. 按配置以正确的位置/朝向生成所有已启用车辆
3. 为每辆车重新映射所有传感器话题（多车模式下不会冲突）
4. 启动 ROS 2 桥接器
5. 通过 ActionGraph 注入键盘控制

### 无头模式

添加 `--headless` 可在没有显示窗口的情况下运行模拟器。这适用于**分布式模式与硬件在环测试**、**CI 流水线**以及未连接显示器的**远程/服务器部署**。

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_sim.py \
  --config scripts/configs/pumptrack_simple_config.yaml --headless
```

**与普通模式的区别：**

| 功能       | 普通模式                           | 无头模式                   |
| ---------- | ---------------------------------- | -------------------------- |
| 默认控制模式 | `KEYBOARD_CONTROL`                 | `ROS2_CONTROL`（所有车辆） |
| 键盘控制   | 可用（长按 `1`/`2` 切换）          | 不可用                     |

在无头模式下，所有车辆会直接以 **ROS2_CONTROL** 启动——无需长按 `1` 或 `2` 来启用 ROS 2 命令。启动时终端会打印：

```
[Headless] Running without display. Control mode: ROS2_CONTROL for ['Ego_Vehicle']
[Headless] Subscribe to drive commands via /ego/drive (AckermannDriveStamped) or /ego/control (autoware_control_msgs/Control)
```

启动后可立即向车辆发布命令：

```bash
ros2 topic pub --rate 15 /ego/control autoware_control_msgs/msg/Control \
  '{longitudinal: {velocity: 2.0, acceleration: 1.0}, lateral: {steering_tire_angle: 0.5}}'
```

所有传感器（`/ego/imu`、`/ego/odom`、`/ego/point_cloud`、`/ego/gnss` 等）都会继续正常发布。

### 将新终端附加到正在运行的容器

要为 RViz、ROS 2 命令等将新终端附加到正在运行的容器：

在容器外的 **autoware_off-road_sim** 目录中执行：

```bash
./docker/attach.sh
```

该脚本会自动加载 ROS 2 Humble 环境，并将工作目录设置为 `/workspace/autoware_off-road_sim`。

### 使用完整 UI 原生启动 Isaac Sim

编辑 USD 文件时，建议不使用启动文件，直接启动 Isaac Sim。

在容器内执行：

```bash
/root/isaacsim/_build/linux-x86_64/release/isaac-sim.sh
```

### 草坪地形生成器（阶段 2–3）

首先拉取固定版本的 LARIAD 资产，然后启动编辑器，并在 Isaac Sim 内置搜索目录中注册项目扩展：

```bash
./scripts/tools/fetch_lariad_assets.sh
./scripts/tools/install_lawn_generator.sh
/root/isaacsim/_build/linux-x86_64/release/isaac-sim.sh \
  --ext-folder /workspace/autoware-off-road_sim/extensions \
  --enable lawn.terrain.generator
```

安装程序只会在 Isaac Sim 的 `release/exts` 文件夹下创建符号链接。这可确保 Full 应用在冷启动依赖解析期间找到该扩展。重复运行安装程序是安全的。

打开 **Window > Lawn Terrain Generator**。资产发现功能会检查每个注册表项并报告不可用资产；在固定的公开检出版本中，`Switchgrass` 和 `Container` 会被禁用并显示警告。需要时可通过 `--/exts/lawn.terrain.generator/assetRoot=/absolute/path/to/data` 覆盖外部数据根目录。该窗口提供与上游类似的可滚动参数编辑器。使用 **Load Config** 从 YAML/JSON 填充控件，以交互方式编辑地形、区域、资产、表面、光照和真值输出，然后选择 **Generate All**。**Save Config** 会验证并持久化当前控件；生成本身不要求预先保存。

同一生成器可以从 YAML 以无头方式运行，无需点击 UI：

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh \
  scripts/tools/generate_lawn_scene.py \
  --config scripts/configs/lawn_generator_stage2.yaml \
  --output /tmp/lawn_generator_stage2.usda
```

阶段 3 增加了面向割草机的地形模式、坡度约束、作业/禁割区域、区域感知植被、业务对象和导航真值。UI 默认打开 Offroad-Nav 森林草坪预设。该预设将受约束的割草机作业区与固定版本的上游森林 PBR 纹理、HDRI、混合树木、林下植被和苔藓岩石结合起来。可在现有容器中以无头方式生成：

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/tools/generate_lawn_scene.py --config scripts/configs/lawn_generator_stage3_offroad.yaml --output assets/environments/lawn_generated/offroad_lawn_01/root.usda
```

#### 可生成的地形与模型

草坪地形生成器并不是从零创建复杂的三维模型，而是生成地形网格，并根据配置在场景中批量布置 USD 模型资产。当前支持以下内容。

**地形模型**

| 模式 | 说明 |
| ---- | ---- |
| `flat` | 平坦草坪 |
| `single_slope` | 单向坡地 |
| `rolling_lawn` | 起伏草坪 |
| `terraced_lawn` | 梯田式草坪 |
| `heightmap` | 根据高度图生成地形 |
| `legacy_fbm` | 与阶段 2 配置兼容的程序化噪声地形 |

高度图支持 `.npy`、`.csv`、`.txt`，以及当前 Isaac Sim 环境中的 Pillow 所支持的图像格式。还可以配置坡度、横坡、局部曲率、平滑次数，以及车辆出生点周围的平坦区域。

**内置植物与自然物体模型**

| 类别 | 模型 |
| ---- | ---- |
| 树木 | `Birch`（桦树）、`Spruce`（云杉）、`Pine`（松树）、`Holly`（冬青）、`Yew`（紫杉） |
| 灌木 | `Bush`（灌木）、`Blueberry`（蓝莓灌木） |
| 草地 | `Grass`（短草）、`Switchgrass`（柳枝稷/高草） |
| 岩石 | `Rock` |
| 人造物体 | `Container`（集装箱） |

当前固定版本的公开 LARIAD 资产中缺少 `Switchgrass` 和 `Container`，因此生成器会将其报告为不可用并安全跳过。阶段 3 越野预设通过基于短草资产的 `TallGrassFallback` 来模拟高草。

**区域与地表类型**

生成器可以划分可修剪草坪区域 `mowable_area`、禁割区域 `no_mow_zone`、边界缓冲林带 `boundary_buffer`、高草带 `tall_grass_band`、裸土斑块 `bare_soil_patch` 和割草机出生平台 `spawn_pad`。显式配置的区域支持圆形 `circle`、矩形 `rectangle` 和任意多边形 `polygon`。

每类模型可以限定在指定区域内生成。例如，短草可以只生成在可修剪区域，树木、灌木和岩石可以布置在边界或禁割区，高草可以布置在高草带，同时避免在裸土和出生平台上生成植被。各资产还可以配置生成密度、缩放范围、最小间距、语义标签、LiDAR 可见性以及是否贴合地表坡度。

**业务对象与自定义模型**

生成器支持创建 `box`（长方体）和 `cylinder`（圆柱体）形式的简单业务对象占位模型，并可设置名称、类别、位置、尺寸、朝向、碰撞体和语义标签。它们可用于表示围栏、树桩、柱子、设备、工作区障碍物或角色出生点等场景元素。

也可以通过 YAML 中的 `asset_manifest` 注册外部 USD/USDA 模型：

```yaml
asset_manifest:
  - name: Bench
    path: assets/bench.usda
    kind: object
    base_scale: 1.0
```

注册后，可在业务对象或分布式资产配置中引用该模型。因此，内置资产清单不是生成器的硬性上限；只要提供兼容的 USD 资产，就可以扩展座椅、围栏、工具棚、交通锥、树桩或其他园林设施。

相邻的 `truth/` 目录包含 `elevation.npy`、`slope_deg.npy`、全部六个区域掩码、ROS 兼容占用地图、作业区域 GeoJSON、语义标签、推荐生成位姿，以及种子/资产哈希清单。生成的场景输出会被 Git 有意忽略，并可根据版本化的 YAML 重新生成。

无需打开 UI，即可使用同一个固定验证相机进行渲染：

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/tools/render_lawn_preview.py --stage assets/environments/lawn_generated/offroad_lawn_01/root.usda --output assets/environments/lawn_generated/offroad_lawn_01/preview.png
```

使用 `--cycles 10` 执行所要求的生成/清理泄漏冒烟测试。执行完整验收测试时添加 `--validate-lifecycle`，该测试还会构造真实 UI 窗口，并验证扩展的禁用/重新启用。普通场景生成会跳过这一相对耗时的生命周期检查，并在 USD 和真值数据写入后立即退出。两种模式都会检查固定种子的高度场/对象数量，并验证没有生成 `PhysicsScene`。Python 依赖声明在 `extensions/lawn.terrain.generator/requirements.txt` 中，并由 Dockerfile 安装到 Isaac Sim 自带的 Python 中。

---

## 键盘控制

启动后，**点击 Isaac Sim 视口内部**使其获得焦点。

| 按键            | 操作                                                             |
| --------------- | ---------------------------------------------------------------- |
| `W` / `↑`       | 向前加速                                                         |
| `S` / `↓`       | 向后加速                                                         |
| `A` / `←`       | 向左转向                                                         |
| `D` / `→`       | 向右转向                                                         |
| `Space`         | 制动（按住时将速度和转向命令设为零）                             |
| `~`             | 将相机视图切换到透视视图                                         |
| `1`             | 将相机视图切换到主车辆                                           |
| `2`             | 将相机视图切换到对手车辆                                         |
| 长按 `1`（≥1 秒） | 在 **KEYBOARD_CONTROL** 与 **ROS2_CONTROL** 模式间切换主车辆      |
| 长按 `2`（≥1 秒） | 在 **KEYBOARD_CONTROL** 与 **ROS2_CONTROL** 模式间切换对手车辆    |
| `Backspace`     | 重新启动仿真                                                     |
| `/`             | 打开/关闭视口 HUD                                                |
| `R`             | 开始/停止录制分割数据集                                          |

每辆车都有独立的控制模式，可通过长按 `1` / `2` ≥1 秒切换：

- **KEYBOARD_CONTROL** — 通过键盘驾驶车辆（WASD / 方向键）。忽略 drive/control 话题上的 ROS 2 命令。
- **ROS2_CONTROL** — 车辆完全由传入的 `autoware_control_msgs/Control` 或 `AckermannDriveStamped` 消息驾驶。忽略键盘输入。如果没有收到新的 ROS 2 命令，车辆会保持速度 0。

> **注意：** 使用 `Backspace` 启动或重新启动仿真后，由于 TF 时间戳不匹配（警告：TF_OLD_DATA），预配置的 LiDAR 点云话题可能会停止在 RViz 中更新。点击 RViz 窗口左下角的 `Reset` 按钮，LiDAR 点云话题就会重新开始更新。

---

## 用户界面

### 视口 HUD

每辆启用的车辆在其视口左上角都有一张半透明 HUD 卡片，用于显示实时遥测信息：

- **标题行** — 车辆名称；蓝色表示当前已选中（键盘焦点）
- **CONTROL** — 当前模式（`KEYBOARD` 或 `ROS2`）
- **Speed / Steer** — 指令速度（m/s）和转向角（°）
- **ODOMETRY** — 位置（X/Y/Z）、线速度、角速度
- **IMU** — 线加速度、角加速度
- **GNSS** — 纬度和经度

按 `/` 可打开或关闭所有 HUD 窗口。

### 终端状态

终端会原地更新两行内容（每辆车一行），显示当前控制模式、指令速度、转向角和实时因子：

```
[KBD] Ego_Vehicle: Spd=+2.50 m/s  Str=+15.3°  | RT=98%
[ROS] Opponent_Vehicle: Spd=+3.10 m/s  Str=-4.2°
```

### 分屏模式

当配置中设置 `split_screen: true` 且两辆车都已启用时，Isaac Sim 会打开两个并排视口：

| 视口 | 显示内容 |
| ---- | -------- |
| 左侧 | 主车辆   |
| 右侧 | 对手车辆 |

在分屏模式下：

- **WASD 键**控制**左侧**视口中显示的车辆；**方向键**控制**右侧**视口中的车辆。
- 点击**左侧**或**右侧**视口使其获得焦点。按 `1` / `2` 可在所选视口中将车辆切换为主车辆或对手车辆。
- 仍可长按 `1` / `2` 独立切换每辆车的控制模式。
- 按 `~` 返回自由透视相机。

---

## ROS 2 话题

### 话题重映射

启动时，会在 `topics_to_remap` 中的话题前添加每辆车的 `topic_prefix`（例如 `/imu` → `/ego/imu`）。`frame_ids_to_remap` 中的坐标系 ID 会改写为 `<prefix>/<frame>`（例如 `odom` → `ego/odom`）。无需修改 USD 文件。

```yaml
# 写入 USD 的话题，会使用每辆车的 topic_prefix 进行重写。
topics_to_remap:
  - "/drive"
  - "/imu"
  - "/odom"
  - "/point_cloud"
  - "/rgb"

# 写入 USD 的 TF 坐标系 ID，会重写为 <prefix>/<frame>。
frame_ids_to_remap:
  - "odom"
  - "base_link"
```

### 驾驶命令（重映射后）

每辆车提供两个独立的命令话题——每种消息类型各一个。车辆必须处于 **ROS2_CONTROL 模式**（长按 `1` 或 `2` ≥1 秒），才会响应任一话题。

| 车辆 | 话题                | 消息类型                               | 技术栈              |
| ---- | ------------------- | -------------------------------------- | ------------------- |
| 主车 | `/ego/control`      | `autoware_control_msgs/Control`        | Autoware            |
| 主车 | `/ego/drive`        | `ackermann_msgs/AckermannDriveStamped` | RoboRacer / F1TENTH |
| 对手 | `/opponent/control` | `autoware_control_msgs/Control`        | Autoware            |
| 对手 | `/opponent/drive`   | `ackermann_msgs/AckermannDriveStamped` | RoboRacer / F1TENTH |

### 传感器输出（重映射后）

| 传感器 | 主车话题           | 对手话题                | 消息类型                  |
| ------ | ------------------ | ----------------------- | ------------------------- |
| IMU    | `/ego/imu`         | `/opponent/imu`         | `sensor_msgs/Imu`         |
| 里程计 | `/ego/odom`        | `/opponent/odom`        | `nav_msgs/Odometry`       |
| LiDAR  | `/ego/point_cloud` | `/opponent/point_cloud` | `sensor_msgs/PointCloud2` |
| 相机   | `/ego/rgb`         | `/opponent/rgb`         | `sensor_msgs/Image`       |
| GNSS   | `/ego/gnss`        | `/opponent/gnss`        | `sensor_msgs/NavSatFix`   |
| TF     | `/tf`              | `/tf`                   | `tf2_msgs/TFMessage`      |

### 控制模式状态

每辆车都通过 `std_msgs/Int32` 话题发布其当前控制模式：

| 车辆 | 话题                     | 值                                             |
| ---- | ------------------------ | ---------------------------------------------- |
| 主车 | `/ego/control_mode`      | `0` = `KEYBOARD_CONTROL`，`1` = `ROS2_CONTROL` |
| 对手 | `/opponent/control_mode` | `0` = `KEYBOARD_CONTROL`，`1` = `ROS2_CONTROL` |

每当控制模式发生变化（长按 `1` / `2` ≥1 秒）、仿真启动以及重新启动（`Backspace`）后，该话题都会立即更新。

---

## 控制接口

模拟器接受来自 **Autoware** 和 **RoboRacer / F1TENTH** 技术栈的驾驶命令。每个技术栈都发布到各自专用的话题——无需更改配置。

### 工作原理

启动时，`launch_sim.py` 会生成一个 Python 3.10 子进程（`isaacsim_drive_bridge`，可在 `ros2 node list` 中看到），该进程为每辆车注册**两个订阅**：

- `/ego/drive` — `AckermannDriveStamped`：直接转发给车辆内置的 OmniGraph Ackermann 控制器。
- `/ego/control` — `autoware_control_msgs/Control`：桥接器提取 `longitudinal.velocity` + `lateral.steering_tire_angle`，并将其作为 `AckermannDriveStamped` 重新发布到 `/ego/drive`，使 OmniGraph 控制器接收到该命令。

长按 `1`（主车）/ `2`（对手）≥1 秒，可为每辆车设置**控制模式**：

- **KEYBOARD_CONTROL 模式** — 应用键盘输入；忽略 ROS 2 命令
- **ROS2_CONTROL 模式** — 应用 ROS 2 命令；忽略键盘输入

### 第 1 步——将车辆切换到 ROS2_CONTROL 模式

点击 Isaac Sim 视口内部，然后长按 `1`（主车）或 `2`（对手）≥1 秒，直到 HUD 显示从 `CONTROL:KEYBOARD` -> `CONTROL:ROS2`。

当前控制源会显示在终端状态行中：

```
[KBD] Ego_Vehicle: Spd=+0.00 m/s  Str=+0.0°  | RT=88%
[Control] Ego_Vehicle → ROS2_CONTROL
[ROS] Ego_Vehicle: Spd=+0.00 m/s  Str=+0.0°  | RT=89%
```

### 第 2 步——发布 Autoware 或 RoboRacer 控制命令

在容器内的另一个终端中（参见**将新终端附加到正在运行的容器**）：

```bash
ros2 topic pub --rate 15 /ego/control autoware_control_msgs/msg/Control \
  '{longitudinal: {velocity: 2.0, acceleration: 1.0}, lateral: {steering_tire_angle: 0.5}}'
```

或者

```bash
ros2 topic pub --rate 15 /ego/drive ackermann_msgs/msg/AckermannDriveStamped \
  '{drive: {speed: 2.0, steering_angle: 0.5}}'
```

---

## 多车设置

两辆车可以使用**同一个 USD 资产**（`roboracer_max.usd`）。启动脚本通过 `topic_prefix` 自动处理所有话题隔离。

```yaml
vehicles:
  - name: "Ego_Vehicle"
    enabled: true
    asset: "assets/vehicles/roboracer_max.usd"
    topic_prefix: "/ego"
    spawn_position: [0.0, 0.0, 0.0]
    spawn_orientation: [0.0, 0.0, -90.0]
    enable_camera: true
    enable_lidar: true
    enable_gnss: true

  - name: "Opponent_Vehicle"
    enabled: false
    asset: "assets/vehicles/roboracer_max.usd"
    topic_prefix: "/opponent"
    spawn_position: [-2.0, 0.0, 0.0]
    spawn_orientation: [0.0, 0.0, -90.0]
    enable_camera: true
    enable_lidar: true
    enable_gnss: true
```

---

## 分布式模式与硬件在环（HIL）测试

此设置通过**局域网或 WiFi**，将运行 **Autoware** 或 **RoboRacer** 自动驾驶技术栈的远程 **PC** 或 **Jetson**，连接到在**仿真 PC** 上运行的 Isaac Sim 环境。这样可使自动驾驶技术栈与仿真环境运行在不同机器上，将仿真 PC 的资源留给仿真使用。

> **注意：** 支持 WiFi，但会引入不稳定的延迟。对于高频传感器流（LiDAR、相机），请使用专用的 5 GHz 接入点，或优先使用有线以太网。

该技术栈使用 **CycloneDDS**（`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`），与 Autoware 的默认中间件一致。CycloneDDS 通过多播自动发现对等节点——无需发现服务器。

#### 网络拓扑

```
┌──────────────────────────────────┐         ┌──────────────────────────────────┐
│           PC（仿真）             │         │  PC 或 Jetson（自动驾驶技术栈）  │
│                                  │         │                                  │
│  Isaac Sim（CycloneDDS）         │◄────────│  Autoware / RoboRacer 技术栈     │
│  ├─ 发布 /ego/imu                │ 局域网/ │  ├─ 订阅 /ego/imu                │
│  ├─ 发布 /ego/odom               │  WiFi   │  ├─ 订阅 /ego/odom               │
│  ├─ 发布 /ego/point_cloud        │         │  ├─ 订阅 /ego/point_cloud        │
│  ├─ 发布 /ego/gnss               │         │  ├─ 订阅 /ego/gnss               │
│  ├─ 订阅 /ego/control            │────────►│  └─ 发布 /ego/control            │
│  └─ 订阅 /ego/drive              │         │     (autoware_control_msgs)      │
│                                  │         │  或发布 /ego/drive               │
│  自动多播发现                    │         │     (AckermannDriveStamped)      │
│  （无需服务器）                  │         │  RMW_IMPLEMENTATION=             │
└──────────────────────────────────┘         │    rmw_cyclonedds_cpp            │
                                             └──────────────────────────────────┘
```

#### 第 1 步——启动模拟器（无需额外配置）

默认的 `pumptrack_simple_config.yaml` 已包含正确设置：

```yaml
network_setup:
  ros2_domain_id: 0
  network_interface: "auto"   # "auto" = CycloneDDS 多播；设为 "eth0" 或 "192.168.x.x" 可固定网卡
```

照常启动即可。CycloneDDS 会自动启动，并显示：

```
[ROS2] RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  ROS_DOMAIN_ID=0
[ROS2] CycloneDDS using automatic interface/multicast discovery
```

> **多网卡主机：** 如果仿真 PC 同时使用以太网和 WiFi，请设置 `network_interface: "eth0"`（或相应的接口名称/IP），以确保 CycloneDDS 绑定到正确的适配器。

#### 第 2 步——配置远程 PC 或 Jetson

CycloneDDS 使用 UDP 多播自动发现同一子网中的对等节点。启动自动驾驶技术栈前，只需设置两个环境变量：

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 然后启动技术栈（示例）：
ros2 launch f1tenth_stack bringup_launch.py
```

请确保仿真 PC 和远程 PC/Jetson 上的 `ROS_DOMAIN_ID` 相同。不同的仿真实例应使用不同的 `ROS_DOMAIN_ID`。

#### 第 3 步——验证连接

在远程 PC 或 Jetson 上：

```bash
ros2 topic list                 # 应显示 /ego/imu、/ego/odom 等
ros2 topic hz /ego/point_cloud # 验证 LiDAR 数据正从 PC 传输
ros2 topic echo /ego/drive     # 验证技术栈正在发布命令
```

在仿真 PC 上：

```bash
ros2 topic echo /ego/drive     # 应显示 Jetson 发送的命令
```

> **防火墙注意事项：** CycloneDDS 使用 UDP 多播（239.255.0.1）和单播。请确保 UDP 端口未被阻止：`sudo ufw allow from <远程 IP>`，或在局域网接口上禁用防火墙。

---

## 表面摩擦

环境中每种物理材质的摩擦值在配置文件的 `environment.frictions` 下配置。每个条目通过 prim 名称匹配材质，并设置 `dynamic_friction` 和 `static_friction`：

```yaml
environment:
  asset: "assets/environments/pumptrack_simple.usd"
  frictions:
    - name: "TirePhysicsMaterial"
      dynamic_friction: 1.0
      static_friction: 1.0
    - name: "AsphaltPhysicsMaterial"
      dynamic_friction: 0.7
      static_friction: 0.9
    - name: "GrassPhysicsMaterial"
      dynamic_friction: 0.4
      static_friction: 0.6
```

这些值在启动时应用到场景，并覆盖 USD 资产中已有的值。摩擦力计算基于两个接触材质摩擦值的平均值。

---

## 语义分割数据集录制

模拟器可在驾驶过程中生成配对的 RGB + 语义分割数据集，用于训练感知模型。

### 语义分割图像设置

```yaml
semantic_segmentation:
  id_keywords:
    0: ["default"]      # 兜底标签（不可行驶）
    1: ["track"]        # 可行驶表面——通过 prim 名称关键字匹配
  color_map:
    0: [61, 93, 255]    # 不可行驶 → 蓝色
    1: [0, 255, 220]    # 可行驶 → 青色
  capture_frequency: 8  # 每隔 (120Hz / 8Hz) = 15 帧
  image_resolution: [1280, 720]
  images_dir: "data/segmentation/pumptrack_simple/pumptrack_simple/images"
  gt_masks_dir: "data/segmentation/pumptrack_simple/pumptrack_simple/gt_masks"
  overwrite_existing: false
```

### 工作原理

- 在视口中按 `R` **开始**录制。再次按 `R` **停止**录制。
- 每次捕获都会以相同的补零文件名保存两个文件：
  - `images/000000.png` — 来自主车辆彩色相机的 RGB 图像
  - `gt_masks/000000.png` — 颜色编码的语义分割掩码
- 捕获频率、输出路径、标签类别和颜色均在配置的 `semantic_segmentation` 下设置。
- 启用 `overwrite_existing` 可将帧计数器重置为 0，并从头开始保存图像。

### 语义标注

通过将环境网格 prim 的名称与 `id_keywords` 列表匹配来进行标注。对于不匹配更具体关键字的任何 prim，`"default"` 关键字充当兜底项。

### 输出结构

```
data/
└── segmentation/
    └── pumptrack_simple/
        └── pumptrack_simple/
            ├── images/
            │   ├── 000000.png
            │   ├── 000001.png
            │   └── ...
            └── gt_masks/
                ├── 000000.png
                ├── 000001.png
                └── ...
```

> **注意：** `data/` 目录已被 Git 忽略。图像以所有用户可读的权限（`0o666`）保存，因此在 Docker 中以 root 身份运行时，仍可从主机访问。

### 语义分割图像示例

- 不可行驶 → 蓝色
- 可行驶 → 青色

语义分割图像

---

## 性能调优

模拟器会在终端状态行中打印实时因子（`RT=X%`）。`RT=100%` 表示仿真跟得上实时速度。低于 100% 表示仿真运行速度慢于实时。

```
[KEYBOARD_CONTROL] ACTIVE: Ego_Vehicle | Spd=+0.00 m/s, Str=+0.0° | RT=97%
```

所有调优参数都位于配置文件的 `physics_settings` 和 `graphics_settings` 块中。

---

### 物理设置

```yaml
physics_settings:
  time_steps_per_second: 60       # 物理更新频率（Hz）
  solver_position_iterations: 8   # 每个关节系统的位置求解迭代次数
  solver_velocity_iterations: 2   # 每个关节系统的速度求解迭代次数
```

#### `time_steps_per_second`

设置物理子步频率。每次调用 `simulation_app.update()` 都会将仿真推进 `1 / time_steps_per_second` 秒。如果 GPU/CPU 无法在相应的实际时间预算内完成一次更新，RT% 会降至 100% 以下。

| 值                    | 效果                                                     |
| --------------------- | -------------------------------------------------------- |
| 较高（例如 120 Hz）   | 碰撞/悬架更准确；CPU 开销更高，RT% 更低                  |
| 较低（例如 55 Hz）    | CPU 开销更低；适合单车使用，RT% 更高                     |
| 过低（< 30 Hz）       | 出现明显物理伪影（穿透、抖动），车辆可能解体             |

**建议起点：** 单辆遥控车在平滑赛道上使用 `55`–`60` Hz。

#### `solver_position_iterations`

控制 PhysX 每个时间步为每辆关节式车辆优化关节位置、接触穿透和悬架几何结构的次数。

| 值             | 效果                                   |
| -------------- | -------------------------------------- |
| `16`（默认）   | 最准确；CPU 开销最高                   |
| `8`            | 适合使用简单旋转关节的遥控车，平衡良好 |
| `4`            | 较激进；低速时在平滑地形上稳定         |
| `< 4`          | 关节漂移、车轮穿模，可能发生解体       |

#### `solver_velocity_iterations`

控制 PhysX 用于解析摩擦、恢复系数和速度阻尼的迭代次数。

| 值           | 效果                                     |
| ------------ | ---------------------------------------- |
| `4`（默认） | 完整精度；摩擦响应正确                   |
| `2`          | 摩擦精度略有降低；开销更小               |
| `1`          | 开销最低；急转弯时可能出现横向滑动       |

#### 典型配置

| 场景             | `time_steps_per_second` | `pos_iters` | `vel_iters` |
| ---------------- | ----------------------- | ----------- | ----------- |
| 单车、平滑赛道   | 55–60                   | 8           | 2           |
| 两辆车           | 50–55                   | 8           | 2           |
| 最高保真度       | 120                     | 16          | 4           |
| 最高性能         | 40–50                   | 4           | 1           |

> **注意：** `solver_type` 在代码中永久设为 `"PGS"`。TGS 更快，但在赛车场景中非常不稳定，车辆很容易解体。

---

### 图形设置

```yaml
graphics_settings:
  render_resolution: [2560, 1440]
  enable_DLSS_FPS_Multiplier_x2: false
  disable_shadows: false
  disable_ambient_occlusion: true
  disable_reflections: true
```

#### `render_resolution`

RTX 渲染器每帧生成的分辨率。降低该值可以提升 GPU 性能。

| 分辨率          | GPU 开销 |
| --------------- | -------- |
| `[3840, 2160]`  | 非常高   |
| `[2560, 1440]`  | 高       |
| `[1920, 1080]`  | 中       |
| `[1280, 720]`   | 低       |

#### `enable_DLSS_FPS_Multiplier_x2`

启用 NVIDIA DLSS 超分辨率（性能模式与 DLSS-G 2× 帧生成相结合）。帧生成需要 RTX 40 系列 GPU。在受支持的硬件上，它几乎可以将感知帧率提高一倍，而视觉质量损失很小。但是，它可能会降低 RT%。

#### `disable_shadows` / `disable_ambient_occlusion` / `disable_reflections`

禁用阴影、环境光遮蔽和反射有助于提升 GPU 性能。

## 实用脚本

| 脚本                      | 用途                                                                                                                                                                                                                                                 |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `scripts/launch_sim.py`   | 主仿真启动器（加载 USD、生成车辆、启动键盘控制、录制分割数据）                                                                                                                                                                                       |
| `scripts/gnss_bridge.py`  | 通过 rclpy 发布 `sensor_msgs/NavSatFix` 的 Python 3.10 子进程（自动生成）                                                                                                                                                                             |
| `scripts/drive_bridge.py` | Python 3.10 子进程，订阅每辆车的 `AckermannDriveStamped` 和 `autoware_control_msgs/Control` 驾驶话题，并将命令转发给 Isaac Sim OmniGraph 控制器；还会发布 `/map`、静态 TF 和每辆车的 `control_mode` 状态（`std_msgs/Int32`）（自动生成） |

---

## 故障排查

### 车辆不移动

1. 点击 Isaac Sim 视口内部，使其获得键盘焦点。
2. 检查终端中是否显示 `[Teleop] SUCCESS: Control path initialized for Ego_Vehicle`。
3. 如果没有显示，ActionGraph 路径可能已更改——检查场景的 OmniGraph 节点，并更新 `launch_sim.py` 中的搜索字符串。

### 外部 ROS 2 驾驶命令无效

1. **先切换到 ROS2_CONTROL 模式。** 在视口内长按 `1`（主车）或 `2`（对手）≥1 秒。终端状态行必须显示 `[ROS2_CONTROL]`，而不是 `[KEYBOARD_CONTROL]`。
2. **确认桥接器正在运行。** 启动后，`ros2 node list` 必须包含 `/isaacsim_drive_bridge`。如果缺失，请检查 `scripts/drive_bridge.log` 中的错误。
3. **检查启动日志。** 终端应打印类似以下内容：
   ```
   [drive_bridge] Ego_Vehicle: subscribed '/ego/drive' [AckermannDriveStamped]
   [drive_bridge] Ego_Vehicle: subscribed '/ego/control' [autoware_control_msgs/Control] → republish on '/ego/drive'
   ```
   如果缺少 `autoware_control_msgs/Control`，则未安装该软件包——请确认 Docker 构建期间已安装 `ros-humble-autoware-control-msgs`。
4. **使用正确的话题。** Autoware 技术栈应发布到 `/ego/control`（`autoware_control_msgs/Control`）。RoboRacer/F1TENTH 技术栈发布到 `/ego/drive`（`AckermannDriveStamped`）。
5. **检查状态行。** 收到实时命令后，它会从 `[ROS2_CONTROL]` 切换为 `[ACKERMANN]` 或 `[AUTOWARE]`。

### 分割图像未保存

- 启动后检查终端是否显示 `[Segmentation] Setup complete`。如果显示 `seg_enabled=False`，请检查其上方的设置错误。
- 确认在视口获得焦点时按下 `R`。
- 确认 `data/` 下存在 `images_dir` 和 `gt_masks_dir` 路径——启动时会自动创建。
- 在 Docker 内（以 root 身份运行）保存的图像权限为 `0o666`，应可从主机读取。

### ROS 2 话题不可见

- 确认 Isaac Sim 与终端之间的 `ros2_domain_id` 一致（默认：`0`）。
- 如果使用 CycloneDDS（默认），请确保**两台**机器都设置了 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，并且位于启用了多播的**同一子网**。在终端输出中查找 `[ROS2] RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 以确认。
- 检查两台机器上的 `ROS_DOMAIN_ID` 是否相同（默认：`0`）。

### GNSS 话题未出现

- 确认配置顶层设置了 `gnss.enabled: true`，并且车辆上设置了 `enable_gnss: true`。
- 检查启动时终端是否显示 `[GNSS] <vehicle>: NavSatFix publisher on '/ego/gnss'`。如果显示 `[GNSS] Setup error:`，请检查 `scripts/gnss_bridge.log`，查看 Python 3.10 桥接进程的完整错误。
- 桥接器需要 `/usr/bin/python3.10` 和位于 `/opt/ros/humble` 的 ROS Humble 软件包。Docker 镜像中已包含二者。
- 验证话题是否活跃：`ros2 topic hz /ego/gnss`（预期约 10 Hz）。

### `python.sh: No such file or directory`

Isaac Sim 构建输出位于：

```
/root/isaacsim/_build/linux-x86_64/release/python.sh
```

如果缺失，构建可能尚未完成。请重新运行 `./docker/build.sh`。

---

## 许可证

请参阅 [LICENSE](./LICENSE)。
