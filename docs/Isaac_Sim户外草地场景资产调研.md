# Isaac Sim 户外草地与割草机仿真场景候选方案调研

版本：V1.0  
调研日期：2026-08-09  
适用项目：`autoware_off-road_sim`  
目标平台：NVIDIA Isaac Sim / ROS 2

## 1. 调研目标

本文整理可用于构建高真实感割草机草坪场景的 6 类开源或可公开获取方案，重点考察：

- 草地、缓坡、树木、灌木、石块等户外资产的真实感；
- 是否提供 USD，或能否可靠地转换为 OpenUSD；
- 与 Isaac Sim 的集成成本；
- 许可证、资产来源和再分发风险；
- 大面积草坪中的渲染与物理性能；
- 是否适合割草路径规划、定位、感知和底盘动力学测试。

这里的“草地场景”分为两层：

1. **物理层**：连续地形网格、坡度、碰撞体、摩擦和可通行区域；
2. **视觉层**：草地 PBR 材质、草簇实例、树木、灌木、落叶、石块、围栏和天空光照。

真实场景不应让每根草都参与刚体碰撞。适合仿真的做法是让低复杂度地形承担物理碰撞，让高密度植被只承担渲染和传感器可见性。

## 2. 候选方案总览

| 编号 | 方案 | 类型 | 原生 USD | 真实感 | 集成工作量 | 许可证/来源清晰度 | 推荐用途 |
|---|---|---|---|---|---|---|---|
| 1 | LARIAD Offroad-Nav | 完整 Isaac Sim 越野场景 | 是 | 高 | 中 | 代码 MIT；内含资产需逐项复核 | 最快获得可用的户外基线 |
| 2 | Isaac Sim Procedural Forest Generator | 程序化森林/地形生成器 | 是 | 中高 | 中高 | MIT；素材来源仍需核验 | 自动生成缓坡、树木和岩石布局 |
| 3 | Infinigen Nature | 高质量程序化自然场景 | 可导出 OpenUSD | 很高 | 高 | BSD-3-Clause；外部资产另行核验 | 生成高多样性自然场景和数据集 |
| 4 | NVIDIA 官方 OpenUSD / SimReady 资产 | 官方资产与材质包 | 是 | 高 | 低至中 | 按各下载包许可证 | 稳定的材质、天空、建筑和道具补充 |
| 5 | Poly Haven | CC0 照片扫描资产、PBR、HDRI | 需转换 | 很高 | 中 | CC0 | 草地材质、草簇、石块和环境光照 |
| 6 | ambientCG + 轻量草簇资产 | CC0 PBR 与低面数模型组合 | 需转换 | 中高 | 中 | CC0 或 CC-BY，逐项确认 | 大面积实时草坪和性能优化 |

### 2.1 初步结论

- **最快形成可信场景**：方案 1 为基线，再用方案 5/6 替换或增强近景草地。
- **许可证最干净**：优先使用方案 5 的 CC0 资产和方案 6 中明确为 CC0 的内容。
- **场景多样性最高**：方案 3，但生成、转换和性能优化成本最高。
- **Isaac Sim 兼容风险最低**：方案 4，但官方库中未找到可直接替代完整割草草坪的单一场景。
- **推荐工程组合**：独立碰撞地形 + PBR 地表 + `UsdGeomPointInstancer` 草簇 + 少量高质量树木/石块 + HDRI/物理天空。

## 3. 方案一：LARIAD Offroad-Nav

### 3.1 项目与资源

- 项目主页：[Offroad-Nav](https://lariad.github.io/Offroad-Nav/)
- GitHub 仓库：[LARIAD/Offroad-Nav](https://github.com/LARIAD/Offroad-Nav)
- 场景资产目录：[assets](https://github.com/LARIAD/Offroad-Nav/tree/main/assets)
- 仓库许可证：MIT
- 项目声明环境：ROS 1 Noetic、Isaac Sim 4.5.0

项目提供 `easy.usd`、`medium.usd` 和 `hard.usd` 三档越野环境。其演示中的高难度环境包含起伏地形、高草和自然障碍物，是 6 个方案中最接近“下载后即可研究户外导航”的完整 Isaac Sim 场景。

### 3.2 实际资产检查结果

对仓库及其 `terrain.generator` 内容进行检查后，确认其中存在真实 USD 资产，而不是只有截图或占位文件，包括：

| 资产 | 作用 | 检查时文件大小（约） |
|---|---|---:|
| `assets/easy.usd` | 简单越野环境 | 333 KB |
| `assets/medium.usd` | 中等越野环境 | 1.69 MB |
| `assets/hard.usd` | 高难度越野环境 | 347 KB |
| `assets/barakuda.usd` | 车辆/平台相关资产 | 4.39 MB |
| `Grass_Short_B/Grass_Short_B.usd` | 短草模型 | 846 KB |
| `Douglas_Fir.usd` | 花旗松 | 3.62 MB |
| `Gray_Birch.usd` | 灰桦 | 33.75 MB |
| `Holly.usd` | 冬青 | 47.76 MB |
| `Norway_Spruce.usd` | 挪威云杉 | 35.16 MB |
| `Yew.usd` | 红豆杉 | 21.21 MB |
| `Rock_obj/Rock.usd` | 石块 | 732 KB |

仓库还包含森林地表纹理和 `autumn_park_2k.hdr` 等环境光资源。完整克隆体积较大，检查时约为 1.9 GB，其中 `terrain.generator` 接近 1 GB。

### 3.3 集成优势

- 已采用 USD 资产组织方式，与 Isaac Sim 的 Stage 工作流接近；
- 同时包含地形、草、树和石块，容易快速形成完整自然环境；
- 已用于机器人越野导航研究，可参考其相机、地形和障碍物配置；
- `easy/medium/hard` 可映射为割草机测试中的平地、缓坡和复杂边界三级场景。

### 3.4 已发现的兼容问题

1. 场景内部引用形如 `../isaac/extsUser/terrain.generator/data/...`，原项目通过安装脚本建立符号链接。直接复制单个 USD 会产生资源丢失。
2. `medium.usd` 与 `hard.usd` 存在对 `block.usd` 的缺失引用，需要替换或移除。
3. 材质引用包括 `OmniPBR.mdl` 和 NVIDIA 云材质，必须在 Isaac Sim 运行环境中解析。
4. `medium.usd` 和 `hard.usd` 没有设置 `defaultPrim`。如果当前加载器只调用 `AddReference(assetPath)` 而不提供 prim path，不能直接作为引用场景使用；需要增加包装层、设置 `defaultPrim`，或重新导出/展平 Stage。
5. 项目目标版本为 Isaac Sim 4.5.0，迁移到 Isaac Sim 6.x 时要验证扩展 API、材质和物理 schema。

### 3.5 许可证风险

仓库根许可证为 MIT，但仓库内树木、草地、纹理和 HDRI 的逐项来源及许可证没有完整清单。用于内部研发通常风险较低；若要把资产随产品、数据集或公开仓库重新分发，应先让维护者确认来源，或仅借鉴场景结构并换成来源明确的 CC0 资产。

### 3.6 对割草机项目的建议

不建议整仓直接复制进当前项目。更稳妥的方式是：

- 先将其作为外部依赖或独立资产包验证；
- 借鉴地形、树木和石块的分布方法；
- 评估 `Grass_Short_B.usd` 的视觉和性能后再决定是否引入；
- 给场景增加独立的低面数碰撞网格；
- 将越野高草密度降低，并增加连续、可割的短草坪区域。

## 4. 方案二：NVIDIA Isaac Sim Procedural Forest Generator

### 4.1 项目与资源

- GitHub 仓库：[joevento/Nvidia-Isaac-Sim-Procedual-Forest-Generator](https://github.com/joevento/Nvidia-Isaac-Sim-Procedual-Forest-Generator)
- 许可证：MIT
- 来源：University of Oulu 相关开源项目

该项目是面向 Isaac Sim 的程序化森林生成器，可生成地形并放置桦树、云杉、松树、石块和植被，还包含 HDRI、自动碰撞等处理。LARIAD 使用的 `terrain.generator` 与这一项目存在衍生关系。

### 4.2 集成优势

- 能用参数生成多套不同坡度、树密度和障碍物布局；
- 比手工摆放更适合批量构建训练与回归测试地图；
- 地形、植被和碰撞生成逻辑可作为当前项目场景生成脚本的参考；
- 可针对割草机定义“草坪核心区、树根禁入区、边界区、坡地区”等语义区域。

### 4.3 局限与风险

- 原始目标偏森林和越野，而不是修整过的住宅/公园草坪；
- 需要更新旧版 Isaac Sim 扩展 API，并验证 Isaac Sim 6.x 的可用性；
- 生成器代码采用 MIT，不代表所有随附纹理、树木和 HDRI 自动拥有相同许可证；
- 程序化生成后仍需做性能预算、LOD、碰撞简化和传感器验证；
- 如果每棵树、每块石头都创建复杂 collider，大地图物理性能会迅速下降。

### 4.4 适合割草机的改造方向

- 将森林密度参数改造成草坪覆盖率、树木稀疏度和障碍边距；
- 用低频高度场生成 0–12° 的大尺度缓坡，避免越野式尖锐噪声；
- 草簇使用 Point Instancer，不给每株草创建刚体碰撞；
- 生成导航真值：作业区 polygon、禁区、边界线、坡度图和地表类别；
- 固定随机种子，生成可复现的 CI 测试场景。

## 5. 方案三：Infinigen Nature

### 5.1 项目与资源

- GitHub 仓库：[princeton-vl/infinigen](https://github.com/princeton-vl/infinigen)
- Isaac Sim 文档：[使用 Infinigen 生成 USD 环境](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_infinigen_sdg.html)
- 许可证：BSD-3-Clause（第三方资源按各自声明）

Infinigen 是面向高真实感自然世界的程序化生成系统，能够生成地形、植物、岩石、水体和自然照明，并支持导出 OpenUSD。Isaac Sim 官方教程已给出加载 Infinigen 环境、配置 USD 与物理属性的工作流。

### 5.2 集成优势

- 六个方案中自然环境的视觉上限最高；
- 程序化内容多样性强，适合生成不同季节、植被、坡度和光照；
- 可用于相机感知、语义分割、深度估计和合成数据生成；
- 开源许可证相对清晰，便于二次开发生成流程。

### 5.3 局限与风险

- 生成依赖和计算开销大，不适合每次启动仿真时实时生成；
- 从 Blender/生成结果到可实时运行的 USD，需要纹理烘焙、材质转换和资产打包；
- 高精度自然网格通常不适合作为物理碰撞体，必须生成代理碰撞；
- 需要处理 LOD、实例化、纹理显存和光线追踪性能；
- 视觉资产中的细小枝叶可能给 LiDAR/深度相机带来大量噪点，需按传感器目标调整可见性。

### 5.4 推荐使用方式

将 Infinigen 作为**离线场景工厂**，而不是当前仓库的运行时依赖：

1. 离线生成 3–5 个草坪/公园风格环境；
2. 导出 OpenUSD，并把地形、植被和装饰物分层；
3. 对草和树做实例化与 LOD；
4. 另建简化碰撞地形；
5. 将优化后的 USD 包接入当前启动器。

该方案尤其适合后续构建相机训练数据，但如果当前目标只是尽快得到一个可信的割草测试场景，成本高于方案 1、5、6 的组合。

## 6. 方案四：NVIDIA 官方 OpenUSD、SimReady 与素材包

### 6.1 官方入口

- [Isaac Sim USD Assets Overview](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_overview.html)
- [NVIDIA SimReady 说明](https://www.nvidia.com/en-eu/glossary/simready/)
- [Isaac Sim Asset Browsers](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/utilities/browsers.html)
- [OpenUSD Downloadable Packs](https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html)

### 6.2 可用素材包

| 素材包 | 官方页面标注规模 | 内容 | 对草坪项目的价值 |
|---|---:|---|---|
| XR Samples | 约 5.3 GB | 82 个 OpenUSD 环境，含户外模板 | 获取场景组织、建筑和户外布局参考 |
| vMaterials 2 | 约 5.5 GB | 1,854 个写实 MDL 材质 | 地表、土壤、石材和建筑表面 |
| Environment Skies | 约 8.9 GB | 32 个 HDR 天空 | 阴天、晴天、日落等真实光照 |
| Sample Scenes | 约 26 GB | 441 个资产组成的主题场景 | 道具、建筑与 USD 组合示例 |

### 6.3 集成优势

- USD、MDL 和 Isaac Sim 工作流兼容性最好；
- SimReady 资产通常具有较规范的语义、材质和物理配置；
- 官方浏览器可以直接检查资产，而无需先写转换脚本；
- 适合作为天空、灯光、房屋、围栏、园林道具和通用材质的来源。

### 6.4 局限与核验结果

- 官方资料中未找到一个可以直接作为“高真实感割草机草坪”的完整场景；
- 素材包体积较大，不能仅为一两项材质整体纳入仓库；
- 资产的本地路径会随 Isaac Sim 版本、安装方式和 Nucleus 配置变化，应通过 Asset Browser 查询实际路径；
- 某第三方中文页面曾列出 `/Isaac/Environments/Agriculture/field_01.usd`，但对 Isaac Sim 6.0 官方 S3 路径进行检查时返回 404，官方资产文档也未列出该场景。因此不能把它作为已验证可用的官方农业草地资产。

### 6.5 推荐使用方式

- 先在目标 Isaac Sim 版本的 Asset Browser 中筛选 `grass`、`ground`、`park`、`garden`、`tree`、`rock`；
- 只记录和引用当前版本真实存在的资产路径；
- 对需要离线部署的资产使用官方许可允许的打包方式，不依赖不稳定的云端绝对路径；
- 用 vMaterials/天空包增强光照与地表，完整草坪结构仍由项目自己生成。

## 7. 方案五：Poly Haven CC0 资产

### 7.1 官方入口与许可证

- [Poly Haven License](https://polyhaven.com/license)
- 许可证：站内资产统一为 CC0，可商用、修改和再分发，仍建议保留来源清单。

### 7.2 与草坪相关的候选资产

| 资产 | 类型 | 页面信息 | 建议用途 |
|---|---|---|---|
| [Grass Medium 01](https://polyhaven.com/a/grass_medium_01) | 3D 草簇 | 约 200 万三角面、61.6 MB、最高 8K | 近景母体，必须减面与做 LOD |
| [Grass Medium 02](https://polyhaven.com/a/grass_medium_02) | 3D 草簇 | 约 100 万三角面、53.42 MB | 近景变化型，不能密集直接复制 |
| [Leafy Grass](https://polyhaven.com/a/leafy_grass) | PBR 地表 | 最高 8K | 覆盖地形主体，提供远景草地细节 |
| [Meadow](https://polyhaven.com/a/meadow) | HDRI | 最高 16K | 草地环境光与远景背景 |
| [Meadow 2](https://polyhaven.com/a/meadow_2) | HDRI | 多分辨率 | 第二套天气/光照条件 |
| [Dry Meadow](https://polyhaven.com/a/dry_meadow) | HDRI | 多分辨率 | 干燥季节或泛化测试 |

### 7.3 集成优势

- 来源和许可证最清晰，适合长期维护和公开发布；
- 照片扫描材质与 HDRI 能显著改善“塑料草地”和空洞背景；
- 可按需要下载 1K/2K/4K，不必强制使用超高分辨率；
- FBX/OBJ/Blend 与 PBR 贴图可通过 Blender 或 Omniverse 工具转换成 USD。

### 7.4 性能注意事项

Poly Haven 的草簇是离线渲染级资产。以百万级三角面的模型作为单个原型，再实例化几万次，仍会造成严重的几何、阴影和光追负担。建议：

- 从高模烘焙法线/透明贴图，制作 200–2,000 三角面的实时草簇；
- 至少设置近、中、远三级 LOD；
- 近处使用少量真实几何，中远处使用交叉面片或仅保留 PBR 地表；
- 用 `UsdGeomPointInstancer` 批量实例化，避免创建数万个独立 prim；
- 给机器人周围设置草簇加载半径，远处只显示地表材质；
- 草簇默认不创建 collider。

### 7.5 推荐定位

Poly Haven 最适合作为场景的**视觉质量核心**：用 PBR 地表建立连续草坪，用降模草簇增强近景，用 HDRI 提供自然光照。它不是现成的 Isaac Sim 完整场景，需要与项目的地形、碰撞和语义系统组合。

## 8. 方案六：ambientCG 与轻量草簇资产

### 8.1 ambientCG 的 CC0 PBR

- [Grass 001](https://ambientcg.com/view?id=Grass001)：草地 PBR 材质；
- [Ground 066](https://ambientcg.com/view?id=Ground066)：森林/自然地表扫描材质；
- ambientCG 站内资产采用 CC0，但仍应在下载时保存资产 ID 和许可证快照。

ambientCG 的优势是材质选择多、分辨率可控、许可证清晰；不足是多数草地资源只有表面材质，没有立体草叶几何。

### 8.2 轻量草簇模型候选

| 资产 | 许可证 | 复杂度/形式 | 评价 |
|---|---|---|---|
| [Sketchfab Grass Patches](https://sketchfab.com/3d-models/grass-patches-6952780b80594a31aab2dedf7249a47a) | CC-BY | 约 1.9k 三角面 | 适合作为实时实例原型，发布时需署名 |
| [Sketchfab Grass Variations](https://sketchfab.com/3d-models/grass-variations-c84573687bf14f89938002df4ca0e696) | CC-BY | 约 1.6k 三角面 | 有变化型，利于打破重复感 |
| [Sketchfab Realistic Grass Pack](https://sketchfab.com/3d-models/realistic-grass-pack-for-games-free-9b958d613e9a44dbba580748e7a1789c) | CC-BY | 约 794 三角面 | 性能友好，适合大面积实例化 |
| [Sketchfab 5m x 5m Grass Square](https://sketchfab.com/3d-models/5m-x-5m-grass-square-ab8ffde30e2f4a6b9da54fffaa584b39) | CC-BY | 约 40 万三角面/25 m² | 面数过高，不建议大面积铺设 |
| [BlendSwap Realistic Grass Field](https://blendswap.com/blend/17567) | CC0 | Blender 粒子系统，约 4.3 MB | 许可友好，但需实现粒子并优化 |
| [OpenGameArt Grass Patch](https://opengameart.org/content/grass-patch) | CC0 | `.blend`，含 LOD 思路 | 适合制作项目自有草簇原型 |
| [OpenGameArt Basic Grass](https://opengameart.org/content/basic-grass) | CC0 | Blender Hair | 不能直接用于实时仿真，需烘焙/转换 |

### 8.3 集成优势

- 可以组合“CC0 地表材质 + 低面数草簇”，同时控制法律风险和实时性能；
- 轻量草簇比扫描高模更适合 Point Instancer；
- 容易制作多种颜色、高度和干湿程度，用于域随机化；
- 可以只在摄像机近景和机器人路径两侧显示立体草。

### 8.4 局限与注意事项

- CC-BY 资产必须保留作者、链接和许可证署名，不能只记录下载站名称；
- Blender Hair/粒子系统必须先实现为网格或卡片，再导出 USD；
- 透明草叶需要检查 MDL 的 alpha cutout、双面渲染、阴影和 RTX LiDAR 表现；
- 不同站点的切线空间、法线方向和尺寸单位可能不一致，需要统一到米制和 Z-up；
- 下载页面信息可能变化，纳入仓库时应同时增加 `THIRD_PARTY_ASSETS.md`。

## 9. 面向割草机仿真的推荐组合

单独采用任一候选都不够理想。综合真实感、性能、授权和开发周期，推荐以下组合：

```text
低面数连续地形（自建，负责坡度和碰撞）
  ├─ CC0 草地 PBR（Poly Haven 或 ambientCG）
  ├─ 3–6 种低面数草簇（Point Instancer，只负责视觉）
  ├─ 少量树木、灌木、石块和落叶（独立 LOD）
  ├─ 围栏、花坛、充电站等割草业务障碍物
  ├─ HDRI + 物理太阳/天空
  └─ 作业区、禁区、边界、坡度和材质语义层
```

### 9.1 推荐优先级

1. **地形与物理**：保留项目自己的规则草坪地形生成能力，把坡度限制在割草机合理范围；
2. **地表质感**：使用 Poly Haven 或 ambientCG 的 CC0 PBR 替换纯色材质；
3. **立体草**：从 CC0/CC-BY 候选制作低面数原型，用 Point Instancer 非均匀散布；
4. **自然障碍物**：参考 LARIAD/森林生成器的布局方式，但优先替换为来源明确资产；
5. **环境光照**：引入 Meadow HDRI，并保留可调物理太阳支持时间变化；
6. **高级场景**：后续用 Infinigen 离线生成更复杂的公园或野外泛化测试集。

### 9.2 视觉真实性关键点

- 地形应平缓但不能完全平面，使用大尺度、低频起伏；
- 草地颜色至少有 3–5 种低幅随机变化，避免整片单色；
- 草簇分布应有疏密和裸土斑块，不使用完全规则网格；
- 草高靠近花坛、树根和围栏时增加，主作业区保持短草；
- 纹理比例必须符合真实尺寸，避免草叶被放大成灌木；
- 树木和道具必须产生合理的接触阴影；
- 远景使用 HDRI/环境几何，避免场景边界突然终止；
- 碰撞网格与视觉网格解耦，保证轮胎接触稳定。

### 9.3 性能预算建议

以下是起始建议，最终数值应以目标 GPU 和传感器频率实测为准：

- 连续地形：单个或少量分块 Mesh collider；
- 草簇原型：3–6 种，每种约 200–2,000 三角面；
- 近景草显示半径：约 15–30 m；
- 地表贴图：开发期 2K，近景必要时 4K；
- 树木：使用实例和 LOD，不使用高精度三角网格碰撞；
- 静态石块/树干：使用 convex hull 或简化代理碰撞；
- 仿真验收：同时测相机、LiDAR、RTX 渲染和物理步进，而不是只看空场景帧率。

## 10. 分阶段落地建议

### 阶段 A：许可清晰的真实草坪基线

- 下载一套 CC0 草地 PBR 和一套 HDRI；
- 制作 3 种低面数草簇 USD；
- 在当前缓坡地形上使用 Point Instancer；
- 添加树木、花坛、围栏和石块的简化障碍物；
- 完成材质、单位、轴向和碰撞检查。

### 阶段 B：户外多样性与传感器验证

- 引入湿草、干草、裸土和落叶材质变化；
- 增加晴天、阴天、逆光和傍晚 HDRI；
- 检查草叶 alpha 对 RGB、深度、语义分割和 RTX LiDAR 的影响；
- 构建平地、缓坡、树木稀疏区三套回归场景。

### 阶段 C：程序化生成与大规模数据

- 借鉴方案 2 生成可复现的坡度和障碍物布局；
- 使用方案 3 离线生产高多样性自然环境；
- 输出作业边界、禁区和真值地图；
- 将优化后的场景打包为独立 USD 资产包，不在运行时生成高复杂度内容。

## 11. 资产引入检查清单

每个外部资产进入仓库前至少记录：

- 原始名称、作者、下载页面和下载日期；
- 许可证全文或许可证页面快照；
- 是否允许修改、商用和再分发；
- 原始格式、转换工具和转换参数；
- 米制缩放、Up Axis、材质和纹理路径；
- 三角面数、纹理分辨率、LOD 和显存占用；
- collider 类型及是否参与 LiDAR/相机/语义传感器；
- Isaac Sim 目标版本和验证结果；
- 是否存在绝对路径、Nucleus 路径或缺失 MDL 依赖。

## 12. 最终选型建议

当前项目不宜直接把某个“森林场景”当成最终割草草坪。推荐采用以下路线：

> 以当前项目可控的平缓物理地形为底座，使用 Poly Haven/ambientCG 的 CC0 PBR 与优化草簇提升真实感，参考 LARIAD 和程序化森林生成器构建自然分布，再用 NVIDIA 官方资产补充天空、建筑和园林道具。Infinigen 留作后续高质量数据集与泛化场景生成。

如果追求最快演示，可先适配 LARIAD；如果准备长期公开发布或商业化，应优先采用 CC0-only 资产链，并为所有第三方内容建立独立资产清单。
