# LARIAD Offroad-Nav 第三方资产清单与使用策略

审计日期：2026-08-09  
审计对象：[LARIAD/Offroad-Nav](https://github.com/LARIAD/Offroad-Nav)  
固定 commit：`69640cf19eec1d90214ed4a3615788dd5dc1e3c4`  
生成器版本：`terrain.generator` 2.0.0

## 1. 结论与当前授权边界

LARIAD Offroad-Nav 根仓库声明 MIT 许可证，许可证副本保存在 [LARIAD_OFFROAD_NAV_LICENSE.txt](LARIAD_OFFROAD_NAV_LICENSE.txt)。该许可证可以作为项目代码及明确由项目作者发布内容的使用依据，但 `terrain.generator/data` 内包含从 NVIDIA Omniverse Content、照片扫描或其他来源收集的模型、纹理和 HDRI；根 MIT 文件没有逐项说明这些第三方素材的授权来源和再分发条件。

因此当前项目采用以下策略：

- **生成器源代码**：可以在保留版权和 MIT 许可证的前提下研究、修改和派生；
- **LARIAD 场景与视觉资产**：当前仅批准内部研发和兼容性验证；
- **对外发布、产品交付或数据集再分发**：在逐项取得明确授权前禁止携带这些资产；
- **长期方案**：把未确认资产逐步替换为许可证明确的 CC0/自有资产；
- 本文件是工程审计记录，不构成法律意见。

## 2. 审计范围与方法

本次检查了固定 commit 中：

- `assets/` 下的 LARIAD 保存场景；
- `terrain.generator/config/extension.toml`；
- `terrain.generator/terrain/generator/generator_logic.py` 的资产注册表；
- `terrain.generator/data` 的全部 163 个文件；
- 六个 `.collect.mapping.json` 中的原始下载 URL 和哈希记录；
- 顶层目录大小、文件数量及可复现的目录内容摘要。

`terrain.generator/data` 总大小为 `1,028,189,844` bytes（约 980.56 MiB）。该仓库没有把这些大文件交给当前项目的主 Git 历史管理。

## 3. 状态定义

| 状态 | 含义 | 当前项目允许行为 |
|---|---|---|
| `CODE_MIT` | 上游明确以 MIT 发布的代码/文档 | 可修改和再分发，但必须保留许可证 |
| `SOURCE_IDENTIFIED_LICENSE_UNVERIFIED` | 找到原始下载 URL，但未找到随文件提供的逐项许可证 | 仅内部验证 |
| `SOURCE_UNKNOWN` | 仓库中没有可确认的来源或许可证记录 | 仅内部验证，优先替换 |
| `MISSING` | 代码引用资产，但固定 commit 不包含文件 | 禁用相关功能或使用合规资产替换 |

## 4. `terrain.generator/data` 顶层资产清单

目录摘要的计算方式为：在对应目录内按相对路径排序，对每个文件执行 SHA-256，再对排序后的校验结果计算一次 SHA-256。摘要用于发现意外变化，不替代上游 Git commit。

| 资产组 | 主要入口/内容 | 文件数 | 大小 | 来源证据 | 状态 | 再分发 |
|---|---|---:|---:|---|---|---|
| `Blueberry_obj` | `Blueberry.usd`、OBJ、叶片/树干纹理 | 6 | 4.92 MiB | 无 `.collect.mapping.json` | `SOURCE_UNKNOWN` | 禁止，待核验 |
| `Bush_obj` | `Bush.usd`、OBJ、枝叶/树皮纹理 | 19 | 74.56 MiB | 无 `.collect.mapping.json` | `SOURCE_UNKNOWN` | 禁止，待核验 |
| `Douglas_Fir` | `Douglas_Fir.usd`、树皮和针叶材质 | 8 | 16.82 MiB | `.collect.mapping.json` 指向 NVIDIA Omniverse Content S3 `/Assets/Vegetation/Trees/` | `SOURCE_IDENTIFIED_LICENSE_UNVERIFIED` | 禁止，待核验 NVIDIA 内容条款 |
| `Grass_Short_B` | `Grass_Short_B.usd`、lawngrass PBR | 6 | 27.30 MiB | `.collect.mapping.json` 指向 NVIDIA Omniverse Content S3 `/Assets/Vegetation/Shrub/` | `SOURCE_IDENTIFIED_LICENSE_UNVERIFIED` | 禁止，待核验 NVIDIA 内容条款 |
| `Gray_Birch` | `Gray_Birch.usd`、树皮/叶片材质 | 10 | 54.00 MiB | `.collect.mapping.json` 指向 NVIDIA Omniverse Content S3 `/Assets/Vegetation/Trees/` | `SOURCE_IDENTIFIED_LICENSE_UNVERIFIED` | 禁止，待核验 NVIDIA 内容条款 |
| `Holly` | `Holly.usd`、冬青及树皮材质 | 10 | 81.42 MiB | `.collect.mapping.json` 指向 NVIDIA Omniverse Content S3 的 Shrub/Trees 路径 | `SOURCE_IDENTIFIED_LICENSE_UNVERIFIED` | 禁止，待核验 NVIDIA 内容条款 |
| `Norway_Spruce` | `Norway_Spruce.usd`、树皮和针叶材质 | 13 | 50.58 MiB | `.collect.mapping.json` 指向 NVIDIA Omniverse Content S3 `/Assets/Vegetation/Trees/` | `SOURCE_IDENTIFIED_LICENSE_UNVERIFIED` | 禁止，待核验 NVIDIA 内容条款 |
| `Rock_obj` | `Rock.usd`、`big_rock.usd`、OBJ、苔藓岩石纹理及 HDRI 副本 | 36 | 234.35 MiB | 无 `.collect.mapping.json` | `SOURCE_UNKNOWN` | 禁止，待核验 |
| `Yew` | `Yew.usd` 及复用的树木材质 | 8 | 33.60 MiB | `.collect.mapping.json` 指向 NVIDIA Omniverse Content S3 的 Shrub/Trees 路径 | `SOURCE_IDENTIFIED_LICENSE_UNVERIFIED` | 禁止，待核验 NVIDIA 内容条款 |
| `textures` | forest floor、moss rock、树桩 PBR、`autumn_park_2k.hdr` | 45 | 400.66 MiB | 无统一映射或许可证文件 | `SOURCE_UNKNOWN` | 禁止，待核验 |
| `icon.png` | Extension 图标 | 1 | 2.30 MiB | 无逐项来源记录 | `SOURCE_UNKNOWN` | 禁止，待核验 |
| `preview.png` | Extension 预览图 | 1 | 42.48 KiB | 无逐项来源记录 | `SOURCE_UNKNOWN` | 禁止，待核验 |

### 4.1 目录内容摘要

| 目录 | SHA-256 目录摘要 |
|---|---|
| `Blueberry_obj` | `6e61588e3d7ff1ef57a233e7dbaab8c79e147e581e634b927dd1769cc19e8b55` |
| `Bush_obj` | `5d54dfa2a17a1b855a19a04afc1ebe568dbd6796f494041ec04f39bf8221c4cf` |
| `Douglas_Fir` | `e17e012c817e49661c3d8287bb7afe5b34b1384e8e7c56ce1772089c4c76416d` |
| `Grass_Short_B` | `f19f7eeba9d5430c0bb59f9bc52e302ebd76ec2c76e89dbddcb7b760ed1c570c` |
| `Gray_Birch` | `a6dfb7ed9c0789a70d6c5cec7a86d4d3538fd55a6fff49aff7c2674061e59298` |
| `Holly` | `10528d5f3d39c72984fe03ea947c8aff80dd87674c53eaf2da65b3435e35a29b` |
| `Norway_Spruce` | `2bcc811f326802e04cdfaa845ee92905b8751d20b6d127944d0a968ed0a8092d` |
| `Rock_obj` | `13d1e295e85d2c6086d2743931acf91e4d0483676557163d27c52a3668b6603a` |
| `Yew` | `0a2aba63493e51b46c4d385971bd1982b5e11f4d829fb4b7cfaaf373571f3f92` |
| `textures` | `1338c094b22c0c13b5521f160c4d4dbbc2f8cdb8024172e40b9cc1bb473aed60` |

单文件摘要：

- `icon.png`: `73f32b5ed16a58c1512abd3eae8f611b4042286aba768414e2525feccabb211c`
- `preview.png`: `fbd0d1013bdb175b46cc47ea7cf43e189e239082bf95398bb54fa190da50186f`

## 5. 上游映射文件能够证明什么

以下目录包含 `.collect.mapping.json`：

- `Douglas_Fir`；
- `Grass_Short_B`；
- `Gray_Birch`；
- `Holly`；
- `Norway_Spruce`；
- `Yew`。

这些文件记录了 `omniverse-content-production.s3-us-west-2.amazonaws.com` 的原始 URL，以及收集前后的 SHA-1。它们能够支持资产来源追踪和完整性检查，但没有同时提供许可证名称、许可证正文或明确的再分发授权，因此本项目不能仅凭 URL 把这些资产认定为 MIT 或 CC0。

## 6. 已确认的缺失资产

`terrain.generator/terrain/generator/generator_logic.py` 的资产注册表引用了：

```text
Switchgrass/Switchgrass.usd
Container_J01/Container_J01_126x120x133cm_PR_V_NVD_01.usd
```

这两个文件在固定 commit 的 `terrain.generator/data` 中不存在，状态为 `MISSING`。适配阶段必须在扩展启动时验证资产注册表，并默认关闭对应生成功能，不能把它们当成完整资产交付的一部分。

此外，`medium.usd` 和 `hard.usd` 存在 `block.usd` 缺失引用，将在阶段 1 的 USD 兼容处理中修复。

## 7. 大型资产交付决策

### 7.1 选定方案

采用**固定 commit 的外部稀疏 Git checkout**：

```text
external_assets/
└── lariad_offroad_nav/
    ├── assets/
    ├── terrain.generator/
    ├── LICENSE
    └── README.md
```

原因：

- 不把约 1 GB 生成器资产写入当前项目 Git 历史；
- 不让这些资产进入 Docker build context 和镜像层；
- 保留上游目录结构，便于当前 USD 相对引用和后续审计；
- checkout 具有固定 commit，可在不同开发机复现；
- 后续可在不改变代码结构的情况下替换为对象存储制品。

### 7.2 获取方法

```bash
./scripts/tools/fetch_lariad_assets.sh
```

也可以显式指定目标目录：

```bash
./scripts/tools/fetch_lariad_assets.sh /data/sim-assets/lariad_offroad_nav
```

脚本行为：

1. 使用 blob filter 创建 Git checkout；
2. 只检出 `assets`、`terrain.generator`、`LICENSE` 和 `README.md`；
3. checkout 固定 commit；
4. 验证最终 `HEAD`；
5. 如果目标目录已经存在但版本不符，停止且不删除任何内容。

由于所需二进制资产本身接近 1 GB，首次下载仍需要相应网络流量和磁盘空间；稀疏 checkout 主要避免下载导航栈等无关工作区内容。

### 7.3 后续对象存储方案

当团队需要 CI 或离线交付时，可以从已审计 checkout 构建版本化制品，例如：

```text
lariad-terrain-assets-69640cf.tar.zst
lariad-terrain-assets-69640cf.sha256
```

制品进入对象存储前仍需完成第三方资产授权确认。当前阶段不生成或发布该制品。

## 8. 干净环境复现步骤

```bash
git clone https://github.com/autowarefoundation/autoware_off-road_sim
cd autoware_off-road_sim
./scripts/tools/fetch_lariad_assets.sh
./docker/build.sh
```

版本核验：

```bash
git -C external_assets/lariad_offroad_nav rev-parse HEAD
```

期望输出：

```text
69640cf19eec1d90214ed4a3615788dd5dc1e3c4
```

Isaac Sim 的固定 tag 和 commit 记录在 [lariad_versions.yaml](lariad_versions.yaml)，Docker 构建会在下载后执行完整 commit 比对，不匹配则直接失败。

## 9. 资产进入公开交付前的复核清单

- 找到资产原始发布页面和作者；
- 保存许可证名称、正文或权利人授权；
- 明确是否允许修改、商用、再分发和嵌入生成场景；
- 确认生成的合成数据是否受到额外限制；
- 保存下载日期、原始文件哈希和转换记录；
- 更新本清单状态；
- 只有状态变为明确允许再分发后，才能进入公开场景包。
