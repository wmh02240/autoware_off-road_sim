EXTENSION_ID = "lawn.terrain.generator"
EXTENSION_NAME = "Lawn Terrain Generator"
DEFAULT_PARENT_PATH = "/World/GeneratedForest"
ASSET_ROOT_SETTING = f"/exts/{EXTENSION_ID}/assetRoot"
ASSET_ROOT_ENV = "LARIAD_TERRAIN_ASSET_ROOT"

ASSET_REGISTRY = {
    "Birch": {"file": "Gray_Birch/Gray_Birch.usd", "kind": "tree", "base_scale": 0.01},
    "Spruce": {"file": "Norway_Spruce/Norway_Spruce.usd", "kind": "tree", "base_scale": 0.01},
    "Pine": {"file": "Douglas_Fir/Douglas_Fir.usd", "kind": "tree", "base_scale": 0.01},
    "Rock": {"file": "Rock_obj/Rock.usd", "kind": "rock", "base_scale": 1.0},
    "Blueberry": {"file": "Blueberry_obj/Blueberry.usd", "kind": "vegetation", "base_scale": 1.0},
    "Bush": {"file": "Bush_obj/Bush.usd", "kind": "vegetation", "base_scale": 1.0},
    "Grass": {"file": "Grass_Short_B/Grass_Short_B.usd", "kind": "vegetation", "base_scale": 0.01},
    "Switchgrass": {"file": "Switchgrass/Switchgrass.usd", "kind": "vegetation", "base_scale": 0.01},
    "Container": {"file": "Container_J01/Container_J01_126x120x133cm_PR_V_NVD_01.usd", "kind": "object", "base_scale": 0.01},
}

