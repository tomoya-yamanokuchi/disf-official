import xml.etree.ElementTree as ET
from service import indent_tree


def generate_mujoco_grasping_scene_with_ycb(
        output_file: str,
        object_name: str,
        base_path  : str,
    ):
    """
    指定されたオブジェクトを含むMuJoCo XMLファイルを生成する。
    改行とインデントを含む形式で保存。

    Args:
        output_file (str): 生成されるXMLファイルのパス
        object_name (str): ロードするオブジェクトの名前
        base_path (str): オブジェクトのベースディレクトリパス
    """
    mujoco = ET.Element("mujoco", {"model": "panda scene"})
    ET.SubElement(mujoco, "include", {"file": "panda_arm_with_hand.xml"})
    obj_path = f"{base_path}/{object_name}/tsdf/textured/textured.xml"
    ET.SubElement(mujoco, "include", {"file": obj_path})
    ET.SubElement(mujoco, "statistic", {"center": "0.3 0 0.4", "extent": "1"})

    visual = ET.SubElement(mujoco, "visual")
    ET.SubElement(visual, "headlight", {"diffuse": "0.6 0.6 0.6", "ambient": "0.3 0.3 0.3", "specular": "0 0 0"})
    ET.SubElement(visual, "rgba", {"haze": "0.15 0.25 0.35 1"})
    ET.SubElement(visual, "global", {"azimuth": "120", "elevation": "-20", "offwidth": "1920", "offheight": "1080"})

    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(asset, "texture", {"type": "skybox", "builtin": "gradient", "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0", "width": "512", "height": "3072"})
    ET.SubElement(asset, "texture", {"type": "2d", "name": "groundplane", "builtin": "checker", "mark": "edge",
                                      "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "markrgb": "0.8 0.8 0.8",
                                      "width": "300", "height": "300"})
    ET.SubElement(asset, "material", {"name": "groundplane", "texture": "groundplane", "texuniform": "true", "texrepeat": "5 5", "reflectance": "0.2"})
    ET.SubElement(asset, "texture", {"name": "cube_surface", "file": "./assets/texture/iris_block.png", "gridsize": "3 4", "gridlayout": ".U..LFRB.D.."})
    ET.SubElement(asset, "material", {"name": "obj_material", "texture": "cube_surface", "specular": "1", "shininess": "0.0", "rgba": "1 1 1 1"})

    worldbody = ET.SubElement(mujoco, "worldbody")
    ET.SubElement(worldbody, "light", {"pos": "0 0 1.5", "dir": "0 0 -1", "directional": "true"})
    floor = ET.SubElement(worldbody, "body", {"name": "floor"})
    ET.SubElement(floor, "geom", {"name": "floor", "size": "0 0 0.05", "type": "plane", "material": "groundplane"})
    table = ET.SubElement(worldbody, "body", {"name": "table", "pos": "0 0 0.1"})
    ET.SubElement(table, "geom", {"name": "table", "size": "0.1 0.1 0.1", "type": "box", "rgba": "1 1 1 0.3", "friction": "0.001 0.005 0.0001"})

    contact = ET.SubElement(mujoco, "contact")
    exclusions = [
        ("floor", "table"), ("floor", "link5"), ("floor", "link6"), ("floor", "link7"),
        ("floor", "hand"), ("floor", "right_finger"), ("floor", "left_finger"),
        ("table", "link5"), ("table", "link6"), ("table", "link7"),
        ("table", "hand"), ("table", "right_finger"), ("table", "left_finger")
    ]
    for body1, body2 in exclusions:
        ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})

    keyframe = ET.SubElement(mujoco, "keyframe")
    ET.SubElement(keyframe, "key", {"name": "home",
                                    "qpos": "0 0 0 -1.57079 0 1.57079 -0.7853       0.04 0.04       0 0 0.196   1 0 0 0",
                                    "ctrl": "0 0 0 -1.57079 0 1.57079 -0.7853 0.04"})

    # ツリーにインデントを適用
    indent_tree(mujoco)

    # XMLツリーをファイルに保存
    tree = ET.ElementTree(mujoco)
    tree.write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"XMLファイル '{output_file}' が生成されました！")




# ユースケース例
if __name__ == "__main__":
    object_name = "065-j_cups"
    base_path   = "/home/cudagl/padova2ndvisit/ycb-tools/models/ycb"
    output_file = "/home/cudagl/padova2ndvisit/mujoco_menagerie/franka_emika_panda_original/generated_scene_pretty.xml"
    generate_mujoco_grasping_scene_with_ycb(output_file, object_name, base_path)
