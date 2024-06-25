import os.path as osp
import rospy
from xml.etree import ElementTree as ET
from jinja2 import Environment, FileSystemLoader


def get_drone_name() -> str:
    """URDFからドローンの名前を取得する．"""
    description = rospy.get_param("/robot_description")
    root = ET.fromstring(description)
    assert root.tag == "robot"

    name = root.get("name")
    return name if name else "unknown"


class TemplateGenerator:
    def __init__(self, dir: str) -> None:
        self._env = Environment(loader=FileSystemLoader(dir), trim_blocks=True, lstrip_blocks=True)

    def generate(self, items: dict, template_path: str, output_dir: str, overwrite: bool = True) -> None:
        assert template_path.endswith(".tpl")

        basename_without_ext = osp.basename(osp.splitext(template_path)[0])
        output_path = osp.join(output_dir, basename_without_ext)

        if not overwrite and osp.exists(output_path):
            return

        template = self._env.get_template(template_path)
        content = template.render(items)  # テンプレートにdict型で文字を埋め込む
        with open(output_path, "w") as f:
            f.write(content)
