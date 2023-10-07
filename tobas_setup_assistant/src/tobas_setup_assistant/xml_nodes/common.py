from xml.etree import ElementTree as ET
from typing import List


class ListElement(ET.Element):
    def __init__(self, name: str, items: List) -> None:
        super().__init__(name)

        for item in items:
            ET.SubElement(self, "item").text = str(item)
