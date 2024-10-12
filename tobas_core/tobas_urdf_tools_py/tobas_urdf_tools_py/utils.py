import re
from xml.etree import ElementTree as ET
from xml.dom import minidom
from typing import Deque, Tuple
from collections import deque


def remove_elements_with_tag(root: ET.Element, tag: str) -> None:
    """特定のタグを持つ要素を全て削除する．"""
    # 要素とその親を追跡するためのdeque
    queue: Deque[Tuple[ET.Element, ET.Element]] = deque([(root, None)])

    while queue:
        current, parent = queue.popleft()

        # 子要素をqueueに追加
        queue.extend((child, current) for child in current)

        # 対象のタグに一致する要素を削除
        if current.tag == tag and parent is not None:
            parent.remove(current)


def remove_specific_element(root: ET.Element, element_to_remove: ET.Element) -> None:
    """ツリー全体を捜査し，特定の要素を削除する．"""
    # 要素とその親を追跡するためのdeque
    queue: Deque[Tuple[ET.Element, ET.Element]] = deque([(root, None)])

    while queue:
        current, parent = queue.popleft()

        # 子要素をqueueに追加
        queue.extend((child, current) for child in current)

        # 削除する要素を見つけた場合
        if current is element_to_remove:
            if parent is not None:
                parent.remove(current)
            return


def prettify(elem: ET.Element) -> str:
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(elem, "utf-8")
    reparsed = minidom.parseString(rough_string)
    pretty = re.sub(r"[\t ]+\n", "", reparsed.toprettyxml(indent="\t"))
    pretty = pretty.replace(">\n\n\t<", ">\n\t<")
    return pretty
