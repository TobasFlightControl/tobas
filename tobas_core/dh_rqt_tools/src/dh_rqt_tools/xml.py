import re
from xml.etree import ElementTree as ET
from xml.dom import minidom


def prettify(elem: ET.Element) -> str:
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(elem, "utf-8")
    reparsed = minidom.parseString(rough_string)
    pretty = re.sub(r"[\t ]+\n", "", reparsed.toprettyxml(indent="\t"))
    pretty = pretty.replace(">\n\n\t<", ">\n\t<")
    return pretty


def prettify_and_save(elem: ET.Element, path: str) -> None:
    """Save a pretty-printed XML string for the Element."""
    pretty_xml = prettify(elem)
    with open(path, mode="w") as f:
        f.write(pretty_xml)
