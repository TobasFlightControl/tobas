from abc import ABC
from xml.etree import ElementTree as ET


class Shape(ABC, ET.Element):
    pass


class Box(Shape):
    def __init__(self, size_x: float, size_y: float, size_z: float) -> None:
        assert size_x > 0.0
        assert size_y > 0.0
        assert size_z > 0.0

        super().__init__("box", size=f"{size_x} {size_y} {size_z}")


class Cylinder(Shape):
    def __init__(self, length: float, radius: float) -> None:
        assert length > 0.0
        assert radius > 0.0

        super().__init__("cylinder", length=f"{length}", radius=f"{radius}")


class Sphere(Shape):
    def __init__(self, radius: float) -> None:
        assert radius > 0.0

        super().__init__("sphere", radius=f"{radius}")


class Mesh(Shape):
    def __init__(
        self,
        filename: str,
        scale_x: float = 1.0,
        scale_y: float = 1.0,
        scale_z: float = 1.0,
    ) -> None:
        assert scale_x > 0.0
        assert scale_y > 0.0
        assert scale_z > 0.0

        super().__init__("mesh", filename=filename, scale=f"{scale_x} {scale_y} {scale_z}")
