from xml.etree import ElementTree as ET

from .shapes import Shape


class Origin(ET.Element):
    def __init__(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> None:
        super().__init__("origin", xyz=f"{x} {y} {z}", rpy=f"{roll} {pitch} {yaw}")

    @classmethod
    def Zero(cls):
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    @classmethod
    def Trans(cls, x: float, y: float, z: float):
        return cls(x, y, z, 0.0, 0.0, 0.0)

    @classmethod
    def Rot(cls, roll: float, pitch: float, yaw: float):
        return cls(0.0, 0.0, 0.0, roll, pitch, yaw)


class Mass(ET.Element):
    def __init__(self, value: float) -> None:
        assert value > 0.0

        super().__init__("mass", value=f"{value}")


class Inertia(ET.Element):
    def __init__(
        self, ixx: float, ixy: float, ixz: float, iyy: float, iyz: float, izz: float
    ) -> None:
        assert ixx > 0.0
        assert iyy > 0.0
        assert izz > 0.0

        super().__init__(
            "inertia",
            ixx=f"{ixx}",
            ixy=f"{ixy}",
            ixz=f"{ixz}",
            iyy=f"{iyy}",
            iyz=f"{iyz}",
            izz=f"{izz}",
        )


class Geometry(ET.Element):
    def __init__(self, shape: Shape) -> None:
        super().__init__("geometry")

        self.append(shape)


class Collision(ET.Element):
    def __init__(self, origin: Origin, geometry: Geometry) -> None:
        super().__init__("collision")

        self.append(origin)
        self.append(geometry)


class Visual(ET.Element):
    def __init__(self, origin: Origin, geometry: Geometry) -> None:
        super().__init__("visual")

        self.append(origin)
        self.append(geometry)


class Inertial(ET.Element):
    def __init__(self, mass: Mass, origin: Origin, inertia: Inertia) -> None:
        super().__init__("inertial")

        self.append(mass)
        self.append(origin)
        self.append(inertia)


class Link(ET.Element):
    def __init__(
        self,
        name: str,
        inertial: Inertial = None,
        visual: Visual = None,
        collision: Collision = None,
    ) -> None:
        super().__init__("link", name=name)

        if inertial:
            self.append(inertial)

        if visual:
            self.append(visual)

        if collision:
            self.append(collision)


class Parent(ET.Element):
    def __init__(self, link: str) -> None:
        super().__init__("parent", link=link)


class Child(ET.Element):
    def __init__(self, link: str) -> None:
        super().__init__("child", link=link)


class Limit(ET.Element):
    def __init__(
        self, upper: float, lower: float, velocity: float, effort: float
    ) -> None:
        super().__init__(
            "limit",
            upper=f"{upper}",
            lower=f"{lower}",
            velocity=f"{velocity}",
            effort=f"{effort}",
        )


class Joint(ET.Element):
    def __init__(
        self,
        name: str,
        type: str,
        parent: str,
        child: str,
        origin: Origin = None,
        limit: Limit = None,
    ):
        super().__init__("joint", name=name, type=type)

        self.append(Parent(parent))
        self.append(Child(child))

        if origin:
            self.append(origin)

        if limit:
            self.append(limit)


class Transmission(ET.Element):
    POSITION = "hardware_interface/PositionJointInterface"
    VELOCITY = "hardware_interface/VelocityJointInterface"
    EFFORT = "hardware_interface/EffortJointInterface"  # 力制御でのみ反作用が考慮される

    def __init__(
        self, joint_name: str, interface: str = POSITION, reduction: float = 1.0
    ):
        assert interface in {self.POSITION, self.VELOCITY, self.EFFORT}
        assert reduction >= 1.0

        super().__init__("transmission", name=f"{joint_name}_trans")

        ET.SubElement(self, "type").text = "transmission_interface/SimpleTransmission"

        joint = ET.SubElement(self, "joint", {"name": joint_name})
        ET.SubElement(joint, "hardwareInterface").text = interface

        actuator = ET.SubElement(self, "actuator", {"name": f"{joint_name}_actuator"})
        ET.SubElement(actuator, "hardwareInterface").text = interface
        ET.SubElement(actuator, "mechanicalReduction").text = f"{reduction}"
