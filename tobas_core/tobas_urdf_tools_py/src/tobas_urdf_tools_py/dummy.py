from .core import *
from .shapes import *


class DummyInertialLink(Link):
    MASS = 1e-9
    INERTIA = 1e-9

    def __init__(self, name: str) -> None:
        super().__init__(
            name=name,
            inertial=Inertial(
                mass=Mass(value=self.MASS),
                origin=Origin.Zero(),
                inertia=Inertia(
                    ixx=self.INERTIA,
                    ixy=0.0,
                    ixz=0.0,
                    iyy=self.INERTIA,
                    iyz=0.0,
                    izz=self.INERTIA,
                ),
            ),
            visual=Visual(origin=Origin.Zero(), geometry=Geometry(Mesh())),
        )


class DummyVisualLink(Link):
    def __init__(self, name: str) -> None:
        super().__init__(
            name=name,
            visual=Visual(
                origin=Origin.Zero(),
                geometry=Geometry(Mesh("package://tobas_description/meshes/empty.dae")),
            ),
        )
