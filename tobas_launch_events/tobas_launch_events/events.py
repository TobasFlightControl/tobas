from launch import Event


class ComponentContainersReady(Event):
    pass


class CommonNodesReady(Event):
    pass


class HardwareInterfacesReady(Event):
    pass
