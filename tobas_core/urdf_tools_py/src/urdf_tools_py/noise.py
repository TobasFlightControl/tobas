from abc import ABC
from xml.etree import ElementTree as ET


class Noise(ABC, ET.Element):
    pass


class GaussianNoise(Noise):
    def __init__(self, mean: float, stddev: float) -> None:
        assert stddev >= 0.0

        super().__init__("noise")

        ET.SubElement(self, "type").text = "gaussian"
        ET.SubElement(self, "mean").text = f"{mean}"
        ET.SubElement(self, "stddev").text = f"{stddev}"
