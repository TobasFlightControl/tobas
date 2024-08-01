import os
from glob import glob
from typing import List
from setuptools import setup

pkg_name = "tobas_kdl_conversions_py"

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + pkg_name]))
data_files.append(("share/" + pkg_name, ["package.xml"]))


def package_files(directory: str, data_files: List[str]) -> List[str]:
    for path, _, filenames in os.walk(directory):
        for _ in filenames:
            data_files.append(("share/" + pkg_name + "/" + path, glob(path + "/**/*.*", recursive=True)))
    return data_files


# data_files = package_files("launch/", data_files)

setup(
    name=pkg_name,
    version="0.0.0",
    packages=[pkg_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Masayoshi Dohi",
    maintainer_email="masa0u0masa1215@gmail.com",
    description="TODO",
    license="TODO",
    tests_require=["pytest"],
    # entry_points={
    #     "console_scripts": [
    #         "my_node = tobas_kdl_conversions_py.my_node:main",
    #     ],
    # },
)
