import os
import os.path as osp
from glob import glob
from typing import List
from setuptools import setup, find_packages

pkg_name = osp.basename(osp.abspath(osp.curdir))


def package_files(directory: str, data_files: List[str]) -> List[str]:
    for path, _, filenames in os.walk(directory):
        for _ in filenames:
            data_files.append((f"share/{pkg_name}/{path}", glob(path + "/**/*.*", recursive=True)))
    return data_files


data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + pkg_name]))
data_files.append(("share/" + pkg_name, ["package.xml"]))
# data_files = package_files("launch/", data_files)


console_scripts = []
for node_file in glob(f"scripts/*.py"):
    node_name = osp.basename(osp.splitext(node_file)[0])
    if node_name == "__init__":
        continue
    console_scripts.append(f"{node_name} = scripts.{node_name}:main")


setup(
    name=pkg_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Masayoshi Dohi",
    maintainer_email="masa0u0masa1215@gmail.com",
    description="TODO",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": console_scripts},
)
