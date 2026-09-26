import os.path as osp
from glob import glob
from setuptools import setup, find_packages

pkg_name = "tobas_ssh_server"

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + pkg_name]))
data_files.append(("share/" + pkg_name, ["package.xml"]))

console_scripts = []
for node_file in glob(f"{pkg_name}/*_node.py"):
    node_name = osp.basename(osp.splitext(node_file)[0])
    console_scripts.append(f"{node_name} = {pkg_name}.{node_name}:main")

setup(
    name=pkg_name,
    version="2.16.4",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    entry_points={"console_scripts": console_scripts},
)
