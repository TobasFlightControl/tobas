import os
from glob import glob
from typing import List
from setuptools import setup, find_packages


def package_files(directory: str, data_files: List[str]) -> List[str]:
    for path, _, filenames in os.walk(directory):
        for _ in filenames:
            data_files.append((f"share/tobas_f550_np_user_py/{path}", glob(path + "/**/*.*", recursive=True)))
    return data_files


data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/tobas_f550_np_user_py"]))
data_files.append(("share/tobas_f550_np_user_py", ["package.xml"]))
data_files = package_files("launch/", data_files)

console_scripts = [
    "user_node = tobas_f550_np_user_py.user_node:main",
    # Add executable Python scripts here
]

setup(
    name="tobas_f550_np_user_py",
    version="0.0.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="example",
    maintainer_email="example@gmail.com",
    description="Tobas user Python package for f550_np",
    license="BSD",
    tests_require=["pytest"],
    entry_points={"console_scripts": console_scripts},
)
