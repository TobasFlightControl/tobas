from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["wpa_supplicant_parser_py"],
    package_dir={"": "src"},
)

setup(**setup_args)
