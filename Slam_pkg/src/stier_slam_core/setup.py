from setuptools import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    packages=["stier_slam_core"],
    package_dir={"": "src"},
    version="0.1.0",
))
