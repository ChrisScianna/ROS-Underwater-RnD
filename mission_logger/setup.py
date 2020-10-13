from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mission_logger'],
    package_dir={'': 'src'}
)

setup(**d)
