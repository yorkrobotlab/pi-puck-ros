from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=['pi_puck_examples'], scripts=[], package_dir={'': 'src'})

setup(**d)
