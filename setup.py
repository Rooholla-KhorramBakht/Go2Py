# from setuptools import setup

# setup()

from setuptools import find_packages
from distutils.core import setup

setup(
    name='Go2Py',
    version='1.0.0',
    author='Rooholla Khorrambakht, Bolun Dai',
    license="BSD-3-Clause",
    packages=find_packages(),
    author_email='rk4342@nyu.edu, bd1555@nyu.edu',
    description='A Python interface and simulation environemtn for the Unitree Go2 robot.',
    install_requires=[
    ]
)
