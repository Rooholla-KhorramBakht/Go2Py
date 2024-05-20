# from setuptools import setup

# setup()

from setuptools import find_packages
from distutils.core import setup

setup(
    name='Go2Py',
    version='1.0.0',
    author='Rooholla Khorrambakht',
    license="BSD-3-Clause",
    packages=find_packages(),
    author_email='rk4342@nyu.edu',
    description='Toolkit for deployment using Unitree Go2.',
    install_requires=[
                      ]
)