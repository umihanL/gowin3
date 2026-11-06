from setuptools import find_packages
from setuptools import setup

setup(
    name='rescue_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('rescue_interfaces', 'rescue_interfaces.*')),
)
