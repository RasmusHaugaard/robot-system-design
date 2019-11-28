#!/usr/bin/env python3
from setuptools import setup

setup(
    name='rsd',
    version='0.0.1',
    install_requires=[
        'numpy',
        'requests',
        'pymodbus',
        'matplotlib',
        'redis',
        'PySide2',
        'PyQt5',
    ]
)
