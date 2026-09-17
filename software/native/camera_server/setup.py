# setup.py
from setuptools import setup, find_packages

setup(
    name="camera_server",
    version="0.0.1",
    packages=find_packages(),
    install_requires=[],
    entry_points={
        "console_scripts": [
            "camera_server=src.main:main",
        ],
    },
)
