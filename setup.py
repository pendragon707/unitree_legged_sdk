from setuptools import setup, find_packages

setup(
    name="unitree-legged-sdk",
    version="0.0.0",  # Required
    
    packages=find_packages(where="ucl"),  # Required
    python_requires=">=3.7, <4",
    install_requires=[
        "crcmod",
        "numpy"
    ]
)