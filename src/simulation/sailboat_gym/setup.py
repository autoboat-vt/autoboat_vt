"""Setups the project."""

import pathlib

from setuptools import setup, find_packages


CWD = pathlib.Path(__file__).absolute().parent


def get_version():
    """Gets the sailboat-gym version."""
    path = CWD / "sailboat_gym" / "__init__.py"
    content = path.read_text()

    for line in content.splitlines():
        if line.startswith("__version__"):
            return line.strip().split()[-1].strip().strip('"').strip("'")
    raise RuntimeError("bad version data in __init__.py")


def get_description():
    """Gets the description from the readme."""
    with open("README.md") as fh:
        long_description = ""
        header_count = 0
        for line in fh:
            if line.startswith("##"):
                header_count += 1
            if header_count < 1:
                if not line.startswith("#") and not line.startswith("!"):
                    long_description += line
            else:
                break
    return long_description


setup(
    name="sailboat-gym",
    version=get_version(),
    description="A dynamic gym simulation environment specifically designed for sailboats.",
    long_description=get_description(),
    author="Lucas Marandat",
    author_email="lucas.mrdt+sailboat@gmail.com",
    license="MIT License",
    keywords=["Simulation", "Sailboat", "Gymnasium", "Gym"],
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    python_requires=">=3.7",
    url="https://github.com/lucasmrdt/sailboat-gym",
    project_urls={
        "Homepage": "https://github.com/lucasmrdt/sailboat-gym",
        "Repository": "https://github.com/lucasmrdt/sailboat-gym",
        "Documentation": "https://github.com/lucasmrdt/sailboat-gym/blob/main/DOCUMENTATION.md",
        "Bug Report": "https://github.com/lucasmrdt/sailboat-gym/issues",
    },
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "gymnasium==0.28.1",
        "msgpack_python==0.5.6",
        "pyzmq==25.0.2",
        "numpy==1.26.4",
        "pydantic",
        "tqdm",
        "imageio-ffmpeg",
        "docker",
        "moviepy",
        "multiexit",
    ],
    package_data={
        "sailboat_gym": [
            "docs/demo.gif",
            "docs/sailing_schema.png",
            "pkl/SailboatLSAEnv-v0_bounds_v_wind_1.pkl",
            "pkl/SailboatLSAEnv-v0_bounds_v_wind_2.pkl",
        ]
    },
)
