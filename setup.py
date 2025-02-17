from setuptools import setup
import os

# Read dependencies from requirements.txt
def read_requirements():
    req_path = os.path.join(os.path.dirname(__file__), "requirements.txt")
    with open(req_path, encoding="utf-8") as f:
        return f.read().splitlines()

setup(
    install_requires=read_requirements(),
)
