"""disf: MuJoCo Robotic Manipulation focused on Geometric Control"""
import os
from pathlib import Path

# package root
ROOT_DIR = str(Path(__file__).parent.absolute())

# package project
PROJECT_DIR = str(Path(ROOT_DIR).parent.absolute())

# models
MODELS_DIR = os.path.join(ROOT_DIR, "models")

# assets
ASSETS_DIR = os.path.join(ROOT_DIR, "assets")
