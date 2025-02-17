# Disentangled Iterative Surface Fitting for Contact-stable Grasp Planning (DISF)


![Demo Video](videos/grasp_execution_video_drill_high_resolution.gif)

## Overview

This is the official implementation for the paper entitled with "Disentangled Iterative Surface Fitting for Contact-stable Grasp Planning".

## 1-1. Preparation for docker run

make `data` directory in local envieonment to save data from DISF

```bash
cd ~ & mkdir data
```

## 1-2. Buld and run docker environment

```bash
cd ~
git clone https://github.com/tomoya-yamanokuchi/disf.git
cd ./disf
sh buld.sh
sh run.sh
```

## 2. Install packages with setup.py

The below procedures will be conducted inside the docker.

```bash
pip install -e .
```

## 3. Download YCB object dataset

```bash
python scripts/download_ycb_dataset.py
```

## 4. Apply pre-convex shape approximation

We use [CoACD] for pre-convex shape approximation inside the
[obj2mjcf].

Initial setup and check:
```bash
cd ~/disf/src/obj2mjcf/ & pip install -e .
ls /home/cudagl/.local/bin/obj2mjcf
export PATH=$PATH:/home/cudagl/.local/bin
obj2mjcf --help
```

Apply pre-convex shape approximation:

```bash
cd ~/disf/scripts/ & ./pre_convex_shape_approximation.sh
```

## 4. Grasp Planngin by DISF

We use [MuJoCo] as simulartor and use Franka Emika Panda environment from [MuJoCo Menagerie] for the grasp experiments.

You can try the grasp planning by `DISF`, `ISF` or `CMA`:

```bash
python ~/disf/test/disf_grasp.py
python ~/disf/test/visf_grasp.py
python ~/disf/test/cma_grasp.py
```

## Parameter Settings

You can change the experimental settings by changing parameters in the config files: `disf/src/config_loader/config`

```
config:
 - env              # mujoco simulation parameters
 - grasp_evaluation # grasp evaluation parameters for position and orientation
 - icp              # ICP filtering parameter
 - ik_solver        # inverse kinematics parameters for robot arm reaching
 - isf              # DISF/ISF related parameters
 - cma              # CMA-ES related parameters
 - point_cloud_data # data loading parameters for YCB-object point cloud data
```

[CoACD]: https://github.com/SarahWeiii/CoACD
[obj2mjcf]: https://github.com/kevinzakka/obj2mjcf
[MuJoCo]: https://github.com/deepmind/mujoco
[MuJoCo Menagerie]: https://github.com/deepmind/mujoco_menagerie

