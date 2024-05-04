#!/bin/bash

set -e

pip install \
    "torch==2.2.1" \
    "torchvision==0.17.1" \
    "numpy>=1.26.4" \
    "pybullet>=3.2.6" \
    "proxsuite==0.6.3" \
    "ipykernel==6.29.3" \
    "open3d==0.18.0"