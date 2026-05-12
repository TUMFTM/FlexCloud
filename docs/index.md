# FlexCloud

**Georeferencing of Point Cloud Maps**

![License](https://img.shields.io/badge/license-Apache%202.0-blue)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
![pip](https://img.shields.io/badge/pip-Ubuntu%2024.04%20x86__64-E95420?logo=ubuntu&logoColor=white)
[![PyPI](https://github.com/TUMFTM/FlexCloud/actions/workflows/pypi.yml/badge.svg)](https://github.com/TUMFTM/FlexCloud/actions/workflows/pypi.yml)
[![arXiv](https://img.shields.io/badge/arXiv-1234.56789-b31b1b.svg)](https://arxiv.org/abs/2502.00395)
[![DOI:10.5220/0013405400003941](https://img.shields.io/badge/DOI-10.5220/0013405400003941-00629B.svg)](https://doi.org/10.5220/0013359600003941)


![FlexCloud visualization](assets/viz.gif)

FlexCloud enables the georeferencing of an existing point cloud map created only from perception sensor data (e.g. LiDAR) by leveraging the corresponding GNSS data. Using the concept of *rubber-sheeting* from cartography, the tool also accounts for accumulated errors during map creation and rectifies the map.

## Functionalities

- **Direct & modular** — keyframe interpolation and georeferencing are exposed as two independent CLI tools that can be combined with any SLAM pipeline.
- **Rubber-sheet drift correction** — piecewise linear 3D rubber-sheeting via
  Delaunay triangulation corrects accumulated SLAM drift.
- **Reference-data agnostic** — accepts text files or
  ROS 2 bags (`NavSatFix` / `Odometry`).
- **Built-in evaluation** — RMSE, mean, median and per-segment deviation visualised
  in the [Rerun](https://rerun.io/) viewer.

## Citation

If you use FlexCloud in academic work, please cite the preprint:

```bibtex
@conference{leitenstern2025flexcloud,
author={Maximilian Leitenstern and Marko Alten and Christian Bolea-Schaser and Dominik Kulmer and Marcel Weinmann and Markus Lienkamp},
title={FlexCloud: Direct, Modular Georeferencing and Drift-Correction of Point Cloud Maps},
booktitle={Proceedings of the 11th International Conference on Vehicle Technology and Intelligent Transport Systems - Volume 1: VEHITS},
year={2025},
pages={157-165},
publisher={SciTePress},
organization={INSTICC},
doi={10.5220/0013359600003941},
isbn={978-989-758-745-0},
}
```

See [Functionality & References](functionality.md#references) for more details and related publications.
