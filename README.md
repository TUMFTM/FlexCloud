<div align="center">

<h1>FlexCloud</h1>

Georeferencing of Point Cloud Maps

![License](https://img.shields.io/badge/license-Apache%202.0-blue)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
![pip](https://img.shields.io/badge/pip-Ubuntu%2024.04%20x86__64-E95420?logo=ubuntu&logoColor=white)
[![PyPI](https://github.com/TUMFTM/FlexCloud/actions/workflows/pypi.yml/badge.svg)](https://github.com/TUMFTM/FlexCloud/actions/workflows/pypi.yml)
[![arXiv](https://img.shields.io/badge/arXiv-1234.56789-b31b1b.svg)](https://arxiv.org/abs/2502.00395)
[![DOI:10.5220/0013405400003941](https://img.shields.io/badge/DOI-10.5220/0013405400003941-00629B.svg)](https://doi.org/10.5220/0013359600003941)

<img src="docs/assets/viz.gif" width="800"/>
</div>

**Full documentation: [https://TUMFTM.github.io/FlexCloud](https://TUMFTM.github.io/FlexCloud)**

<h2>Installation</h2>

```bash
pip install flexcloud
```

> **Supported platform:** Ubuntu 24.04 on x86_64 with CPython 3.12. The published PyPI wheel is built for `manylinux_2_39_x86_64` / `cp312` only. On other platforms, use the [Docker image](https://ghcr.io/tumftm/pointcloudcrafter) or build from source.

<h2>Usage</h2>

All algorithm parameters are CLI flags with reasonable defaults.

**Keyframe Interpolation**

```bash
flexcloud-keyframe-interpolation -h

ros2 run flexcloud keyframe_interpolation
```

**Georeferencing**

```bash
flexcloud-georeferencing -h

ros2 run flexcloud georeferencing
```

<h2>Documentation </h2>

For more details on the implementation and available features, refer to the full documentation hosted on GitHub pages: [https://TUMFTM.github.io/FlexCloud](https://TUMFTM.github.io/FlexCloud)

<h2>Test Data </h2>

The data was recorded by the [TUM Autonomous Motorsport Team](https://www.mos.ed.tum.de/ftm/forschungsfelder/team-av-perception/tum-autonomous-motorsport/) during the [Abu Dhabi Autonomous Racing League](https://a2rl.io/) 2025.
The LiDAR/SLAM trajectory is created using [glim](https://github.com/koide3/glim).
The reference trajectory presents raw data from the RTK-corrected GNSS-signal of the vehicle.

<h2>Developers </h2>

* [Maximilian Leitenstern](mailto:maxi.leitenstern@tum.de),
Institute of Automotive Technology,
School of Engineering and Design,
Technical University of Munich,
85748 Garching,
Germany
* Marko Alten (student research project)
* Christian Bolea-Schaser (student research project)

<h2>Citation </h2>

If you use this repository for any academic work, please consider citing our paper (preprint):

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
