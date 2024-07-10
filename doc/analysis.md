# Analysis

## Overview

- export of various data by setting corresponding parameters in config-file
  - data is exported to `.txt` files that are then read by python-scripts
  - set export path in config-file
  - adjust import paths at the beginning of python-scripts if necessary
- analysis scripts in `/analysis`

## Analysis of Geometric Alignment

- visualization of initial trajectories, [Umeyama transformation](https://web.stanford.edu/class/cs273/refs/umeyama.pdf) and [Rubber-Sheet transformation](https://www.tandfonline.com/doi/abs/10.1559/152304085783915135)
- execute script `plot_traj_matching.py` in `/analysis`
- produces graphs shown in [alignment](alignment.md)
- calculation of deviation between trajectories based on euclidean distance of two points
