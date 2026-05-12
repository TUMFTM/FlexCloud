# Installation

FlexCloud ships both as a PyPI wheel (recommended for most users) and as a buildable
source tree with a pre-built Docker image.

## Standalone pip-package

The simplest path:

```bash
pip install flexcloud
```

!!! tip "Use a virtual environment"
    Install into a virtual environment (or `pipx`) instead of your system Python to
    avoid dependency clashes and keep the install fully removable:

    ```bash
    python -m venv .venv
    source .venv/bin/activate
    pip install flexcloud
    ```

!!! note "Rerun viewer required for visualization"
    The visualization output is streamed to the [Rerun](https://rerun.io/) viewer,
    which is **not** pulled in by the wheel — you have to install it yourself on the
    host:

    ```bash
    pip install rerun-sdk
    ```

    Without it the algorithms still run, but you will not see any 3D output. The
    Docker image bundles its own Rerun viewer (see below), so this step is only
    needed for the standalone pip install.

**Verifying the installation**

```bash
flexcloud-keyframe-interpolation --help
flexcloud-georeferencing --help
```

Both commands should print their usage banner.

## Docker

A containerised build is provided for users who prefer not to install system
dependencies.

```bash
# build locally
./docker/build_docker.sh

# or pull a prebuilt image
docker pull ghcr.io/tumftm/flexcloud:latest

# run, mounting your data directory
./docker/run_docker.sh /your/local/directory/data
```

The container ships with a Rerun viewer instance; to use it, you need to activate the virtual environment in the container that it is installed in:

```bash
source /ros_ws/rerun/bin/activate
```

If you encounter graphics-driver issues you can launch a Rerun viewer on the host instead — see [Test Data & Visualization](test-data.md).
