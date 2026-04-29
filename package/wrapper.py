import os
import sys
from pathlib import Path


def _launch(bin_name):
    pkg_root = Path(__file__).parent.resolve()

    bin_path = pkg_root / "bin" / bin_name
    lib_path = pkg_root / "lib"
    audit_libs = pkg_root.parent / "flexcloud.libs"

    env = os.environ.copy()

    ld_paths = [str(lib_path), str(audit_libs)]
    existing = env.get("LD_LIBRARY_PATH", "")
    if existing:
        ld_paths.append(existing)
    env["LD_LIBRARY_PATH"] = ":".join(ld_paths)
    env["AMENT_PREFIX_PATH"] = str(pkg_root)
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

    os.execve(bin_path, [str(bin_path)] + sys.argv[1:], env)


def run_keyframe_interpolation():
    _launch("keyframe_interpolation")


def run_georeferencing():
    _launch("georeferencing")
